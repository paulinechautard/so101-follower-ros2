#!/usr/bin/env python3
import math
import os
import struct
import threading
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from so101_interfaces.srv import DetectObjects

CALL_TIMEOUT_S = 5.0
MIN_BLOB_PX = 80
MIN_IOU = 0.6  # below that the box model doesn't explain the blob (failure)
MAX_CANDIDATES = 8  # blobs of the right colour tried, largest first
MAX_BLOBS = 30  # blobs looked at, largest first
BACKGROUND_RING_PX = 8  # width of the ring around a blob that should be background...
BACKGROUND_MIN = 0.6  # ...in this proportion at least
SELF_MARGIN_PX = 6  # grows the robot's own outline, for model and calibration errors
TAKEN_RATIO = 0.7  # a fit closer than this x the footprint's diagonal to another object is that one


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def load_stl(path):
    """Triangles of a binary or ASCII STL file, as an Nx3x3 array"""
    with open(path, 'rb') as f:
        data = f.read()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * count:  # binary
        records = np.frombuffer(data, dtype=np.dtype([
            ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')]), count=count, offset=84)
        return records['vertices'].astype(np.float64)
    vertices = [line.split()[1:4] for line in data.decode(errors='ignore').splitlines()
                if line.strip().startswith('vertex')]
    return np.array(vertices, dtype=np.float64).reshape(-1, 3, 3)


def link_meshes(urdf, links):
    """link name -> triangles of its visual meshes in the link's frame"""
    result = {}
    for link in ET.fromstring(urdf).iter('link'):
        if link.get('name') not in links:
            continue
        parts = []
        for visual in link.iter('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            filename = mesh.get('filename')
            if filename.startswith('package://'):
                package, _, rest = filename[len('package://'):].partition('/')
                filename = f'{get_package_share_directory(package)}/{rest}'
            elif filename.startswith('file://'):
                filename = filename[len('file://'):]
            triangles = load_stl(filename) * np.array([float(v) for v in mesh.get('scale', '1 1 1').split()])
            origin = visual.find('origin')
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            parts.append(triangles @ rpy_matrix(*rpy).T + xyz)
        if parts:
            result[link.get('name')] = np.concatenate(parts)
    return result


def hsv_mask(hsv, section):
    """Pixels in [hsv_min, hsv_max]. A hue range with min > max wraps around 180 (red)"""
    lo, hi = section['hsv_min'], section['hsv_max']
    if lo[0] <= hi[0]:
        return cv2.inRange(hsv, np.array(lo), np.array(hi))
    return (cv2.inRange(hsv, np.array(lo), np.array([180, hi[1], hi[2]])) |
            cv2.inRange(hsv, np.array([0, lo[1], lo[2]]), np.array(hi)))


def on_background(blob, background_mask):
    """Proportion of a ring around the blob that is background"""
    inside = np.zeros_like(background_mask)
    cv2.drawContours(inside, [blob], -1, 255, cv2.FILLED)
    size = 2 * BACKGROUND_RING_PX + 1
    ring = cv2.dilate(inside, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))) & ~inside
    return cv2.countNonZero(ring & background_mask) / max(cv2.countNonZero(ring), 1)


class Camera:

    def __init__(self, info):
        self.K = np.array(info.k).reshape(3, 3)
        self.D = np.array(info.d, dtype=float)
        self.fisheye = info.distortion_model == 'equidistant'
        if self.fisheye:
            self.D = self.D[:4].reshape(4, 1)

    def rays(self, pixels):
        """Nx2 pixels -> Nx3 directions in the optical frame (z = 1)"""
        pts = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
        if self.fisheye:
            norm = cv2.fisheye.undistortPoints(pts, self.K, self.D)
        else:
            norm = cv2.undistortPoints(pts, self.K, self.D)
        norm = norm.reshape(-1, 2)
        return np.hstack([norm, np.ones((len(norm), 1))])

    def project(self, points):
        """Nx3 points in the optical frame -> Nx2 pixels"""
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 1, 3)
        zero = np.zeros(3)
        if self.fisheye:
            px, _ = cv2.fisheye.projectPoints(pts, zero, zero, self.K, self.D)
        else:
            px, _ = cv2.projectPoints(pts, zero, zero, self.K, self.D)
        return px.reshape(-1, 2)


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        path = self.declare_parameter('world', '').value
        if not path:
            raise ValueError('parameter world (a world file) is required')
        with open(path) as f:
            scene = yaml.safe_load(f)
        self.frame = scene.get('frame_id', 'base_link')
        self.table_z = float(scene.get('table_z', 0.0))
        self.objects = {o['id']: o for o in scene['objects'] if 'detect' in o}
        if not self.objects:
            raise ValueError(f'{path}: no object has a detect: section')
        for obj in self.objects.values():
            d = obj['detect']
            for section in (d, d.get('background', {'hsv_min': [0] * 3, 'hsv_max': [0] * 3})):
                if len(section.get('hsv_min', [])) != 3 or len(section.get('hsv_max', [])) != 3:
                    raise ValueError(f"{obj['id']}: detect and detect.background need hsv_min and "
                                     'hsv_max, [h, s, v] each')
        self.settle = self.declare_parameter('settle_time', 0.3).value
        self.debug_dir = self.declare_parameter('debug_dir', '').value
        if self.debug_dir:
            os.makedirs(self.debug_dir, exist_ok=True)
        self.self_links = list(self.declare_parameter(
            'self_links', ['gripper_link', 'moving_jaw_so101_v1_link']).value)
        self.meshes = None  # link -> triangles from /robot_description

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.image = None
        self.info = None
        self.new_image = threading.Condition(self.lock)

        group = ReentrantCallbackGroup()
        self.create_subscription(Image, '/wrist_camera/image_raw', self.on_image, 2, callback_group=group)
        self.create_subscription(CameraInfo, '/wrist_camera/camera_info', self.on_info, 2, callback_group=group)
        self.create_subscription(
            String, '/robot_description', self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=group)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self.apply_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=group)
        self.get_client = self.create_client(GetPlanningScene, '/get_planning_scene', callback_group=group)
        self.create_service(DetectObjects, '~/detect', self.detect, callback_group=group)
        names = ', '.join(self.objects)
        self.get_logger().info(f'Ready: call ~/detect to locate {names}')

    def on_image(self, msg):
        with self.lock:
            self.image = msg
            self.new_image.notify_all()

    def on_description(self, msg):
        try:
            meshes = link_meshes(msg.data, self.self_links)
        except (OSError, ValueError, ET.ParseError) as e:
            self.get_logger().error(f'robot_description: {e}')
            return
        missing = set(self.self_links) - set(meshes)
        if missing:
            self.get_logger().warning(f"no visual mesh for {', '.join(sorted(missing))}: not masked")
        with self.lock:
            self.meshes = meshes

    def self_mask(self, shape, stamp, frame, camera):
        """Pixels where the robot's own links are, or an error message"""
        mask = np.zeros(shape, np.uint8)
        with self.lock:
            meshes = self.meshes
        if meshes is None:
            return 'no /robot_description yet'
        for link, triangles in meshes.items():
            try:
                tf = self.tf_buffer.lookup_transform(frame, link, stamp, timeout=Duration(seconds=1.0))
            except tf2_ros.TransformException as e:
                return f'tf: {e}'
            t = tf.transform.translation
            points = triangles.reshape(-1, 3) @ quaternion_matrix(tf.transform.rotation).T + [t.x, t.y, t.z]
            # Drop the triangles that go behind the camera (or too close to project)
            keep = np.all(points.reshape(-1, 3, 3)[:, :, 2] > 0.005, axis=1)
            if not np.any(keep):
                continue
            pixels = camera.project(points.reshape(-1, 3, 3)[keep].reshape(-1, 3))
            pixels = np.clip(pixels, -1e4, 1e4)
            cv2.fillPoly(mask, np.round(pixels.reshape(-1, 3, 2)).astype(np.int32), 255)
        size = 2 * SELF_MARGIN_PX + 1
        return cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size)))

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def wait_for_image(self, after):
        """The first image taken after `after` (the arm may still be moving before it)"""
        deadline = self.get_clock().now() + Duration(seconds=CALL_TIMEOUT_S)
        with self.lock:
            while self.image is None or self.info is None or Time.from_msg(self.image.header.stamp) < after:
                if self.get_clock().now() > deadline:
                    return None, None
                self.new_image.wait(timeout=0.1)
            return self.image, self.info

    def detect(self, request, response):
        unknown = [i for i in request.object_ids if i not in self.objects]
        if unknown:
            response.message = f"unknown object(s) {', '.join(unknown)}: no detect: section in the world file"
            return response
        wanted = request.object_ids or list(self.objects)
        start = self.get_clock().now() + Duration(seconds=self.settle)
        msg, info = self.wait_for_image(start)
        if msg is None:
            response.message = 'no image from /wrist_camera'
            return response
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp), timeout=Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            response.message = f'tf: {e}'
            return response
        rotation = quaternion_matrix(tf.transform.rotation)  # optical -> base
        t = tf.transform.translation
        origin = np.array([t.x, t.y, t.z])
        camera = Camera(info)
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        debug = image.copy()
        robot = self.self_mask(image.shape[:2], Time.from_msg(msg.header.stamp), msg.header.frame_id, camera)
        if isinstance(robot, str):
            response.message = f'robot mask: {robot}'
            return response
        outline, _ = cv2.findContours(robot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, outline, -1, (255, 128, 0), 1)  # blue

        scene = self.scene_positions()
        found, report = {}, []
        for obj in (self.objects[i] for i in wanted):
            others = {i: p for i, p in {**scene, **{i: [q.position.x, q.position.y] for i, q in found.items()}}.items()
                      if i != obj['id']}
            result = self.locate(obj, hsv, robot, camera, rotation, origin, debug, others)
            if isinstance(result, str):
                report.append(f"{obj['id']}: {result}")
                self.get_logger().warning(report[-1])
            else:
                found[obj['id']] = result[0]
                report.append(f"{obj['id']}: x {result[0].position.x:.3f} y {result[0].position.y:.3f} "
                              f'yaw {math.degrees(result[1]):.1f} deg (IoU {result[2]:.2f}{result[3]})')
                self.get_logger().info(report[-1])
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8', header=msg.header))
        if self.debug_dir:
            name = f'{self.debug_dir}/{msg.header.stamp.sec}.{msg.header.stamp.nanosec // 1000000:03d}'
            cv2.imwrite(f'{name}_image.png', image)
            cv2.imwrite(f'{name}_debug.png', debug)
            # Enough to replay the detection offline
            with open(f'{name}_camera.yaml', 'w') as f:
                yaml.safe_dump({'frame_id': self.frame, 'origin': origin.tolist(), 'rotation': rotation.tolist(),
                                'k': np.asarray(info.k, dtype=float).tolist(), 'd': np.asarray(info.d, dtype=float).tolist(), 'distortion_model': info.distortion_model}, f)

        if found and not self.apply(found):
            response.message = 'apply_planning_scene failed'
            return response
        response.success = len(found) == len(wanted)
        response.message = '; '.join(report)
        for obj_id, pose in found.items():
            response.found_ids.append(obj_id)
            response.poses.append(PoseStamped(header=msg.header, pose=pose))
            response.poses[-1].header.frame_id = self.frame
        return response

    def scene_positions(self):
        """object id -> [x, y] of the detectable objects in MoveIt's scene (attached ones aside)"""
        if not self.get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('/get_planning_scene not available: objects sharing a colour may be mixed up')
            return {}
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = self.get_client.call(request, timeout_sec=CALL_TIMEOUT_S)
        if response is None:
            return {}
        positions = {}
        for obj in response.scene.world.collision_objects:
            if obj.id not in self.objects or not obj.primitive_poses or obj.header.frame_id != self.frame:
                continue
            p, b = obj.pose.position, obj.primitive_poses[0].position
            point = quaternion_matrix(obj.pose.orientation) @ [b.x, b.y, b.z] + [p.x, p.y, p.z]
            positions[obj.id] = point[:2].tolist()
        return positions

    def locate(self, obj, hsv, robot, camera, rotation, origin, debug, others=None):
        """-> (pose, yaw, iou, fit note), or an error message. robot: pixels to ignore. others:
        object id -> [x, y] of other objects: a blob fitted there is that object"""
        mask = hsv_mask(hsv, obj['detect']) & ~robot
        background = obj['detect'].get('background')
        background_mask = hsv_mask(hsv, background) if background else None
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return 'not seen'
        h, w = mask.shape
        candidates, skipped = [], []
        for blob in sorted(contours, key=cv2.contourArea, reverse=True)[:MAX_BLOBS]:
            area = cv2.contourArea(blob)
            if area < MIN_BLOB_PX or len(candidates) == MAX_CANDIDATES:
                break
            x, y, bw, bh = cv2.boundingRect(blob)
            if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
                skipped.append(blob)
            else:
                candidates.append((blob, background_mask is None or
                                   on_background(blob, background_mask) >= BACKGROUND_MIN))
        cv2.drawContours(debug, skipped, -1, (0, 160, 255), 1)  # orange
        cv2.drawContours(debug, [b for b, on in candidates if not on], -1, (0, 0, 255), 1)  # red
        on_mat = [b for b, on in candidates if on]
        if not on_mat:
            if candidates:
                return 'not on the background' + (' (or cut by the image border)' if skipped else '')
            return 'cut by the image border' if skipped else 'not seen'

        size = np.array(obj['size'], dtype=float)
        taken_m = TAKEN_RATIO * math.hypot(size[0], size[1])
        best = None  # (iou, x, y, yaw, top_only, blob)
        taken = []  # ids of the other objects that explain blobs
        for blob in on_mat:
            fit = self.fit_box(blob, mask.shape, size, camera, rotation, origin)
            if fit is None:
                continue
            near = [i for i, p in (others or {}).items() if math.hypot(fit[1] - p[0], fit[2] - p[1]) < taken_m]
            if near:
                taken.extend(near)
                cv2.drawContours(debug, [blob], -1, (160, 160, 160), 1)  # grey
                continue
            if best is None or fit[0] > best[0]:
                best = (*fit, blob)
        if best is None:
            if taken:
                return f"only {', '.join(sorted(set(taken)))} there"
            return 'the camera does not look down at it'
        score, x, y, yaw, top_only, blob = best
        cv2.drawContours(debug, [blob], -1, (0, 255, 0), 1)
        px = self.box_pixels([x, y, yaw], size, camera, rotation, origin, top_only)
        cv2.polylines(debug, [cv2.convexHull(px.astype(np.int32))], True, (255, 0, 255), 1)
        if score < MIN_IOU:
            return f'the box model does not match any blob (best IoU {score:.2f})'

        pose = Pose()
        pose.position.x, pose.position.y = float(x), float(y)
        pose.position.z = self.table_z + size[2] / 2.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose, yaw, score, ', top face' if top_only else ''

    def fit_box(self, blob, shape, size, camera, rotation, origin):
        """-> (iou, x, y, yaw, top_only) of the box that best explains the blob, None if not
        below the camera. top_only: the blob is its top face, not its whole outline"""
        blob_mask = np.zeros(shape, np.uint8)
        cv2.drawContours(blob_mask, [blob], -1, 255, cv2.FILLED)

        top_z = self.table_z + size[2]
        rays = camera.rays(blob.reshape(-1, 2)) @ rotation.T
        if np.any(rays[:, 2] >= -1e-6):
            return None
        on_top = origin + rays * ((top_z - origin[2]) / rays[:, 2])[:, None]
        (cx, cy), _, angle = cv2.minAreaRect(on_top[:, :2].astype(np.float32))

        # Refine
        def iou(p, top_only):
            px = self.box_pixels(p, size, camera, rotation, origin, top_only)
            if px is None:
                return 0.0
            model = np.zeros(shape, np.uint8)
            hull = cv2.convexHull(np.round(px * 16).astype(np.int32))
            cv2.fillConvexPoly(model, hull, 255, cv2.LINE_8, 4)
            inter = cv2.countNonZero(model & blob_mask)
            return inter / max(cv2.countNonZero(model | blob_mask), 1)

        best_p, best, best_top = None, -1.0, False
        for top_only in (False, True):
            for turn in (0.0, 45.0, 90.0, 135.0):  # minAreaRect's angle can be any side's
                p, score = self.refine(lambda q: iou(q, top_only), np.array([cx, cy, math.radians(angle + turn)]))
                if score > best:
                    best_p, best, best_top = p, score, top_only
        x, y, yaw = best_p
        period = math.pi / 2 if abs(size[0] - size[1]) < 1e-4 else math.pi
        yaw = (yaw + period / 2) % period - period / 2
        return best, x, y, yaw, best_top

    def box_pixels(self, p, size, camera, rotation, origin, top_only=False):
        """Pixels of the 8 corners of the box at (x, y, yaw) on the table (the 4 of its top
        face with top_only), None if behind"""
        x, y, yaw = p
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = size[0] / 2, size[1] / 2
        corners = []
        heights = (self.table_z + size[2],) if top_only else (self.table_z, self.table_z + size[2])
        for dx, dy in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)):
            for z in heights:
                corners.append([x + c * dx - s * dy, y + s * dx + c * dy, z])
        in_camera = (np.array(corners) - origin) @ rotation  # base -> optical
        if np.any(in_camera[:, 2] < 0.01):
            return None
        return camera.project(in_camera)

    @staticmethod
    def refine(score, p):
        """Coordinate search on (x, y, yaw), halving the steps down to 0.25 mm and 0.25 deg"""
        best = score(p)
        steps = np.array([0.004, 0.004, math.radians(8.0)])
        while steps[0] > 0.00025:
            improved = False
            for i in range(3):
                for sign in (1.0, -1.0):
                    q = p.copy()
                    q[i] += sign * steps[i]
                    s = score(q)
                    if s > best:
                        p, best, improved = q, s, True
            if not improved:
                steps /= 2.0
        return p, best

    def apply(self, poses):
        if not self.apply_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('/apply_planning_scene not available')
            return False
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene(is_diff=True)
        for obj_id, pose in poses.items():
            obj = CollisionObject()
            obj.header.frame_id = self.frame
            obj.id = obj_id
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.BOX, dimensions=[float(v) for v in self.objects[obj_id]['size']]))
            obj.primitive_poses.append(pose)
            request.scene.world.collision_objects.append(obj)
        response = self.apply_client.call(request, timeout_sec=CALL_TIMEOUT_S)
        return response is not None and response.success


def main():
    rclpy.init()
    node = ObjectDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
