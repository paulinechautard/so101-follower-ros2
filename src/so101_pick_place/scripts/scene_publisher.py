#!/usr/bin/env python3
import math
import threading

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

# Smaller sim motions are not sent to MoveIt
MOVE_THRESHOLD_M = 0.002
TURN_THRESHOLD_RAD = 0.03
CALL_TIMEOUT_S = 5.0


def pose_from_list(values):
    """[x, y, z, yaw] -> Pose"""
    x, y, z, yaw = (float(v) for v in values)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def moved(a, b):
    distance = math.dist((a.position.x, a.position.y, a.position.z),
                         (b.position.x, b.position.y, b.position.z))
    qa, qb = a.orientation, b.orientation
    dot = abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w)
    angle = 2.0 * math.acos(min(dot, 1.0))
    return distance > MOVE_THRESHOLD_M or angle > TURN_THRESHOLD_RAD


class ScenePublisher(Node):

    def __init__(self):
        super().__init__('scene_publisher')
        path = self.declare_parameter('scene_file', '').value
        if not path:
            raise ValueError('parameter scene_file is required')
        with open(path) as f:
            scene = yaml.safe_load(f)
        self.frame = scene.get('frame_id', 'base_link')
        self.objects = {}
        for obj in scene['objects']:
            if len(obj['size']) != 3 or len(obj['pose']) != 4:
                raise ValueError(f"{obj['id']}: size must be [x, y, z], pose [x, y, z, yaw]")
            self.objects[obj['id']] = obj

        group = ReentrantCallbackGroup()
        self.apply_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=group)
        self.get_client = self.create_client(GetPlanningScene, '/get_planning_scene', callback_group=group)
        self.create_service(Trigger, '~/reset', self.reset, callback_group=group)

        self.tracked = {o['mujoco_body']: o['id'] for o in self.objects.values() if 'mujoco_body' in o}
        self.sim_poses = {}  # object id -> latest pose in the sim
        self.sent_poses = {}  # object id -> pose last sent to MoveIt
        if self.tracked:
            self.create_subscription(
                FreeJointStateArray, '/free_joint_state_publisher/free_joint_states', self.on_sim, 10)
            sim_node = self.declare_parameter('sim_node', '/mujoco_ros2_control_node').value
            self.sim_client = self.create_client(
                SetFreeJointState, f'{sim_node}/set_free_joint_state', callback_group=group)

        self.started = False
        self.lock = threading.Lock()  # one scene update at a time
        self.waiting_logged = False
        self.create_timer(0.5, self.tick, callback_group=group)

    def box(self, obj_id, pose):
        obj = CollisionObject()
        obj.header.frame_id = self.frame
        obj.id = obj_id
        obj.operation = CollisionObject.ADD  # replaces an object with the same id
        obj.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=[float(v) for v in self.objects[obj_id]['size']]))
        obj.primitive_poses.append(pose)
        return obj

    def apply(self, poses):
        """poses: object id -> Pose"""
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene(is_diff=True)
        request.scene.world.collision_objects = [self.box(i, p) for i, p in poses.items()]
        response = self.apply_client.call(request, timeout_sec=CALL_TIMEOUT_S)
        if response is None or not response.success:
            self.get_logger().warn('apply_planning_scene failed')
            return False
        self.sent_poses.update(poses)
        return True

    def attached_ids(self):
        """Ids of the objects attached to the robot, None if unknown"""
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        response = self.get_client.call(request, timeout_sec=CALL_TIMEOUT_S)
        if response is None:
            return None
        return {a.object.id for a in response.scene.robot_state.attached_collision_objects}

    def file_poses(self):
        return {i: pose_from_list(o['pose']) for i, o in self.objects.items()}

    def on_sim(self, msg):
        for joint in msg.free_joints:
            if joint.name in self.tracked:
                self.sim_poses[self.tracked[joint.name]] = joint.pose.pose

    def tick(self):
        if not self.lock.acquire(blocking=False):
            return
        try:
            if not self.started:
                self.start()
            elif self.tracked:
                self.follow_sim()
        finally:
            self.lock.release()

    def start(self):
        if not self.apply_client.service_is_ready():
            if not self.waiting_logged:
                self.get_logger().info('Waiting for move_group (/apply_planning_scene)')
                self.waiting_logged = True
            return
        if self.apply(self.file_poses()):
            self.started = True
            self.get_logger().info(f"Added {', '.join(self.objects)} to the planning scene")

    def follow_sim(self):
        changed = {i: p for i, p in dict(self.sim_poses).items()
                   if i not in self.sent_poses or moved(p, self.sent_poses[i])}
        if not changed:
            return
        attached = self.attached_ids()
        if attached is None:
            return
        changed = {i: p for i, p in changed.items() if i not in attached}
        if changed:
            self.apply(changed)

    def move_sim_bodies(self, poses):
        """Teleport the tracked bodies to poses (object id -> Pose). Returns an error or ''"""
        if not self.sim_client.wait_for_service(timeout_sec=1.0):
            return f'{self.sim_client.srv_name} not available'
        request = SetFreeJointState.Request()
        for body, obj_id in self.tracked.items():
            joint = FreeJointState(name=body)
            joint.pose.pose = poses[obj_id]  # empty frame_id: MuJoCo's world, i.e. base_link
            request.free_joints.append(joint)
        response = self.sim_client.call(request, timeout_sec=CALL_TIMEOUT_S)
        if response is None:
            return f'{self.sim_client.srv_name} timed out'
        if not response.success:
            return f'{self.sim_client.srv_name}: {response.message}'
        # Until the next sim message, so that follow_sim doesn't send the old pose back.
        self.sim_poses.update({i: poses[i] for i in self.tracked.values()})
        return ''

    def reset(self, request, response):
        poses = self.file_poses()
        with self.lock:
            error = self.move_sim_bodies(poses) if self.tracked else ''
            if not error and not self.apply(poses):
                error = 'apply_planning_scene failed'
        response.success = not error
        response.message = error
        if error:
            self.get_logger().warn(f'reset: {error}')
        return response


def main():
    rclpy.init()
    node = ScenePublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():  # e.g. Ctrl-C
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
