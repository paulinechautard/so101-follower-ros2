import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from so101_interfaces.action import FindObject, MoveTo, Pick, Place


class SkillFailed(RuntimeError):
    """A skill was aborted canceled or rejected, or its server went away"""


def pose(x, y, z, yaw=0.0, frame_id='base_link'):
    """PoseStamped at (x, y, z) turned by yaw (rad) about the vertical"""
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return p


def yaw_of(p):
    """Yaw (rad) of a PoseStamped about the vertical"""
    q = p.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class So101:

    def __init__(self, node, log_stages=False):
        self.node = node
        self.log = node.get_logger()
        self.log_stages = log_stages
        self._actions = {
            'find_object': ActionClient(node, FindObject, 'find_object'),
            'pick': ActionClient(node, Pick, 'pick'),
            'place': ActionClient(node, Place, 'place'),
            'move_to': ActionClient(node, MoveTo, 'move_to'),
        }
        self._reset = node.create_client(Trigger, '/scene_publisher/reset')

    def find(self, object_id, look_at=None):
        """Locate an object"""
        goal = FindObject.Goal(object_id=object_id)
        if isinstance(look_at, PoseStamped):
            goal.look_at = PointStamped(header=look_at.header, point=look_at.pose.position)
        elif look_at is not None:
            goal.look_at = look_at
        return self._run('find_object', goal, object_id).object_pose

    def pick(self, object_id):
        """Grasp an object known to the planning scene"""
        return self._run('pick', Pick.Goal(object_id=object_id), object_id).object_pose

    def place(self, place_pose):
        """Put the held object's centre at place_pose"""
        self._run('place', Place.Goal(place_pose=place_pose), 'place')

    def move_to(self, named_target):
        self._run('move_to', MoveTo.Goal(named_target=named_target), named_target)

    def home(self):
        self.move_to('home')

    def reset_scene(self, timeout=10.0):
        """Sim only"""
        if not self._reset.wait_for_service(timeout_sec=5.0):
            self.log.warning(f'{self._reset.srv_name} not available')
            return False
        future = self._reset.call_async(Trigger.Request())
        if not self._spin_until(future, timeout) or not future.result().success:
            self.log.warning(f'{self._reset.srv_name} failed')
            return False
        return True

    def _spin_until(self, future, timeout=math.inf):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return future.done()

    def _run(self, name, goal, label):
        client = self._actions[name]
        if not client.server_is_ready():
            self.log.info(f'Waiting for the {name} action server (skills.launch.py)...')
            while not client.server_is_ready():
                rclpy.spin_once(self.node, timeout_sec=0.1)

        feedback = None
        if self.log_stages:
            def feedback(msg):
                self.log.info(f'  {label}: {msg.feedback.stage}')
        future = client.send_goal_async(goal, feedback_callback=feedback)
        if not self._spin_until(future, timeout=10.0):
            raise SkillFailed(f'{name}: no answer to the goal')
        handle = future.result()
        if not handle.accepted:
            raise SkillFailed(f'{name}: goal rejected (busy with another goal)')

        future = handle.get_result_async()
        try:
            while not self._spin_until(future, timeout=1.0):
                if not client.server_is_ready():  # its result won't come
                    raise SkillFailed(f'{name}: the server went away during the goal')
        except KeyboardInterrupt:
            self.log.info(f'Canceling {name}')
            self._spin_until(handle.cancel_goal_async(), timeout=5.0)
            raise
        response = future.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise SkillFailed(f'{name}: {response.result.message}')
        return response.result
