#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from so101_skills import So101, SkillFailed, pose, yaw_of


class Keeper(Node):

    def __init__(self):
        super().__init__('keeper')
        self.object_id = self.declare_parameter('object_id', 'red_1').value
        self.tolerance = self.declare_parameter('tolerance', 0.02).value
        self.pause = self.declare_parameter('pause', 2.0).value  # s between checks
        self.keep_x = self.declare_parameter('keep_x', math.nan).value
        self.keep_y = self.declare_parameter('keep_y', math.nan).value
        self.keep_yaw = self.declare_parameter('keep_yaw', math.nan).value
        if math.isnan(self.keep_x) != math.isnan(self.keep_y):
            raise ValueError('set both keep_x and keep_y, or neither')
        self.robot = So101(self)

    def find_home(self):
        found = self.robot.find(self.object_id)
        p = found.pose.position  # its height comes from the camera
        x, y = (p.x, p.y) if math.isnan(self.keep_x) else (self.keep_x, self.keep_y)
        yaw = yaw_of(found) if math.isnan(self.keep_yaw) else math.radians(self.keep_yaw)
        self.get_logger().info(f'Home: x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.0f} deg. '
                               f'Keeping {self.object_id} there: Ctrl-C to stop')
        return pose(x, y, p.z, yaw, found.header.frame_id)

    def run(self):
        log = self.get_logger()
        home = None
        in_place = False
        while rclpy.ok():
            try:
                if home is None:
                    home = self.find_home()
                    continue
                found = self.robot.find(self.object_id, look_at=home)
                p, h = found.pose.position, home.pose.position
                off = math.hypot(p.x - h.x, p.y - h.y)
                if off < self.tolerance:
                    if not in_place:
                        log.info(f'{self.object_id} is home: {off * 1000:.0f} mm from the spot')
                        in_place = True
                else:
                    self.robot.pick(self.object_id)
                    self.robot.place(home)
                    log.info(f'{self.object_id} was moved ({off * 1000:.0f} mm): brought it back home')
                    in_place = False
            except SkillFailed as e:
                log.warning(f'{e}: trying again')
                in_place = False
            time.sleep(self.pause)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Keeper()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
