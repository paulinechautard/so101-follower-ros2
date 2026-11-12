#!/usr/bin/env python3
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from so101_skills import So101, SkillFailed, pose


class Sorter(Node):

    def __init__(self):
        super().__init__('sorter')
        path = self.declare_parameter(
            'world', f"{get_package_share_directory('so101_examples')}/worlds/sort.yaml").value
        self.reset = self.declare_parameter('reset', False).value
        with open(path) as f:
            world = yaml.safe_load(f)
        self.frame = world.get('frame_id', 'base_link')
        self.table_z = float(world.get('table_z', 0.0))
        self.bins = {o['bin']: o['pose'] for o in world['objects'] if 'bin' in o}  # colour -> [x, y, z, yaw]
        self.bricks = [o for o in world['objects'] if 'colour' in o]
        if not self.bricks:
            raise ValueError(f'{path}: no object has a colour:')
        for brick in self.bricks:
            if brick['colour'] not in self.bins:
                raise ValueError(f"{path}: {brick['id']} is {brick['colour']}, which has no bin")
            if 'detect' not in brick:
                raise ValueError(f"{path}: {brick['id']} has no detect: section: the camera must find it")
        self.robot = So101(self, log_stages=True)

    def place_pose(self, brick, level):
        """On the bin of the brick's colour, on top of `level` bricks"""
        x, y, _, yaw = (float(v) for v in self.bins[brick['colour']])
        height = float(brick['size'][2])
        return pose(x, y, self.table_z + height * (level + 0.5), yaw, self.frame)

    def run(self):
        log = self.get_logger()
        if self.reset:
            if self.robot.reset_scene():
                log.info('Bricks put back')
                time.sleep(1.0)  # let them settle
            else:
                log.warning('Not reset')
        stacked = {colour: 0 for colour in self.bins}
        missed = []
        start = time.monotonic()
        for n, brick in enumerate(self.bricks, 1):
            brick_id, colour = brick['id'], brick['colour']
            log.info(f'[{n}/{len(self.bricks)}] {brick_id}: a {colour} brick to its bin '
                     f'(level {stacked[colour] + 1})')
            try:
                found = self.robot.find(brick_id)
                self.robot.pick(brick_id)
                self.robot.place(self.place_pose(brick, stacked[colour]))
            except SkillFailed as e:
                log.warning(f'{brick_id}: {e}: skipped')
                missed.append(brick_id)
                continue
            stacked[colour] += 1
            p = found.pose.position
            log.info(f'{brick_id}: found at x {p.x:.3f} y {p.y:.3f}, now on the {colour} bin')
        try:
            self.robot.home()
        except SkillFailed as e:
            log.warning(str(e))
        summary = ', '.join(f'{c} {k}' for c, k in stacked.items())
        log.info(f'Done in {time.monotonic() - start:.0f} s: {summary}' +
                 (f"; not sorted: {', '.join(missed)}" if missed else ''))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Sorter()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
