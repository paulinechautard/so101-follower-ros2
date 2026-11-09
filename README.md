# so101-follower-ros2

ROS 2 stack for a standalone SO-101 follower arm.

- **Arm:** SO-101 follower (TheRobotStudio design), 6 Feetech STS3215 servos
  on one serial bus: 5 arm joints + 1 gripper
- **Driver:** [sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)
  (ros2_control plugin) built from source
- **Simulation:** MuJoCo via `mujoco_ros2_control`
- **Motion planning:** MoveIt 2 with pick_ik, and MoveIt Servo for Cartesian teleop
- **Platform:** Ubuntu 26.04, ROS 2 Lyrical (tested on WSL2)

## Setup

### Build the workspace

The repo root is the colcon workspace.

```bash
source /opt/ros/lyrical/setup.bash
cd so101-follower-ros2
vcs import src < so101.repos          # external sources into src/external/
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## View the robot model

```bash
ros2 launch so101_description display.launch.py
```

Opens RViz and a slider window with one slider per joint.

## Third-party files

`src/so101_description/urdf/so101_new_calib.urdf` and the STL files in
`src/so101_description/meshes/` are taken from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100),
`Simulation/SO101/`, at commit `a758567`.