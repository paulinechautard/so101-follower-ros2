# so101-follower-ros2

ROS 2 stack for a standalone SO-101 follower arm.

- **Driver:** [sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface), from
  [a fork](https://github.com/paulinechautard/sts_hardware_interface/tree/feat/sign-inversion-for-directions)
  adding per-joint `invert_direction` (the URDF follows LeRobot's sign convention)
- **Simulation:** MuJoCo via `mujoco_ros2_control`
- **Planning:** MoveIt 2 with pick_ik
- **Wrist camera:** innomaker 130° UVC module on the hex-nut mount, via [usb_cam](https://github.com/ros-drivers/usb_cam)
- **Platform:** Ubuntu 26.04, ROS 2 Lyrical (tested on WSL2)

## Build

The repo root is the colcon workspace.

```bash
source /opt/ros/lyrical/setup.bash
vcs import src < so101.repos
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch so101_description display.launch.py          # model + joint sliders
ros2 launch so101_bringup sim.launch.py                   # mock hardware
ros2 launch so101_bringup sim.launch.py sim:=mujoco       # MuJoCo (headless:=true, world:=<world file>)
ros2 launch so101_bringup real.launch.py calibration:=<lerobot_calibration.json>
```

All take `wrist_camera:=false` and `rviz:=false`. Controllers: `joint_state_broadcaster`,
`arm_controller` (JointTrajectoryController) and `gripper_controller` (parallel GripperActionController).
In MuJoCo, the wrist camera is rendered on the same topics as the real one (`/wrist_camera/image_raw`).

### Real arm

- Read-only by default (torque off). `read_only:=false` turns torque on and writes LeRobot's servo settings.
- Before that, put the arm **inside the URDF limits**: the folded rest pose has `shoulder_lift`
  below its limit and would snap to it.
- **Don't call** `/one_key_calibration`: it rewrites the encoder offset in EEPROM and breaks the
  LeRobot calibration.

## MoveIt

Runs on top of a bringup started with `rviz:=false`:

```bash
ros2 launch so101_moveit_config moveit.launch.py          # use_sim_time:=true with MuJoCo
```

Groups: `arm` (named state `home`) and `gripper` (`open`, `closed`). The arm has 5 DOF: tick
*Approx IK Solutions* in RViz when dragging the marker.

## Skills

```bash
W=$(ros2 pkg prefix so101_examples)/share/so101_examples
ros2 launch so101_bringup sim.launch.py sim:=mujoco world:=$W/worlds/sort.yaml rviz:=false
ros2 launch so101_moveit_config moveit.launch.py use_sim_time:=true
ros2 launch so101_skills skills.launch.py use_sim_time:=true world:=$W/worlds/sort.yaml \
  params_file:=$W/config/sort_skills.yaml     # camera:=true: find objects with the wrist camera
```

The `skills` node serves the arm's skills as actions, one goal at a time: `find_object`, `pick`, `place`
(the held object's centre at `place_pose`, only its yaw is used) and `move_to` (a named state:
`home`). The Python client wraps them:

```python
from so101_skills import So101, pose

robot = So101(node)
robot.find('red_1')
robot.pick('red_1')
robot.place(pose(0.17, -0.04, 0.0096, yaw=0.5))
robot.home()
```

A world file (YAML) lists the objects once, for MuJoCo (generated at launch), MoveIt and the
camera: boxes with a size and a pose, loose with a `mass`, `visual: true` for decoration,
`mujoco: false` for MoveIt only; `detect:` for the camera. See `so101_examples/worlds/sort.yaml`.
`/scene_publisher/reset` puts the objects back. On the real arm, add
`params_file:=…/config/real_skills.yaml` (`gripper_grasp` measured on your object).

## Examples

`so101_examples`, on the Python client:

```bash
ros2 run so101_examples keeper     
ros2 run so101_examples sorter     
```

## Third-party files

From [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) `Simulation/SO101/`
at commit `a758567`: the URDF, meshes, `mjcf/scene.xml`, `mjcf/upstream/so101_new_calib.xml`
and the wrist camera placement. The upstream MJCF is kept unmodified; `scripts/build_mjcf.py`
generates the arm models from it at build time (box pad on the fixed jaw, wrist camera).
