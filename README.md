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
ros2 launch so101_bringup sim.launch.py sim:=mujoco       # MuJoCo (headless:=true, world:=pick_place)
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

## Pick and place

```bash
ros2 launch so101_bringup sim.launch.py sim:=mujoco world:=pick_place rviz:=false
ros2 launch so101_moveit_config moveit.launch.py use_sim_time:=true
ros2 launch so101_pick_place pick_place.launch.py use_sim_time:=true

ros2 action send_goal --feedback /pick_place so101_interfaces/action/PickPlace \
  "{object_id: cube, place_pose: {pose: {position: {x: 0.18, y: 0.15, z: 0.0125}}}}"
```

`place_pose` is where the object's centre ends up (only its yaw is used). Objects come from
`scene_file:=` (default `config/sim_scene.yaml`); `/scene_publisher/reset` puts them back.
On the real arm, pass your own scene file and a `params_file:=` with `gripper_grasp` set.

## Third-party files

From [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) `Simulation/SO101/`
at commit `a758567`: the URDF, meshes, `mjcf/scene.xml`, `mjcf/upstream/so101_new_calib.xml`
and the wrist camera placement. The upstream MJCF is kept unmodified; `scripts/build_mjcf.py`
generates the arm models from it at build time (box pad on the fixed jaw, wrist camera).
