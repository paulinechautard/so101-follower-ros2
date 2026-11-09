# so101-follower-ros2

ROS 2 stack for a standalone SO-101 follower arm.

- **Arm:** SO-101 follower (TheRobotStudio design), 6 Feetech STS3215 servos
  on one serial bus: 5 arm joints + 1 gripper
- **Driver:** [sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)
  (ros2_control plugin) built from source
- **Simulation:** MuJoCo via `mujoco_ros2_control`
- **Motion planning:** MoveIt 2 with pick_ik, and MoveIt Servo for Cartesian teleop
- **Wrist camera:** innomaker 1080P USB2.0 UVC 130° module on the 3D-printed
  hex-nut wrist mount
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

Both launch files take `wrist_camera:=false` to leave the camera and its mount
out of the model. With the camera, three links hang under `gripper_link`:
`wrist_camera_mount_link`, `wrist_camera_link` and `wrist_camera_optical_frame`
(REP 103: z out of the lens, x right, y down). The optical frame's roll about z
has not been checked against a real image yet.

## Run with mock hardware

```bash
ros2 launch so101_bringup so101.launch.py          # rviz:=false for headless
```

Starts ros2_control with mock hardware (commands are echoed back as states)
and three controllers: `joint_state_broadcaster`, `arm_controller`
(JointTrajectoryController on the 5 arm joints) and `gripper_controller`
(GripperActionController on `gripper`). Try them:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll],
    points: [{positions: [0.5, -0.3, 0.4, 0.2, -0.5], time_from_start: {sec: 2}}]}}"

ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand \
  "{command: {name: [gripper], position: [1.0]}}"
```

## Third-party files

`src/so101_description/urdf/so101_new_calib.urdf` and the STL files in
`src/so101_description/meshes/` are taken from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100),
`Simulation/SO101/`, at commit `a758567`.

The wrist camera meshes, and the placement of the camera links in
`urdf/so101_wrist_camera.xacro`, come from the same source
(`so101_new_calib_camera.urdf`):

- `wrist_camera_so101_v1.stl`: the innomaker 1080P USB2.0 UVC 130° module
  (32 × 32 mm board), modelled in meters
- `wrist_camera_mount_so101_v1.stl`: the 3D-printed hex-nut wrist mount
  (`Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module`), modelled in
  millimeters, so it needs `scale="0.001 0.001 0.001"`
