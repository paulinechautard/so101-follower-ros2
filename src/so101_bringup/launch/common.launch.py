from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROL_NODE_PACKAGE = {
    'mock': 'controller_manager',
    'mujoco': 'mujoco_ros2_control',
    'real': 'controller_manager',
}


def launch_setup(context):
    description_share = FindPackageShare('so101_description')
    bringup_share = FindPackageShare('so101_bringup')

    hardware = LaunchConfiguration('hardware').perform(context)
    if hardware not in CONTROL_NODE_PACKAGE:
        raise ValueError(
            f"hardware must be one of {list(CONTROL_NODE_PACKAGE)}, got '{hardware}'")
    use_sim_time = {'use_sim_time': hardware == 'mujoco'}
    read_only = hardware == 'real' and LaunchConfiguration('read_only').perform(context) == 'true'

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([description_share, 'urdf', 'so101.urdf.xacro']),
            ' wrist_camera:=', LaunchConfiguration('wrist_camera'),
            ' hardware:=', hardware,
            ' headless:=', LaunchConfiguration('headless'),
            ' calibration:=', LaunchConfiguration('calibration'),
            ' serial_port:=', LaunchConfiguration('serial_port'),
            ' read_only:=', LaunchConfiguration('read_only'),
            ' dry_run:=', LaunchConfiguration('dry_run'),
            ' servo_p:=', LaunchConfiguration('servo_p'),
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}, use_sim_time],
        ),
        # Reads the URDF from /robot_description (published above).
        Node(
            package=CONTROL_NODE_PACKAGE[hardware],
            executable='ros2_control_node',
            parameters=[controllers, use_sim_time],
            output='screen',
            # Closing the MuJoCo window ends the whole launch.
            on_exit=Shutdown() if hardware == 'mujoco' else None,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            # Read-only joints have no command interface, so only the broadcaster can run.
            arguments=[
                'joint_state_broadcaster',
                *([] if read_only else ['arm_controller', 'gripper_controller']),
                '--param-file', controllers,
            ],
            parameters=[use_sim_time],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([description_share, 'rviz', 'display.rviz'])],
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware', choices=list(CONTROL_NODE_PACKAGE),
            description='ros2_control hardware backend',
        ),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Add the wrist camera and its mount to the model',
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='hardware:=mujoco only: run without the MuJoCo viewer window',
        ),
        DeclareLaunchArgument(
            'calibration', default_value='',
            description='hardware:=real only: LeRobot calibration file (JSON)',
        ),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='hardware:=real only: serial port of the servo bus',
        ),
        DeclareLaunchArgument(
            'read_only', default_value='true',
            description='hardware:=real only: torque off, joint states only',
        ),
        DeclareLaunchArgument(
            'servo_p', default_value='16',
            description='hardware:=real only: servo position P gain (EEPROM), LeRobot uses 16',
        ),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description="hardware:=real only: the driver's mock mode, no serial port",
        ),
        OpaqueFunction(function=launch_setup),
    ])
