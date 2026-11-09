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
}


def launch_setup(context):
    description_share = FindPackageShare('so101_description')
    bringup_share = FindPackageShare('so101_bringup')

    sim = LaunchConfiguration('sim').perform(context)
    if sim not in CONTROL_NODE_PACKAGE:
        raise ValueError(f"sim must be one of {list(CONTROL_NODE_PACKAGE)}, got '{sim}'")
    use_sim_time = {'use_sim_time': sim == 'mujoco'}

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([description_share, 'urdf', 'so101.urdf.xacro']),
            ' wrist_camera:=', LaunchConfiguration('wrist_camera'),
            ' sim:=', sim,
            ' headless:=', LaunchConfiguration('headless'),
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
            package=CONTROL_NODE_PACKAGE[sim],
            executable='ros2_control_node',
            parameters=[controllers, use_sim_time],
            output='screen',
            # Closing the MuJoCo window ends the whole launch.
            on_exit=Shutdown() if sim == 'mujoco' else None,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                'arm_controller',
                'gripper_controller',
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
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Add the wrist camera and its mount to the model',
        ),
        DeclareLaunchArgument(
            'sim', default_value='mock', choices=list(CONTROL_NODE_PACKAGE),
            description='mock: commands echoed back as states. mujoco: MuJoCo physics',
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='sim:=mujoco only: run without the MuJoCo viewer window',
        ),
        OpaqueFunction(function=launch_setup),
    ])
