from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare('so101_description')
    bringup_share = FindPackageShare('so101_bringup')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([description_share, 'urdf', 'so101.urdf.xacro']),
            ' wrist_camera:=', LaunchConfiguration('wrist_camera'),
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Add the wrist camera and its mount to the model',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # Reads the URDF from /robot_description (published above).
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers],
            output='screen',
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
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([description_share, 'rviz', 'display.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
