from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', description="World file (YAML) of the objects for MoveIt's planning scene"),
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='true: leave the objects with a detect: section to object_detector',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true when the bringup runs MuJoCo (sim:=mujoco)',
        ),
        Node(
            package='so101_skills',
            executable='scene_publisher',
            output='screen',
            parameters=[{
                'world': LaunchConfiguration('world'),
                'camera': ParameterValue(LaunchConfiguration('camera'), value_type=bool),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
