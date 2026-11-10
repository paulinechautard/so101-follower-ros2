from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scene_file',
            default_value=PathJoinSubstitution([FindPackageShare('so101_pick_place'), 'config', 'sim_scene.yaml']),
            description="Objects to put in MoveIt's planning scene (default: world:=pick_place's)",
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true when the bringup runs MuJoCo (sim:=mujoco)',
        ),
        Node(
            package='so101_pick_place',
            executable='scene_publisher',
            output='screen',
            parameters=[{
                'scene_file': LaunchConfiguration('scene_file'),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
