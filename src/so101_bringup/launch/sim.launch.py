from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    common = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('so101_bringup'), 'launch', 'common.launch.py']),
        launch_arguments={
            'hardware': LaunchConfiguration('sim'),
            'rviz': LaunchConfiguration('rviz'),
            'wrist_camera': LaunchConfiguration('wrist_camera'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='mock', choices=['mock', 'mujoco'],
            description='mock: commands echoed back as states. mujoco: MuJoCo physics',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz (set false when running MoveIt on top)',
        ),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Add the wrist camera and its mount to the model',
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='sim:=mujoco only: run without the MuJoCo viewer window',
        ),
        common,
    ])
