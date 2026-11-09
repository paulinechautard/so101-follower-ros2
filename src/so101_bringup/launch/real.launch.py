from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    common = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('so101_bringup'), 'launch', 'common.launch.py']),
        launch_arguments={
            'hardware': 'real',
            'read_only': 'true',
            'calibration': LaunchConfiguration('calibration'),
            'serial_port': LaunchConfiguration('serial_port'),
            'dry_run': LaunchConfiguration('dry_run'),
            'rviz': LaunchConfiguration('rviz'),
            'wrist_camera': LaunchConfiguration('wrist_camera'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration',
            description='LeRobot calibration file (JSON) of this arm',
        ),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Serial port of the servo bus',
        ),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description="Run the driver's mock mode instead of opening the serial port",
        ),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Add the wrist camera and its mount to the model',
        ),
        common,
    ])
