from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    wrist_camera = LaunchConfiguration('wrist_camera').perform(context)
    moveit_config = (
        MoveItConfigsBuilder('so101', package_name='so101_moveit_config')
        .robot_description(mappings={'wrist_camera': wrist_camera})
        .to_moveit_configs()
    )
    extra = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    return [Node(
        package='so101_pick_place',
        executable='pick_place',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            PathJoinSubstitution([FindPackageShare('so101_pick_place'), 'config', 'pick_place.yaml']),
            *([extra] if extra else []),
            {'use_sim_time': use_sim_time},
        ],
    )]


def generate_launch_description():
    share = FindPackageShare('so101_pick_place')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='YAML (/** keys) overriding config/pick_place.yaml, e.g. a real-arm setup',
        ),
        DeclareLaunchArgument(
            'scene_file', default_value=PathJoinSubstitution([share, 'config', 'sim_scene.yaml']),
            description="Objects to put in MoveIt's planning scene (default: world:=pick_place's)",
        ),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Must match the bringup: include the wrist camera in the model',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true when the bringup runs MuJoCo (sim:=mujoco)',
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([share, 'launch', 'scene.launch.py']),
            launch_arguments={
                'scene_file': LaunchConfiguration('scene_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        OpaqueFunction(function=launch_setup),
    ])
