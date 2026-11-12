from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    share = FindPackageShare('so101_skills').perform(context)
    camera = LaunchConfiguration('camera').perform(context) == 'true'
    wrist_camera = LaunchConfiguration('wrist_camera').perform(context)
    if camera and wrist_camera != 'true':
        raise RuntimeError('camera:=true needs wrist_camera:=true')
    world = LaunchConfiguration('world').perform(context)
    extra = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    moveit_config = (
        MoveItConfigsBuilder('so101', package_name='so101_moveit_config')
        .robot_description(mappings={'wrist_camera': wrist_camera})
        .to_moveit_configs()
    )

    actions = [
        IncludeLaunchDescription(
            f'{share}/launch/scene.launch.py',
            launch_arguments={
                'world': world, 'camera': str(camera).lower(), 'use_sim_time': str(use_sim_time).lower(),
            }.items(),
        ),
        Node(
            package='so101_skills',
            executable='skills',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                f'{share}/config/skills.yaml',
                *([extra] if extra else []),
                {'use_sim_time': use_sim_time, 'use_camera': camera},
            ],
        ),
    ]
    if camera:
        actions.append(Node(
            package='so101_skills',
            executable='object_detector',
            output='screen',
            parameters=[{
                'world': world,
                'use_sim_time': use_sim_time,
                'debug_dir': LaunchConfiguration('debug_dir').perform(context),
            }],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='false',
            description="true: locate the world file's detect: objects with the wrist camera",
        ),
        DeclareLaunchArgument(
            'debug_dir', default_value='',
            description='camera:=true: folder where object_detector saves each image it processes',
        ),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='YAML (/** keys) overriding config/skills.yaml, e.g. a real-arm setup',
        ),
        DeclareLaunchArgument(
            'world', description='World file (YAML) of the objects, the same as the bringup\'s in MuJoCo',
        ),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Must match the bringup: include the wrist camera in the model',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true when the bringup runs MuJoCo (sim:=mujoco)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
