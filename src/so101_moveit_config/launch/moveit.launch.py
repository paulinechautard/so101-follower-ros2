"""MoveIt (move_group and RViz) on top of a running bringup.

Start so101_bringup's sim.launch.py first (with rviz:=false), then this.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_moveit(context):
    wrist_camera = LaunchConfiguration('wrist_camera').perform(context)
    moveit_config = (
        MoveItConfigsBuilder('so101', package_name='so101_moveit_config')
        .robot_description(mappings={'wrist_camera': wrist_camera})
        .to_moveit_configs()
    )
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), use_sim_time],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('so101_moveit_config'), 'config', 'moveit.rviz'])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Must match the bringup: include the wrist camera in the model',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true when the bringup runs MuJoCo (sim:=mujoco)',
        ),
        OpaqueFunction(function=launch_moveit),
    ])
