"""Mock-hardware bringup + move_group + RViz with the MotionPlanning panel."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_moveit(context):
    # move_group and RViz need the URDF as a parameter, built with the same
    # wrist_camera value as the one robot_state_publisher uses in bringup.
    wrist_camera = LaunchConfiguration('wrist_camera').perform(context)
    moveit_config = (
        MoveItConfigsBuilder('so101', package_name='so101_moveit_config')
        .robot_description(mappings={'wrist_camera': wrist_camera})
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
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
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return [move_group, rviz]


def generate_launch_description():
    # Scoped so that the rviz:=false given to bringup does not leak into
    # this file's own rviz argument.
    bringup = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('so101_bringup'), 'launch', 'so101.launch.py']),
        launch_arguments={
            'rviz': 'false',  # we start our own RViz with the MoveIt config
            'wrist_camera': LaunchConfiguration('wrist_camera'),
        }.items(),
    )])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Add the wrist camera and its mount to the model',
        ),
        bringup,
        OpaqueFunction(function=launch_moveit),
    ])
