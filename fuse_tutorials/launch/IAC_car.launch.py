#! /usr/bin/env python3

# author: Virgile-Foussereau
# date: 2023-10-08

from launch_ros.actions import SetParameter, Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_dir = FindPackageShare('fuse_tutorials')

    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play',
        #          PathJoinSubstitution(['/home/Documents/rosbags/', 'overtaken_from_right']),
        #          '--clock', '--qos-profile-overrides-path', '/home/Documents/rosbags/reliability_override.yaml'],
        #     output='screen'
        # ),
        SetParameter(name='use_sim_time', value=True),
        Node(
            package='fuse_tutorials',
            executable='gps_to_msg.py',
            name='gps_to_msg',
            arguments=['--random_gps_denied', 'True', '--verbose', 'True'],
        ),
        Node(
            package='fuse_optimizers',
            executable='fixed_lag_smoother_node',
            name='state_estimator',
            parameters=[PathJoinSubstitution([
                pkg_dir, 'config', 'iac_car_unicycle.yaml'
            ])]#,
            #arguments=['--ros-args', '--log-level', 'debug']

        )#,
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     arguments=[
        #         '-d', [PathJoinSubstitution([pkg_dir, 'config', 'IAC_car.rviz'])]
        #     ]
        # )
    ])