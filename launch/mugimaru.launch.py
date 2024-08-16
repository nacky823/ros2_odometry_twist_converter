# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
    )

    params_file=os.path.join(
        get_package_share_directory('ros2_odometry_twist_converter'),
        'config', 'mugimaru.param.yaml'
    )

    odom_to_twist=Node(
        package='ros2_odometry_twist_converter',
        executable='odometry_twist_converter',
        name='odom_to_twist_cov_stamp',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('input_odom', 'odom'),
            ('output_twist', 'mugimaru_twist'),
        ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(odom_to_twist)

    return ld
