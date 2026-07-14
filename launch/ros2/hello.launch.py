#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    package_name = "hex_ros_robot_arm"

    robot_param_path = FindPackageShare(package_name).find(
        package_name) + '/config/ros2/hello_params.yaml'
    
    ### robot node
    robot_node = Node(
        package=package_name,
        executable='hex_ros_robot_hello_y6',
        name='hex_ros_robot_hello_y6',
        output="screen",
        emulate_tty=True,
        parameters=[
            robot_param_path,
            {
                'robot_host': LaunchConfiguration('robot_host'),
                'robot_port': LaunchConfiguration('robot_port'),
            },
        ],
    )

    ### robot host arg (for runtime override)
    robot_host_arg = DeclareLaunchArgument(
        name='robot_host',
        default_value='192.168.1.100',
        description='Robot controller IP address')

    ### robot port arg
    robot_port_arg = DeclareLaunchArgument(
        name='robot_port',
        default_value='8439',
        description='Robot WebSocket port')

    return LaunchDescription([
        robot_host_arg,
        robot_port_arg,
        robot_node,
    ])
