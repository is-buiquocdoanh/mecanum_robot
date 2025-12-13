#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# License: Apache-2.0
# Khởi chạy nút động học ROS

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='kinematic_node',
            name='kinematic_node',
            output='screen',
            emulate_tty=True,
        )
    ])