#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# License: Apache-2.0
# Khởi chạy cầu nối nối tiếp ROS

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='ros_serial_bridge',
            name='ros_serial_bridge',
            output='screen',
            emulate_tty=True
        )
    ])