#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummyx_usb2can',
            executable='usb2can_node',
            name='usb2can_node',
            parameters=[{
                'interface': 'socketcan',
                'channel': 'can0',
                'motor_config_json': '{"1":30.0,"2":50.0,"3":50.0,"4":30.0,"5":30.0,"6":30.0}'
            }]
        )
    ])