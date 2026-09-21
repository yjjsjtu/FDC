#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dummyx_usb2can'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaowen08',
    maintainer_email='xin.li@switchpi.com',
    description='A dummy USB to CAN interface node for ROS2',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'usb2can_node = dummyx_usb2can.usb2can_node:main',
        ],
    },
)