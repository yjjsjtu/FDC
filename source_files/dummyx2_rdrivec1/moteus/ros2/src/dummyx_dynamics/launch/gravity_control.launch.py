#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gravity_control.launch.py

Usage:
  ros2 launch dummyx_dynamics gravity_control.launch.py
  ros2 launch dummyx_dynamics gravity_control.launch.py mode:=full_dynamics
  ros2 launch dummyx_dynamics gravity_control.launch.py kp_des:=2600 kd_des:=28

Note: kp_des / kd_des / vel_limit / accel_limit are passed as CLI arguments
      (--kp-des, --kd-des) via argparse inside the node, NOT as ROS2 parameters.
      This avoids INTEGER vs DOUBLE type conflicts when passing integer values.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('dummyx_dynamics')
    config    = os.path.join(pkg_share, 'config', 'dynamics_config.yaml')

    # ── Declare launch arguments ──────────────────────────────
    args = [
        DeclareLaunchArgument('mode',        default_value='gravity',
                              description='gravity | full_dynamics | none'),
        DeclareLaunchArgument('kp_des',      default_value='2600.0',
                              description='Desired Kp (float)'),
        DeclareLaunchArgument('kd_des',      default_value='28.0',
                              description='Desired Kd (float)'),
        DeclareLaunchArgument('vel_limit',   default_value='0.4',
                              description='Velocity limit turns/s'),
        DeclareLaunchArgument('accel_limit', default_value='0.08',
                              description='Accel limit turns/s²'),
    ]

    # ── Node: pass numeric args via CLI arguments (not ROS2 params) ──
    # YAML config only provides grav_scale and accel_filter_hz (arrays/floats
    # that do not come from launch args, so no type conflict).
    gc_node = Node(
        package='dummyx_dynamics',
        executable='gravity_controller',
        name='gravity_controller_node',
        output='screen',
        # Only YAML params (grav_scale, accel_filter_hz) — no numeric overrides
        parameters=[config],
        # Numeric gains passed as CLI args → parsed by argparse inside node
        arguments=[
            '--mode',        LaunchConfiguration('mode'),
            '--kp-des',      LaunchConfiguration('kp_des'),
            '--kd-des',      LaunchConfiguration('kd_des'),
            '--vel-limit',   LaunchConfiguration('vel_limit'),
            '--accel-limit', LaunchConfiguration('accel_limit'),
        ],
    )

    return LaunchDescription(args + [gc_node])
