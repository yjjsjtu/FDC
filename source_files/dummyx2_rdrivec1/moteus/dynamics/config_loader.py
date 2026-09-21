#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

"""
DummyX2 Robot - Configuration Loader for Dynamics Test

Configuration loading and validation for real-time torque compensation
with RdriveS1 0.5.6 and harmonic reducers.
"""

import yaml
from pathlib import Path
from typing import List, Optional
from dataclasses import dataclass


@dataclass
class MotorConfig:
    """Single motor configuration"""
    node_id: int           # RdriveS1 CAN node ID
    reduction: float       # Harmonic reducer ratio
    direction: int = 1     # Motor direction: 1=forward, -1=reverse
    description: str = ""  # Motor description


@dataclass
class DynamicsTestConfig:
    """Complete dynamics test configuration"""
    can_interface: str          # CAN interface type (socketcan, slcan, etc.)
    can_channel: str            # CAN channel (can0, /dev/ttyUSB0, etc.)
    motors: List[MotorConfig]
    control_rate: int           # Control rate Hz
    compensation_mode: str      # Compensation mode: gravity, full_dynamics, none
    vel_limit: float            # Velocity limit (turns/s)
    current_limit: float        # Current limit (A)
    log_interval: float         # Log print interval (s)
    verbose: bool


def get_default_config() -> DynamicsTestConfig:
    """
    Return hardcoded default configuration (based on usb2can_node.py)

    Returns:
        DynamicsTestConfig: Default configuration object
    """
    # Default motor configuration (6 motors, based on usb2can_node.py)
    default_motors = [
        MotorConfig(
            node_id=1,
            reduction=30.0,
            direction=1,       # Joint1 forward
            description="Joint 1 - Base"
        ),
        MotorConfig(
            node_id=2,
            reduction=50.0,
            direction=-1,      # Joint2 reverse
            description="Joint 2 - Shoulder"
        ),
        MotorConfig(
            node_id=3,
            reduction=50.0,
            direction=-1,      # Joint3 reverse
            description="Joint 3 - Elbow"
        ),
        MotorConfig(
            node_id=4,
            reduction=30.0,
            direction=-1,      # Joint4 reverse
            description="Joint 4 - Wrist1"
        ),
        MotorConfig(
            node_id=5,
            reduction=30.0,
            direction=-1,      # Joint5 reverse
            description="Joint 5 - Wrist2"
        ),
        MotorConfig(
            node_id=6,
            reduction=30.0,
            direction=-1,      # Joint6 reverse
            description="Joint 6 - Wrist3"
        ),
    ]

    return DynamicsTestConfig(
        can_interface="socketcan",
        can_channel="can0",
        motors=default_motors,
        control_rate=200,
        compensation_mode="gravity",
        vel_limit=10.0,        # turns/s
        current_limit=20.0,    # A
        log_interval=0.5,
        verbose=False
    )


def validate_config(config: DynamicsTestConfig) -> List[str]:
    """
    Validate configuration

    Args:
        config: Configuration object

    Returns:
        List of error messages (empty if valid)
    """
    errors = []

    # Validate motor count
    if len(config.motors) == 0:
        errors.append("At least one motor must be configured")

    # Validate motor ID uniqueness
    motor_ids = [m.node_id for m in config.motors]
    if len(motor_ids) != len(set(motor_ids)):
        errors.append("Motor node_id must be unique")

    # Validate control rate
    if config.control_rate <= 0:
        errors.append(f"Control rate must be > 0 (current: {config.control_rate})")

    # Validate compensation mode
    valid_modes = ["gravity", "full_dynamics", "none"]
    if config.compensation_mode not in valid_modes:
        errors.append(
            f"Invalid compensation mode: '{config.compensation_mode}' "
            f"(valid: {', '.join(valid_modes)})"
        )

    # Validate velocity limit
    if config.vel_limit <= 0:
        errors.append(f"Velocity limit must be > 0 (current: {config.vel_limit})")

    # Validate log interval
    if config.log_interval <= 0:
        errors.append(f"Log interval must be > 0 (current: {config.log_interval})")

    return errors


def load_config(config_path: Optional[str] = None) -> DynamicsTestConfig:
    """
    Load configuration file

    Args:
        config_path: Configuration file path (if None, use default path)

    Returns:
        DynamicsTestConfig: Configuration object
    """
    # Use default path if not specified
    if config_path is None:
        config_path = Path(__file__).parent / "dynamics_test.yaml"
    else:
        config_path = Path(config_path)

    # Check if file exists
    if not config_path.exists():
        print(f"Warning: Configuration file not found: {config_path}")
        print("Using default configuration")
        return get_default_config()

    # Read and parse YAML
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            yaml_data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        print(f"Error: YAML format error: {e}")
        print("Using default configuration")
        return get_default_config()
    except Exception as e:
        print(f"Error: Cannot read configuration file: {e}")
        print("Using default configuration")
        return get_default_config()

    # Parse configuration
    try:
        # CAN configuration
        can_config = yaml_data.get('can', {})
        can_interface = can_config.get('interface', 'socketcan')
        can_channel = can_config.get('channel', 'can0')

        # Motor configuration
        motors_data = yaml_data.get('motors', [])
        motors = []
        for motor_data in motors_data:
            motor = MotorConfig(
                node_id=motor_data.get('id'),
                reduction=motor_data.get('reduction', 30.0),
                direction=motor_data.get('direction', 1),
                description=motor_data.get('description', '')
            )
            motors.append(motor)

        # Control configuration
        control_config = yaml_data.get('control', {})
        control_rate = control_config.get('rate', 200)
        compensation_mode = control_config.get('compensation_mode', 'gravity')
        vel_limit = control_config.get('vel_limit', 10.0)
        current_limit = control_config.get('current_limit', 20.0)

        # Logging configuration
        logging_config = yaml_data.get('logging', {})
        log_interval = logging_config.get('print_status_interval', 0.5)
        verbose = logging_config.get('verbose', False)

        # Create configuration object
        config = DynamicsTestConfig(
            can_interface=can_interface,
            can_channel=can_channel,
            motors=motors,
            control_rate=control_rate,
            compensation_mode=compensation_mode,
            vel_limit=vel_limit,
            current_limit=current_limit,
            log_interval=log_interval,
            verbose=verbose
        )

        # Validate configuration
        errors = validate_config(config)
        if errors:
            print(f"Error: Configuration validation failed:")
            for error in errors:
                print(f"  - {error}")
            print("\nUsing default configuration")
            return get_default_config()

        return config

    except Exception as e:
        print(f"Error: Configuration loading error: {e}")
        print("Using default configuration")
        return get_default_config()


def print_config_summary(config: DynamicsTestConfig):
    """
    Print configuration summary

    Args:
        config: Configuration object
    """
    print("\n" + "="*60)
    print("Configuration Summary")
    print("="*60)
    print(f"CAN Interface: {config.can_interface}")
    print(f"CAN Channel: {config.can_channel}")
    print(f"\nMotor Count: {len(config.motors)}")
    for i, motor in enumerate(config.motors, 1):
        print(f"  Motor {i}:")
        print(f"    Node ID: {motor.node_id}")
        print(f"    Reduction: {motor.reduction}")
        direction_str = "forward" if motor.direction == 1 else "reverse"
        print(f"    Direction: {motor.direction} ({direction_str})")
        if motor.description:
            print(f"    Description: {motor.description}")
    print(f"\nControl Rate: {config.control_rate} Hz")
    print(f"Compensation Mode: {config.compensation_mode}")
    print(f"Velocity Limit: {config.vel_limit} turns/s")
    print(f"Current Limit: {config.current_limit} A")
    print(f"Log Interval: {config.log_interval} s")
    print(f"Verbose: {config.verbose}")
    print("="*60 + "\n")


if __name__ == "__main__":
    """Test configuration loading"""
    print("Testing configuration loader\n")

    # Test default configuration
    print("1. Testing default configuration:")
    default_config = get_default_config()
    print_config_summary(default_config)

    # Test loading configuration file
    print("\n2. Testing configuration file loading:")
    config_path = Path(__file__).parent / "dynamics_test.yaml"
    if config_path.exists():
        config = load_config(str(config_path))
        print_config_summary(config)
    else:
        print(f"Configuration file not found: {config_path}")

    print("\nConfiguration loader test complete")
