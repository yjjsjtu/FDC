#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

"""
RdriveS1 0.5.6 CAN Driver for DummyX2 Dynamics Testing

This module provides CAN communication with RdriveS1 motor controllers
for real-time torque compensation. Reference: usb2can_node.py
"""

import can
import struct
import time
import math
from threading import Lock
from typing import Dict, Optional, Callable
from dataclasses import dataclass


# CAN Command Definitions (RdriveS1 0.5.6 Simple CAN Protocol)
CMD_ID_HEARTBEAT = 0x01
CMD_ID_SET_AXIS_STATE = 0x07
CMD_ID_GET_ENCODER_ESTIMATES = 0x09
CMD_ID_SET_CONTROLLER_MODES = 0x0B
CMD_ID_SET_INPUT_POS = 0x0C
CMD_ID_SET_INPUT_VEL = 0x0D
CMD_ID_SET_INPUT_TORQUE = 0x0E
CMD_ID_SET_LIMITS = 0x0F

# Axis States
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# Control Modes
CONTROL_MODE_VOLTAGE_CONTROL = 0
CONTROL_MODE_TORQUE_CONTROL = 1
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3

# Input Modes
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2
INPUT_MODE_POS_FILTER = 3
INPUT_MODE_TRAP_TRAJ = 5
INPUT_MODE_TORQUE_RAMP = 6


@dataclass
class MotorState:
    """Motor state"""
    position: float = 0.0      # Position (turns)
    velocity: float = 0.0      # Velocity (turns/s)
    timestamp: float = 0.0     # Timestamp


class RdriveS1Motor:
    """RdriveS1 motor control class (via CAN)"""

    def __init__(self, bus: can.Bus, node_id: int, reduction: float = 1.0,
                 direction: int = 1, vel_limit: float = 10.0,
                 current_limit: float = 20.0):
        """
        Initialize RdriveS1 motor

        Args:
            bus: python-can Bus object
            node_id: RdriveS1 CAN node ID
            reduction: Harmonic reducer ratio
            direction: Motor direction (1=forward, -1=reverse)
            vel_limit: Velocity limit (turns/s)
            current_limit: Current limit (A)
        """
        self.bus = bus
        self.node_id = node_id
        self.reduction = reduction
        self.direction = direction
        self.vel_limit = vel_limit
        self.current_limit = current_limit
        
        self._state = MotorState()
        self._lock = Lock()
        self._enabled = False

    def build_can_id(self, cmd_id: int) -> int:
        """Build CAN ID: (node_id << 5) | cmd_id"""
        return (self.node_id << 5) | cmd_id

    def _send_message(self, cmd_id: int, data: bytes):
        """Send CAN message"""
        msg = can.Message(
            arbitration_id=self.build_can_id(cmd_id),
            data=data,
            is_extended_id=False
        )
        with self._lock:
            self.bus.send(msg)

    def set_axis_state(self, state: int):
        """Set axis state"""
        data = struct.pack("<I", state)
        self._send_message(CMD_ID_SET_AXIS_STATE, data)

    def set_controller_mode(self, control_mode: int, input_mode: int):
        """
        Set controller mode

        Args:
            control_mode: Control mode (CONTROL_MODE_xxx)
            input_mode: Input mode (INPUT_MODE_xxx)
        """
        data = struct.pack("<II", control_mode, input_mode)
        self._send_message(CMD_ID_SET_CONTROLLER_MODES, data)

    def set_limits(self, vel_limit: float, current_limit: float):
        """Set velocity and current limits"""
        data = struct.pack("<ff", vel_limit, current_limit)
        self._send_message(CMD_ID_SET_LIMITS, data)

    def set_input_torque(self, torque: float):
        """
        Set input torque (Nm)

        Note: RdriveS1 torque is motor-side torque, not joint-side
        Conversion based on reduction ratio is needed

        Args:
            torque: Motor-side torque (Nm)
        """
        data = struct.pack("<f", torque)
        self._send_message(CMD_ID_SET_INPUT_TORQUE, data)

    def set_input_velocity(self, velocity: float, torque_ff: float = 0.0):
        """
        Set input velocity and feedforward torque

        Args:
            velocity: Velocity (turns/s)  
            torque_ff: Feedforward torque (Nm)
        """
        data = struct.pack("<ff", velocity, torque_ff)
        self._send_message(CMD_ID_SET_INPUT_VEL, data)

    def enable(self):
        """Enable motor (enter closed-loop control)"""
        # Set limits
        self.set_limits(self.vel_limit, self.current_limit)
        time.sleep(0.01)
        
        # Set to torque control mode, passthrough input
        self.set_controller_mode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH)
        time.sleep(0.01)
        
        # Enter closed-loop control
        self.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        self._enabled = True

    def disable(self):
        """Disable motor (enter idle state)"""
        self.set_axis_state(AXIS_STATE_IDLE)
        self._enabled = False

    def update_encoder_estimates(self, data: bytes):
        """Update encoder estimates (from CAN message)"""
        if len(data) >= 8:
            pos, vel = struct.unpack("<ff", data)
            with self._lock:
                self._state.position = pos
                self._state.velocity = vel
                self._state.timestamp = time.time()

    def get_state(self) -> MotorState:
        """Get current state"""
        with self._lock:
            return MotorState(
                position=self._state.position,
                velocity=self._state.velocity,
                timestamp=self._state.timestamp
            )

    def get_joint_position(self) -> float:
        """
        Get joint position (considering reduction and direction)

        Returns:
            Joint position (rad)
        """
        state = self.get_state()
        # Motor turns -> joint rad
        # Joint position = motor position(turns) / reduction * 2*pi * direction
        return state.position / self.reduction * 2 * math.pi * self.direction

    def get_joint_velocity(self) -> float:
        """
        Get joint velocity (considering reduction and direction)

        Returns:
            Joint velocity (rad/s)
        """
        state = self.get_state()
        # Motor turns/s -> joint rad/s
        return state.velocity / self.reduction * 2 * math.pi * self.direction

    def send_joint_torque(self, joint_torque: float):
        """
        Send joint torque command (auto-convert to motor torque)

        Args:
            joint_torque: Joint-side torque (Nm)
        """
        # Motor torque = joint torque / reduction * direction
        # Harmonic reducer increases torque, so motor needs less torque
        motor_torque = joint_torque / self.reduction * self.direction
        self.set_input_torque(motor_torque)

    @property
    def is_enabled(self) -> bool:
        return self._enabled


class RdriveS1Controller:
    """RdriveS1 controller (manages multiple motors)"""

    def __init__(self, interface: str, channel: str,
                 motor_configs: Dict[int, dict]):
        """
        Initialize controller

        Args:
            interface: CAN interface type (socketcan, slcan, etc.)
            channel: CAN channel (can0, /dev/ttyUSB0, etc.)
            motor_configs: Motor config dict {node_id: {reduction, direction, ...}}
        """
        self.bus = can.Bus(interface=interface, channel=channel,
                          receive_own_messages=True)
        self.motors: Dict[int, RdriveS1Motor] = {}
        self._message_lock = Lock()

        # Initialize motors
        for node_id, config in motor_configs.items():
            self.motors[node_id] = RdriveS1Motor(
                bus=self.bus,
                node_id=node_id,
                reduction=config.get('reduction', 30.0),
                direction=config.get('direction', 1),
                vel_limit=config.get('vel_limit', 10.0),
                current_limit=config.get('current_limit', 20.0)
            )

        # Start CAN message listener
        self.notifier = can.Notifier(self.bus, [self._on_message_received])

    def _on_message_received(self, msg: can.Message):
        """CAN message receive callback"""
        node_id = (msg.arbitration_id >> 5) & 0x3F
        cmd_id = msg.arbitration_id & 0x1F

        if node_id in self.motors:
            if cmd_id == CMD_ID_GET_ENCODER_ESTIMATES:
                self.motors[node_id].update_encoder_estimates(msg.data)

    def enable_all(self):
        """Enable all motors"""
        for motor in self.motors.values():
            motor.enable()
            time.sleep(0.02)

    def disable_all(self):
        """Disable all motors"""
        for motor in self.motors.values():
            motor.disable()
            time.sleep(0.02)

    def get_motor(self, node_id: int) -> Optional[RdriveS1Motor]:
        """Get specified motor"""
        return self.motors.get(node_id)

    def shutdown(self):
        """Shutdown controller"""
        self.disable_all()
        self.notifier.stop()
        self.bus.shutdown()


if __name__ == "__main__":
    """Test RdriveS1 CAN driver"""
    print("RdriveS1 CAN Driver Test\n")
    
    # Test configuration
    motor_configs = {
        1: {'reduction': 30.0, 'direction': 1},
        2: {'reduction': 50.0, 'direction': -1},
    }
    
    print("Test configuration:")
    for node_id, config in motor_configs.items():
        print(f"  Motor {node_id}: reduction={config['reduction']}, direction={config['direction']}")
    
    print("\nNote: Actual testing requires CAN hardware connection")
    print("Please run full test in an environment with hardware")
