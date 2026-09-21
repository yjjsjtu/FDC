#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

import math
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Dict

import moteus


# Moteus Mode Constants (from moteus register 0x000)
MODE_STOPPED = 0
MODE_FAULT = 1
MODE_POSITION = 5
MODE_TIMEOUT = 11
MODE_ZERO_VEL = 12


@dataclass
class MotorStatus:
    mode: int = 0
    fault: int = 0

    # GUI compatibility flags
    motor_enabled: bool = False
    target_reached: bool = False
    current_limit: bool = False
    self_test_error: bool = False
    encoder_error: bool = False
    over_voltage: bool = False
    under_voltage: bool = False
    over_current: bool = False
    status_code: int = 0
    errors_code: int = 0


class MotorConfigs:
    def __init__(self):
        self.current_limit = {'current_limit': 2.0}
        self.control_mode = {'control_mode': 5}  # Position
        self.velocity = {'velocity': 2.0}  # rev/s (motor side)
        self.acceleration = {'acceleration': 5.0}  # rev/s^2
        self.deceleration = {'deceleration': 5.0}
        self.protect_over_current = {'protect_over_current': 5.0}
        self.nodeid = {'node_id': 0}
        self.kp_gain = {'kp_gain': 20.0}
        self.kd_gain = {'kd_gain': 0.05}
        self.ki_gain = {'ki_gain': 0.0}


class Motor:
    def __init__(self, controller: moteus.Controller, node_id: int, reduction: float):
        self.controller = controller
        self.node_id = node_id
        self.reduction = reduction
        self.status = MotorStatus()
        self.configs = MotorConfigs()
        self.last_message_time = 0

        # State data
        self.position = 0.0       # Degrees (output side)
        self.velocity = 0.0       # deg/s
        self.torque = 0.0         # Nm
        self.saved_position = 0.0

        self.motor_current = 0.0
        self.bus_voltage = 0.0
        self.bus_current = 0.0
        self.motor_power = 0.0
        self.mosfet_temp = 0.0
        self.calibration_progress = 0

        self.lock = threading.RLock()
        self.zero_offset = 0.0

        # Pending command (set by sync threads, executed by async loop)
        self._pending_command = None
        self._pending_lock = threading.Lock()

    # --- Unit conversion ---

    def degrees_to_motor_rev(self, degrees: float) -> float:
        """Convert output degrees to motor revolutions."""
        return degrees / 360.0 * self.reduction

    def motor_rev_to_degrees(self, motor_rev: float) -> float:
        """Convert motor revolutions to output degrees."""
        return motor_rev / self.reduction * 360.0

    # --- Async commands (called from async context) ---

    async def async_query(self):
        """Query motor state and update internal data (filtering out mixed responses)."""
        import asyncio
        try:
            # Query motor with a short timeout to prevent hanging on missing nodes
            state = await asyncio.wait_for(self.controller.query(), timeout=0.05)
            if state is None:
                self.mark_offline()
                return
            
            # CRITICAL: Validate that the response ID matches our node_id.
            # PythonCan transport may return the next available packet on the bus 
            # even if it's from a different motor!
            if hasattr(state, 'id') and state.id == self.node_id:
                self.update_from_result(state)
            else:
                # Wrong motor responded, meaning this motor is likely offline
                print(f"Motor {self.node_id} got mismatched query result: id_attr={getattr(state, 'id', 'N/A')}")
                self.mark_offline()
        except asyncio.TimeoutError:
            self.mark_offline()
        except Exception as e:
            self.mark_offline()
            print(f"Motor {self.node_id}: query error: {e}")

    def update_from_result(self, state):
        """Update motor state from a moteus query result."""
        values = state.values
        mode = values.get(moteus.Register.MODE, 0)
        position_rev = values.get(moteus.Register.POSITION, 0.0)
        velocity_rev_s = values.get(moteus.Register.VELOCITY, 0.0)
        torque_nm = values.get(moteus.Register.TORQUE, 0.0)
        voltage = values.get(moteus.Register.VOLTAGE, 0.0)
        temperature = values.get(moteus.Register.TEMPERATURE, 0.0)
        fault = values.get(moteus.Register.FAULT, 0)

        with self.lock:
            self.status.mode = mode
            self.status.fault = fault
            self.status.motor_enabled = (mode == MODE_POSITION or
                                         mode == MODE_TIMEOUT or
                                         mode == MODE_ZERO_VEL)
            self.status.errors_code = fault

            self.position = self.motor_rev_to_degrees(position_rev)
            self.velocity = self.motor_rev_to_degrees(velocity_rev_s)
            self.torque = torque_nm
            self.motor_current = torque_nm
            self.bus_voltage = voltage
            self.mosfet_temp = temperature
            self.motor_power = voltage * abs(torque_nm) if voltage else 0
            self.last_message_time = time.time()

    def mark_offline(self):
        """Mark the motor as offline when it stops responding."""
        with self.lock:
            self.status.mode = -1
            self.status.motor_enabled = False

    async def async_enable(self):
        """Enable motor (enter position mode, hold current position)."""
        try:
            await self.controller.set_position(
                position=math.nan,
                velocity=0.0,
                watchdog_timeout=math.nan,
                query=True,
            )
            print(f"Motor {self.node_id}: Enabled")
        except Exception as e:
            print(f"Motor {self.node_id}: enable error: {e}")

    async def async_disable(self):
        """Disable motor (stop)."""
        try:
            await self.controller.set_stop()
            print(f"Motor {self.node_id}: Disabled")
        except Exception as e:
            print(f"Motor {self.node_id}: disable error: {e}")

    async def async_set_position(self, position_degrees: float):
        """Move to target position in degrees."""
        target_motor_rev = self.degrees_to_motor_rev(position_degrees)
        vel_limit = self.configs.velocity['velocity']
        accel_limit = self.configs.acceleration['acceleration']
        max_torque = self.configs.current_limit['current_limit']

        try:
            await self.controller.set_position(
                position=target_motor_rev,
                velocity=0.0,
                velocity_limit=vel_limit,
                accel_limit=accel_limit,
                maximum_torque=max_torque,
                watchdog_timeout=math.nan,
                query=True,
            )
            print(f"Motor {self.node_id}: Set Position {position_degrees:.2f} deg "
                  f"({target_motor_rev:.3f} motor rev)")
        except Exception as e:
            print(f"Motor {self.node_id}: set_position error: {e}")

    async def async_set_home(self, position_degrees: float = 0.0):
        """Set current position as home (or specified offset)."""
        motor_rev = self.degrees_to_motor_rev(position_degrees)
        try:
            stream = moteus.Stream(self.controller)
            await stream.write_message(f"d exact {motor_rev}")
            await stream.flush()
            print(f"Motor {self.node_id}: Set Home at {position_degrees} deg")
        except Exception as e:
            print(f"Motor {self.node_id}: set_home error: {e}")

    async def async_error_reset(self):
        """Clear errors by stopping motor."""
        try:
            await self.controller.set_stop()
            print(f"Motor {self.node_id}: Errors cleared")
        except Exception as e:
            print(f"Motor {self.node_id}: error_reset error: {e}")

    async def async_set_config(self, values):
        """Apply config parameters via diagnostic stream."""
        try:
            stream = moteus.Stream(self.controller)

            kp = float(values[6].value) if hasattr(values[6], 'value') else float(values[6])
            await stream.write_message(f"conf set servo.pid_position.kp {kp}")

            kd = float(values[7].value) if hasattr(values[7], 'value') else float(values[7])
            await stream.write_message(f"conf set servo.pid_position.kd {kd}")

            ki = float(values[8].value) if hasattr(values[8], 'value') else float(values[8])
            await stream.write_message(f"conf set servo.pid_position.ki {ki}")

            await stream.flush()
            print(f"Motor {self.node_id}: Config applied (kp={kp}, kd={kd}, ki={ki})")
        except Exception as e:
            print(f"Motor {self.node_id}: set_config error: {e}")

    async def async_save_configs(self):
        """Save configuration to flash."""
        try:
            stream = moteus.Stream(self.controller)
            await stream.write_message("conf write")
            await stream.flush()
            print(f"Motor {self.node_id}: Config saved to flash")
        except Exception as e:
            print(f"Motor {self.node_id}: save_configs error: {e}")

    async def async_start_calibration(self):
        """Start motor calibration."""
        try:
            stream = moteus.Stream(self.controller)
            await stream.write_message("d cal 2.0")
            await stream.flush()
            print(f"Motor {self.node_id}: Calibration started")
        except Exception as e:
            print(f"Motor {self.node_id}: calibration error: {e}")

    # --- Sync wrappers (called from GUI threads) ---

    def _schedule_command(self, cmd_name: str, **kwargs):
        """Schedule an async command to be executed by the polling loop."""
        with self._pending_lock:
            self._pending_command = (cmd_name, kwargs)

    def get_pending_command(self):
        """Get and clear the pending command."""
        with self._pending_lock:
            cmd = self._pending_command
            self._pending_command = None
            return cmd

    def enable(self):
        self._schedule_command('enable')

    def disable(self):
        self._schedule_command('disable')

    def error_resets(self):
        self._schedule_command('error_reset')

    def set_position(self, position_degrees: float):
        self._schedule_command('set_position', position_degrees=position_degrees)

    def set_home(self, position_degrees: float = 0.0):
        self._schedule_command('set_home', position_degrees=position_degrees)

    def set_config(self, values):
        self._schedule_command('set_config', values=values)

    def save_configs(self):
        self._schedule_command('save_configs')

    def start_calibration(self):
        self._schedule_command('start_calibration')

    async def execute_pending(self):
        """Execute pending command (called from async context)."""
        cmd = self.get_pending_command()
        if cmd is None:
            return

        cmd_name, kwargs = cmd
        method = getattr(self, f'async_{cmd_name}', None)
        if method:
            await method(**kwargs)

    # --- Legacy methods (GUI compatibility) ---

    def degrees_to_turns(self, degrees: float) -> float:
        return degrees / 360.0 * self.reduction

    def turns_to_degrees(self, turns: float) -> float:
        return turns / self.reduction * 360.0

    def reference_status(self):
        pass

    def reference_value1(self):
        pass

    def reference_saved_position(self):
        pass

    def set_speed_mode(self, values):
        """Update velocity/acceleration configs."""
        try:
            self.configs.velocity['velocity'] = values[0]
            self.configs.acceleration['acceleration'] = values[1]
            self.configs.deceleration['deceleration'] = values[2]
            print(f"Motor {self.node_id}: Speed mode set "
                  f"vel={values[0]} accel={values[1]} decel={values[2]}")
        except Exception as e:
            print(f"Motor {self.node_id}: set_speed_mode error: {e}")

    def set_damping_mode(self):
        self._schedule_command('enable')

    def set_stop_damping_mode(self):
        self._schedule_command('disable')

    def reset_all_to_defaults(self):
        self.configs = MotorConfigs()
        print(f"Motor {self.node_id}: Parameters reset to defaults")

    def get_config(self):
        pass

    def set_homing(self, homing_current, homing_position, homing_expected_position, timeout, tolerance):
        print(f"Motor {self.node_id}: Homing not yet implemented for moteus")
        return 'success'

    def get_status_dict(self) -> dict:
        with self.lock:
            mode_names = {
                -1: 'Offline', 0: 'Stopped', 1: 'Fault', 5: 'Position',
                11: 'Timeout', 12: 'ZeroVel',
            }
            return {
                'node_id': self.node_id,
                'enabled': self.status.motor_enabled,
                'target_reached': self.status.target_reached,
                'current_limit': self.status.current_limit,
                'errors': {
                    'fault': self.status.fault,
                },
                'raw_status': mode_names.get(self.status.mode, str(self.status.mode)),
                'raw_error': self.status.fault,
                'last_update': time.time() - self.last_message_time if self.last_message_time else 999,
                'position': self.position,
                'saved_position': self.saved_position,
                'velocity': self.velocity,
                'torque': self.torque,
                'current': self.motor_current,
                'bus_voltage': self.bus_voltage,
                'bus_current': self.bus_current,
                'motor_power': self.motor_power,
                'mosfet_temp': self.mosfet_temp,
                'calibration_progress': self.calibration_progress,
            }

