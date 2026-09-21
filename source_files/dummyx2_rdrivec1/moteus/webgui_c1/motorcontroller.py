#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

import asyncio
import threading
from typing import Optional, Dict, List

import moteus

from motor import Motor


class MotorController:
    def __init__(self, interface: str = 'socketcan', channel: str = 'can0'):
        self.interface = interface
        self.channel = channel
        self.motors: Dict[int, Motor] = {}
        self.running = False
        self._loop_thread = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._transport = None
        self._initialized = False

        try:
            # Use PythonCan wrapper (available in most moteus versions)
            transport = moteus.PythonCan(
                interface=interface,
                channel=channel,
                disable_brs=True,
            )
            self._transport = transport
            self._initialized = True
            print(f"CAN bus initialized: {interface}/{channel} (disable_brs=True)")
        except Exception as e:
            print(f"Failed to initialize CAN bus: {e}")
            self._transport = None

    def add_motor(self, node_id: int, reduction: float) -> Motor:
        if node_id in self.motors:
            raise ValueError(f"Motor with node ID {node_id} already exists")
        if not self._initialized:
            raise RuntimeError("CAN bus not initialized")

        controller = moteus.Controller(id=node_id, transport=self._transport)
        motor = Motor(controller, node_id, reduction)
        self.motors[node_id] = motor
        return motor

    def start(self):
        """Start the async polling loop in a background thread."""
        if self.running or not self._initialized:
            return

        self.running = True
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()
        print("Motor controller polling started")

    def _run_loop(self):
        """Background thread running the asyncio event loop."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._polling_loop())

    async def _polling_loop(self):
        """Async polling loop: query all motors and execute pending commands."""
        while self.running:
            # First execute pending commands
            for motor in self.motors.values():
                if not self.running:
                    break
                try:
                    await motor.execute_pending()
                except Exception as e:
                    print(f"Motor {motor.node_id}: command error: {e}")

            if not self.running:
                break

    async def _polling_loop(self):
        """Async polling loop: query all motors sequentially."""
        while self.running:
            for motor in list(self.motors.values()):
                if not self.running:
                    break

                # Execute any pending command from GUI thread
                try:
                    await motor.execute_pending()
                except Exception as e:
                    print(f"Motor {motor.node_id}: command error: {e}")

                # Query motor state sequentially
                try:
                    await motor.async_query()
                except Exception as e:
                    print(f"Motor {motor.node_id}: query error: {e}")

            # Poll at ~10 Hz
            await asyncio.sleep(0.05)

    def stop(self):
        """Stop the polling loop."""
        if not self.running:
            return

        self.running = False
        if self._loop_thread:
            self._loop_thread.join(timeout=3.0)
        print("Motor controller stopped")

    def get_motor_status(self, node_id: int) -> Optional[dict]:
        if node_id in self.motors:
            return self.motors[node_id].get_status_dict()
        return None

    def get_all_motor_status(self) -> List[dict]:
        return [motor.get_status_dict() for motor in self.motors.values()]

    def is_initialized(self) -> bool:
        return self._initialized

