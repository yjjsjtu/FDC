#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

"""
DummyX2 Robot - Real-time Torque Compensation using Pinocchio

This program implements real-time dynamics-based torque compensation for DummyX2:
- Uses Pinocchio for inverse dynamics computation
- Controls RdriveS1 motors via CAN (firmware 0.5.6)
- Supports harmonic reducers with configurable ratios
- Provides gravity compensation and full dynamics compensation
"""

import sys
import time
import numpy as np
import pinocchio as pin
from pathlib import Path
import threading
import argparse

from config_loader import (
    DynamicsTestConfig, load_config, get_default_config, print_config_summary
)
from rdrive_s1_can_driver import RdriveS1Controller

# Default configuration file path
DEFAULT_CONFIG_PATH = Path(__file__).parent / "dynamics_test.yaml"


class DummyX2Dynamics:
    """Wrapper class for DummyX2 robot dynamics using Pinocchio"""

    def __init__(self, urdf_path: str):
        """
        Initialize the dynamics model

        Args:
            urdf_path: Path to the URDF file
        """
        # Load the model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # Get model information
        self.nq = self.model.nq  # Number of position variables
        self.nv = self.model.nv  # Number of velocity variables

        print(f"Model loaded: {self.model.name}")
        print(f"Joint count: {self.nq}")
        print(f"Joint names: {[self.model.names[i] for i in range(1, self.model.njoints)]}")

    def compute_inverse_dynamics(self, q, v, a):
        """
        Compute inverse dynamics: tau = M(q)*a + C(q,v) + g(q)

        Args:
            q: Joint positions (nq,)
            v: Joint velocities (nv,)
            a: Joint accelerations (nv,)

        Returns:
            tau: Required joint torques (nv,)
        """
        tau = pin.rnea(self.model, self.data, q, v, a)
        return tau

    def compute_gravity(self, q):
        """
        Compute gravity torques g(q)

        Args:
            q: Joint positions (nq,)

        Returns:
            g: Gravity torques (nv,)
        """
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        return g

    def compute_mass_matrix(self, q):
        """
        Compute the joint space inertia matrix M(q)

        Args:
            q: Joint positions (nq,)

        Returns:
            M: Mass matrix (nv, nv)
        """
        M = pin.crba(self.model, self.data, q)
        return M


class RealtimeTorqueController:
    """
    Real-time torque compensation controller for DummyX2 robot

    Integrates Pinocchio dynamics with RdriveS1 motor control
    """

    def __init__(self, config: DynamicsTestConfig = None):
        """
        Initialize the real-time controller

        Args:
            config: Configuration object (if None, use default config)
        """
        # Load configuration
        if config is None:
            config = get_default_config()

        self.config = config

        # Find URDF file
        urdf_path = Path(__file__).parent / "urdf" / "dummyx2.urdf"

        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        # Initialize dynamics model
        self.dynamics = DummyX2Dynamics(str(urdf_path))

        # Number of controlled motors (should match URDF joints)
        self.num_motors = len(config.motors)

        # Motor node IDs (ordered)
        self.motor_node_ids = [m.node_id for m in config.motors]

        # Motor directions
        self.motor_directions = np.array([m.direction for m in config.motors])

        # Motor reductions
        self.motor_reductions = np.array([m.reduction for m in config.motors])

        # Initialize RdriveS1 controller
        print(f"\nInitializing CAN controller: {config.can_interface}:{config.can_channel}")
        motor_configs = {}
        for motor in config.motors:
            motor_configs[motor.node_id] = {
                'reduction': motor.reduction,
                'direction': motor.direction,
                'vel_limit': config.vel_limit,
                'current_limit': config.current_limit
            }

        self.controller = RdriveS1Controller(
            interface=config.can_interface,
            channel=config.can_channel,
            motor_configs=motor_configs
        )

        # Control parameters
        self.control_rate = config.control_rate  # Hz
        self.dt = 1.0 / self.control_rate

        # Control mode
        self.compensation_mode = config.compensation_mode

        # Logging parameters
        self.log_interval = config.log_interval
        self.verbose = config.verbose

        # Control thread
        self._control_thread = None
        self._running = False
        self._stop_event = threading.Event()

        # Current state
        self._current_q = np.zeros(self.num_motors)
        self._current_v = np.zeros(self.num_motors)
        self._current_tau_cmd = np.zeros(self.num_motors)
        self._state_lock = threading.Lock()

        # Velocity history for acceleration estimation
        self._previous_v = np.zeros(self.num_motors)
        self._filtered_a = np.zeros(self.num_motors)

        # Low-pass filter parameter for acceleration
        self._accel_filter_cutoff = 5.0  # Hz
        self._accel_filter_alpha = self._compute_filter_alpha(self._accel_filter_cutoff)

    def _compute_filter_alpha(self, cutoff_freq):
        """Compute first-order low-pass filter coefficient"""
        rc = 1.0 / (2.0 * np.pi * cutoff_freq)
        alpha = self.dt / (rc + self.dt)
        return alpha

    def _apply_lowpass_filter(self, new_value, filtered_value, alpha):
        """Apply first-order low-pass filter"""
        return alpha * new_value + (1.0 - alpha) * filtered_value

    def setup(self):
        """Setup CAN connection (already done in __init__)"""
        print("\n" + "="*60)
        print("Setting up real-time torque control system")
        print("="*60)
        print(f"Motor count: {self.num_motors}")
        for i, motor_config in enumerate(self.config.motors):
            print(f"  Motor {i+1} (Node {motor_config.node_id}): "
                  f"reduction={motor_config.reduction}, "
                  f"direction={motor_config.direction}")

    def enable_motors(self):
        """Enable all motors"""
        print("\nEnabling motors...")
        self.controller.enable_all()

        # Wait for encoder feedback
        time.sleep(0.5)

        # Print initial positions
        for i, node_id in enumerate(self.motor_node_ids):
            motor = self.controller.get_motor(node_id)
            if motor:
                joint_pos = motor.get_joint_position()
                print(f"Motor {i+1} (Node {node_id}) joint position: {joint_pos:.4f} rad")

        print("Motors enabled")

    def disable_motors(self):
        """Disable all motors"""
        print("\nDisabling motors...")
        self.controller.disable_all()
        print("Motors disabled")

    def get_current_state(self):
        """
        Get current joint state from motors

        Returns:
            q: Joint positions (num_motors,) in rad
            v: Joint velocities (num_motors,) in rad/s
        """
        q = np.zeros(self.num_motors)
        v = np.zeros(self.num_motors)

        for i, node_id in enumerate(self.motor_node_ids):
            motor = self.controller.get_motor(node_id)
            if motor:
                q[i] = motor.get_joint_position()
                v[i] = motor.get_joint_velocity()

        return q, v

    def compute_compensation_torque(self, q, v, mode="gravity"):
        """
        Compute compensation torque based on mode

        Args:
            q: Joint positions (num_motors,) in rad
            v: Joint velocities (num_motors,) in rad/s
            mode: Compensation mode ("gravity", "full_dynamics", "none")

        Returns:
            tau: Compensation torque (num_motors,) in Nm
        """
        if mode == "none":
            return np.zeros(self.num_motors)
        elif mode == "gravity":
            # Gravity compensation only
            tau = self.dynamics.compute_gravity(q)
        elif mode == "full_dynamics":
            # Full inverse dynamics (gravity + coriolis + inertia)
            a_raw = (v - self._previous_v) / self.dt

            # Apply low-pass filter to acceleration
            for i in range(self.num_motors):
                self._filtered_a[i] = self._apply_lowpass_filter(
                    a_raw[i],
                    self._filtered_a[i],
                    self._accel_filter_alpha
                )

            tau = self.dynamics.compute_inverse_dynamics(q, v, self._filtered_a)
        else:
            raise ValueError(f"Unknown compensation mode: {mode}")

        return tau

    def send_torque_command(self, tau):
        """
        Send torque commands to motors

        Args:
            tau: Joint torques (num_motors,) in Nm (joint coordinates)
        """
        for i, node_id in enumerate(self.motor_node_ids):
            motor = self.controller.get_motor(node_id)
            if motor:
                motor.send_joint_torque(tau[i])

    def _control_loop(self):
        """Main control loop running at specified rate"""
        print(f"\nControl loop started (rate: {self.control_rate} Hz)")

        loop_count = 0
        last_print_time = time.time()

        while not self._stop_event.is_set():
            start_time = time.time()

            try:
                # Get current state
                q, v = self.get_current_state()

                # Compute compensation torque
                tau = self.compute_compensation_torque(q, v, self.compensation_mode)

                # Send torque command
                self.send_torque_command(tau)

                # Update internal state
                with self._state_lock:
                    self._current_q = q
                    self._previous_v = self._current_v.copy()
                    self._current_v = v
                    self._current_tau_cmd = tau

                # Print status periodically
                loop_count += 1
                if time.time() - last_print_time >= self.log_interval:
                    q_str = ', '.join([f"{qi:6.3f}" for qi in q])
                    v_str = ', '.join([f"{vi:6.3f}" for vi in v])
                    tau_str = ', '.join([f"{ti:6.3f}" for ti in tau])
                    print(f"\rPos: [{q_str}] rad  "
                          f"Vel: [{v_str}] rad/s  "
                          f"Torque: [{tau_str}] Nm", end='')
                    last_print_time = time.time()

            except Exception as e:
                print(f"\nControl loop error: {e}")
                break

            # Maintain control rate
            elapsed = time.time() - start_time
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -self.dt:
                print(f"\nWarning: Control loop timeout ({elapsed*1000:.1f} ms)")

        print("\nControl loop stopped")

    def start_control(self, mode="gravity"):
        """
        Start the control loop

        Args:
            mode: Compensation mode ("gravity", "full_dynamics", "none")
        """
        if self._running:
            print("Control loop already running")
            return

        self.compensation_mode = mode
        print(f"\nStarting torque compensation control (mode: {mode})")

        self._running = True
        self._stop_event.clear()
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    def stop_control(self):
        """Stop the control loop"""
        if not self._running:
            return

        print("\n\nStopping control loop...")
        self._stop_event.set()

        if self._control_thread:
            self._control_thread.join(timeout=2.0)

        self._running = False

    def get_state_snapshot(self):
        """Get current state snapshot (thread-safe)"""
        with self._state_lock:
            return {
                'q': self._current_q.copy(),
                'v': self._current_v.copy(),
                'a': self._filtered_a.copy(),
                'tau': self._current_tau_cmd.copy()
            }

    def shutdown(self):
        """Shutdown the controller"""
        print("\n" + "="*60)
        print("Shutting down controller")
        print("="*60)

        # Stop control loop
        self.stop_control()

        # Disable motors and close CAN
        self.controller.shutdown()

        print("Controller shutdown complete")


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="DummyX2 Robot - Real-time Torque Compensation Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Usage examples:
  # Use default configuration file
  python dynamics_test.py

  # Specify configuration file
  python dynamics_test.py --config /path/to/config.yaml

  # Run demo directly (skip menu)
  python dynamics_test.py --demo gravity
        """
    )

    parser.add_argument(
        '--config', '-c',
        type=str,
        default=None,
        help=f'Configuration file path (default: {DEFAULT_CONFIG_PATH})'
    )

    parser.add_argument(
        '--mode', '-m',
        type=str,
        choices=['gravity', 'full_dynamics', 'none'],
        default=None,
        help='Compensation mode (overrides config file)'
    )

    parser.add_argument(
        '--control-rate',
        type=int,
        default=None,
        help='Control rate Hz (overrides config file)'
    )

    parser.add_argument(
        '--demo',
        type=str,
        choices=['gravity', 'full_dynamics', 'comparison', 'interactive'],
        default=None,
        help='Run specified demo directly'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Verbose output mode'
    )

    return parser.parse_args()


def demo_gravity_compensation(config=None):
    """Demonstrate gravity compensation"""
    if config is None:
        config = get_default_config()

    print("\n" + "="*60)
    print("Demo: Gravity Compensation Mode")
    print("="*60)
    print("Robot will maintain current position, compensating gravity only")
    print("You can manually move the arm to feel the gravity compensation effect")
    print("="*60)

    controller = RealtimeTorqueController(config)

    try:
        controller.setup()
        controller.enable_motors()
        controller.start_control(mode="gravity")

        print("\nPress Ctrl+C to stop...")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nUser interrupt")

    finally:
        controller.shutdown()


def demo_full_dynamics_compensation(config=None):
    """Demonstrate full dynamics compensation"""
    if config is None:
        config = get_default_config()

    print("\n" + "="*60)
    print("Demo: Full Dynamics Compensation Mode")
    print("="*60)
    print("Compensating gravity, Coriolis, and centrifugal forces")
    print("="*60)

    controller = RealtimeTorqueController(config)

    try:
        controller.setup()
        controller.enable_motors()
        controller.start_control(mode="full_dynamics")

        print("\nPress Ctrl+C to stop...")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nUser interrupt")

    finally:
        controller.shutdown()


def demo_comparison(config=None):
    """Compare different compensation modes"""
    if config is None:
        config = get_default_config()

    print("\n" + "="*60)
    print("Demo: Compensation Mode Comparison")
    print("="*60)

    controller = RealtimeTorqueController(config)

    try:
        controller.setup()
        controller.enable_motors()

        modes = ["none", "gravity", "full_dynamics"]
        duration = 10.0  # seconds per mode

        for mode in modes:
            print(f"\n\n{'='*60}")
            print(f"Testing mode: {mode}")
            print(f"Duration: {duration} seconds")
            print(f"{'='*60}")

            controller.start_control(mode=mode)
            time.sleep(duration)
            controller.stop_control()

            state = controller.get_state_snapshot()
            print(f"\nFinal state:")
            print(f"  Position: {state['q']} rad")
            print(f"  Velocity: {state['v']} rad/s")
            print(f"  Torque: {state['tau']} Nm")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n\nUser interrupt")
    finally:
        controller.shutdown()


def interactive_mode(config=None):
    """Interactive control mode"""
    if config is None:
        config = get_default_config()

    print("\n" + "="*60)
    print("DummyX2 Robot - Real-time Torque Compensation Control")
    print("="*60)

    controller = RealtimeTorqueController(config)

    try:
        controller.setup()
        controller.enable_motors()

        while True:
            print("\n" + "="*60)
            print("Select operation:")
            print("1. Start gravity compensation")
            print("2. Start full dynamics compensation")
            print("3. Stop compensation")
            print("4. View current state")
            print("5. Adjust control parameters")
            print("0. Exit")
            print("="*60)

            choice = input("\nEnter selection (0-5): ").strip()

            if choice == '1':
                controller.start_control(mode="gravity")
                print("\nGravity compensation started")
                print("Press any key to return to menu...")
                input()

            elif choice == '2':
                controller.start_control(mode="full_dynamics")
                print("\nFull dynamics compensation started")
                print("Press any key to return to menu...")
                input()

            elif choice == '3':
                controller.stop_control()
                print("\nCompensation stopped")

            elif choice == '4':
                state = controller.get_state_snapshot()
                print(f"\nCurrent state:")
                print(f"  Joint position: {np.rad2deg(state['q'])} deg")
                print(f"  Joint velocity: {state['v']} rad/s")
                print(f"  Joint acceleration: {state['a']} rad/s^2")
                print(f"  Compensation torque: {state['tau']} Nm")
                print(f"\nControl parameters:")
                print(f"  Compensation mode: {controller.compensation_mode}")
                print(f"  Control rate: {controller.control_rate} Hz")
                print(f"  Accel filter cutoff: {controller._accel_filter_cutoff} Hz")

            elif choice == '5':
                print(f"\nCurrent parameters:")
                print(f"  Control rate: {controller.control_rate} Hz")
                print(f"  Accel filter cutoff: {controller._accel_filter_cutoff} Hz")

                try:
                    fc_new = float(input(
                        f"Enter new filter cutoff (current: {controller._accel_filter_cutoff} Hz, press Enter to skip): "
                    ).strip() or controller._accel_filter_cutoff)

                    controller._accel_filter_cutoff = fc_new
                    controller._accel_filter_alpha = controller._compute_filter_alpha(fc_new)
                    print("Parameters updated")
                    print(f"New filter alpha: {controller._accel_filter_alpha:.4f}")
                except ValueError:
                    print("Invalid input, parameters unchanged")

            elif choice == '0':
                break

            else:
                print("Invalid selection")

    except KeyboardInterrupt:
        print("\n\nUser interrupt")
    finally:
        controller.shutdown()


def main():
    """Main function"""
    args = parse_arguments()

    # Load configuration
    try:
        if args.config:
            print(f"Loading configuration file: {args.config}")
            config = load_config(args.config)
        else:
            config_path = DEFAULT_CONFIG_PATH
            if config_path.exists():
                print(f"Using default configuration file: {config_path}")
                config = load_config(str(config_path))
            else:
                print(f"Configuration file not found, using built-in defaults")
                config = get_default_config()

        # Command line overrides
        if args.mode:
            config.compensation_mode = args.mode
            print(f"Compensation mode overridden by command line: {args.mode}")

        if args.control_rate:
            config.control_rate = args.control_rate
            print(f"Control rate overridden by command line: {args.control_rate} Hz")

        if args.verbose:
            config.verbose = True

        print_config_summary(config)

    except Exception as e:
        print(f"Configuration loading failed: {e}")
        print("Using built-in defaults")
        config = get_default_config()

    # Run demo or interactive mode
    try:
        if args.demo:
            if args.demo == 'gravity':
                demo_gravity_compensation(config)
            elif args.demo == 'full_dynamics':
                demo_full_dynamics_compensation(config)
            elif args.demo == 'comparison':
                demo_comparison(config)
            elif args.demo == 'interactive':
                interactive_mode(config)
        else:
            # Interactive menu mode
            print("\n" + "="*60)
            print("DummyX2 Robot - Real-time Torque Compensation System")
            print("="*60)
            print("\nSelect demo mode:")
            print("1. Gravity compensation demo")
            print("2. Full dynamics compensation demo")
            print("3. Compensation mode comparison")
            print("4. Interactive control")
            print("0. Exit")

            choice = input("\nEnter selection (0-4): ").strip()

            if choice == '1':
                demo_gravity_compensation(config)
            elif choice == '2':
                demo_full_dynamics_compensation(config)
            elif choice == '3':
                demo_comparison(config)
            elif choice == '4':
                interactive_mode(config)
            elif choice == '0':
                print("Exit")
            else:
                print("Invalid selection")

    except KeyboardInterrupt:
        print("\n\nExit")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
