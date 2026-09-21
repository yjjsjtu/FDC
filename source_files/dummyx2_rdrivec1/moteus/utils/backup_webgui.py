#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Moteus 6-Joint Real-Time Web Dashboard
=======================================
NiceGUI-based web GUI for monitoring 6 moteus controllers (IDs 1-6).
Displays position, voltage, and MOSFET temperature in real time.

Uses the standard moteus transport layer (auto-detects fdcanusb, gs_usb,
socketcan, etc.).  BRS and other CAN options are controlled via CLI flags.

Usage:
    # Install dependencies
    pip install nicegui moteus python-can

    # Auto-detect transport (macOS fdcanusb / gs_usb)
    python webgui.py --disable-brs

    # Force socketcan on Linux
    python webgui.py --force-transport pythoncan --can-iface socketcan --can-chan can0
"""

import argparse
import asyncio
import json
import math
import time
import sys
import io
from collections import deque

from nicegui import ui, app

# ---------------------------------------------------------------------------
# moteus imports
# ---------------------------------------------------------------------------
sys.path.insert(0, './lib/python')
import moteus
from moteus.protocol import Register

# ---------------------------------------------------------------------------
# Command-line arguments (standard moteus transport args)
# ---------------------------------------------------------------------------
_parser = argparse.ArgumentParser(description='Moteus Web Dashboard')
_parser.add_argument('--disable-brs', action='store_true',
                     help='Disable CAN FD Bit Rate Switching (1 Mbps only)')
moteus.make_transport_args(_parser)
_args = _parser.parse_args()
if _args.disable_brs:
    _args.can_disable_brs = True

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
JOINT_IDS = list(range(1, 7))          # Motor IDs 1-6
QUERY_INTERVAL_S = 0.02                # 50 Hz CAN query cycle
UI_REFRESH_INTERVAL_S = 0.1            # 10 Hz UI update
QUERY_TIMEOUT_S = 0.05                 # 50ms timeout per motor query

# ---------------------------------------------------------------------------
# Shared state - written by CAN task, read by UI
# ---------------------------------------------------------------------------
motor_data = {
    joint_id: {
        'position': None,
        'velocity': None,
        'torque': None,
        'q_current': None,
        'voltage': None,
        'temperature': None,
        'motor_temperature': None,
        'mode': None,
        'fault': None,
        'last_update': 0.0,
        'online': False,
    }
    for joint_id in JOINT_IDS
}

# PID parameters for each joint
PID_PARAMS = ['kp', 'ki', 'kd', 'iratelimit', 'ilimit', 'max_desired_rate']
pid_data = {
    jid: {p: None for p in PID_PARAMS}
    for jid in JOINT_IDS
}
pid_read_done = False
can_connected = False
can_error_msg = ''

# History data for charts (last 120s at ~50Hz = 6000 points)
HISTORY_LEN = 6000
def _new_hist():
    return {'times': deque(maxlen=HISTORY_LEN), 'values': deque(maxlen=HISTORY_LEN)}

history_data = {
    jid: {
        'pos': _new_hist(),
        'vel': _new_hist(),
        'torq': _new_hist(),
        'cur': _new_hist()
    }
    for jid in JOINT_IDS
}


# Command queue: UI thread pushes commands, CAN loop processes them
# Commands: ('stop',) ('position', pos, torque, velocity, accel)
#           ('teach_start',) ('teach_stop',) ('teach_repeat',)
command_queue = deque()

# Teaching state
teaching_active = False
teaching_frames = []
replaying_active = False
teaching_start_time = 0.0
teaching_last_record_time = 0.0
TEACHING_RECORD_INTERVAL = 0.1   # 10 Hz
RECORDING_FILE = 'recording.json'

# Async Tasks for Teaching
_record_task = None
_replay_task = None

# Global transport instances guarded by a lock for concurrency
transport = None
transport_lock = asyncio.Lock()
controllers = {}

# ---------------------------------------------------------------------------
# Background Tasks (Recording, Replaying)
# ---------------------------------------------------------------------------
async def teach_record_task():
    """Independent loop to record motor data at 10Hz without blocking queries."""
    global teaching_active, teaching_frames, teaching_start_time, teaching_last_record_time
    teaching_start_time = time.time()
    teaching_last_record_time = 0.0
    
    while teaching_active:
        now_t = time.time()
        if now_t - teaching_last_record_time >= TEACHING_RECORD_INTERVAL:
            teaching_last_record_time = now_t
            frame = {
                't': round(now_t - teaching_start_time, 3),
                'joints': {}
            }
            # Snapshot the latest data
            for jid in JOINT_IDS:
                d = motor_data[jid]
                if d['online']:
                    frame['joints'][str(jid)] = {
                        'pos': d['position'],
                        'vel': d['velocity'],
                        'torq': d['torque'],
                        'cur': d['q_current'],
                    }
            teaching_frames.append(frame)
        await asyncio.sleep(0.01)

async def teach_replay_task(replay_torque=6.0):
    """Independent loop to playback frames. Uses transport_lock to avoid CAN bus collisions."""
    global replaying_active
    try:
        with open(RECORDING_FILE, 'r') as f:
            frames = json.load(f)
        print(f'[CMD] Loaded {len(frames)} frames from {RECORDING_FILE} (torque={replay_torque})')
        replay_t0 = time.time()
        rec_t0 = frames[0].get('t', 0.0) if frames else 0.0
        
        for fi, frame in enumerate(frames):
            if not replaying_active:
                break
            
            # Build all joint commands for this frame
            msgs = []
            joints = frame.get('joints', {})
            frame_t = frame.get('t', 0.0)
            dbg_parts = []
            
            for jid_str, jdata in joints.items():
                jid_int = int(jid_str)
                ctrl = controllers.get(jid_int)
                if ctrl and jdata.get('pos') is not None:
                    pos = jdata['pos']
                    vel = jdata.get('vel', 0.0)
                    torq = jdata.get('torq', 0.0)
                    msgs.append(ctrl.make_position(
                        position=pos,
                        velocity=vel,
                        maximum_torque=replay_torque,
                        velocity_limit=0.1,
                        accel_limit=0.1,
                        query=True))
                    dbg_parts.append(f'J{jid_str}:pos={pos*360:.1f}° vel={vel:.3f} torq={torq:.3f}')
            print(f'[REPLAY] f={fi}/{len(frames)} t={frame_t:.2f}s {" | ".join(dbg_parts)}')
            
            # Send ALL joints in one CAN transaction safely
            if msgs:
                async with transport_lock:
                    try:
                        await asyncio.wait_for(
                            transport.cycle(msgs),
                            timeout=QUERY_TIMEOUT_S)
                    except asyncio.TimeoutError:
                        pass
                        
            # Sleep based on recorded timestamp delta
            if fi + 1 < len(frames):
                next_t = frames[fi + 1].get('t', 0.0)
                target_wall = replay_t0 + (next_t - rec_t0)
                sleep_s = target_wall - time.time()
                if sleep_s > 0:
                    await asyncio.sleep(sleep_s)
                else:
                    print(f'[REPLAY] ⚠ behind schedule by {-sleep_s:.3f}s')
                    
        replaying_active = False
        
        # Hold position with zero velocity (don't disable motors)
        async with transport_lock:
            for jid_s, ctrl_s in controllers.items():
                try:
                    await asyncio.wait_for(
                        transport.cycle([ctrl_s.make_position(
                            position=math.nan,
                            velocity=0.0,
                            maximum_torque=replay_torque,
                            query=True)]),
                        timeout=QUERY_TIMEOUT_S)
                except asyncio.TimeoutError:
                    pass
        print('[CMD] Replay finished, motors holding position')
        
    except FileNotFoundError:
        print(f'[ERROR] {RECORDING_FILE} not found')
        replaying_active = False
    except asyncio.CancelledError:
        print(f'[CMD] Replay task cancelled mid-playback.')
        replaying_active = False
    except Exception as e:
        print(f'[ERROR] Replay failed: {e}')
        replaying_active = False


# ---------------------------------------------------------------------------
# CAN background task
# ---------------------------------------------------------------------------
async def _read_pid_inner(jid, ctrl):
    """Inner PID read logic (called with timeout wrapper)."""
    global pid_data
    s = moteus.Stream(ctrl)
    await s.write_message(b'tel stop')
    await s.flush_read()
    for param in PID_PARAMS:
        key = f'servo.pid_position.{param}'
        raw = await s.command(
            f'conf get {key}'.encode('utf8'),
            allow_any_response=True)
        pid_data[jid][param] = float(raw.decode('utf8').strip())
    print(f'[PID] Motor {jid}: {pid_data[jid]}')

async def read_pid_from_motor(jid, ctrl):
    """Read PID parameters from a single motor with 1s timeout."""
    try:
        await asyncio.wait_for(_read_pid_inner(jid, ctrl), timeout=1.0)
    except asyncio.TimeoutError:
        print(f'[PID] Motor {jid}: timeout (1s), skipping')
    except Exception as e:
        print(f'[PID] Failed to read PID from motor {jid}: {e}')

async def write_pid_to_motor(jid, values):
    """Write PID parameters to a single motor using diagnostic protocol and save to flash."""
    ctrl = controllers.get(jid)
    if not ctrl:
        return False, f'Controller {jid} not found'
    try:
        s = moteus.Stream(ctrl)
        await s.write_message(b'tel stop')
        await s.flush_read()
        for param, val in values.items():
            key = f'servo.pid_position.{param}'
            await s.command(
                f'conf set {key} {val}'.encode('utf8'))
        await s.command(b'conf write')
        print(f'[PID] Motor {jid} updated and saved to flash: {values}')
        return True, 'OK'
    except Exception as e:
        print(f'[PID] Failed to write PID to motor {jid}: {e}')
        return False, str(e)

async def set_home_offset_motor(jid):
    """Set the dual-encoder home position to current physical position."""
    ctrl = controllers.get(jid)
    if not ctrl:
        return False, f'Controller {jid} not found'
    try:
        s = moteus.Stream(ctrl)
        await s.write_message(b'tel stop')
        await s.flush_read()
        await s.command(b'd cfg-set-output 0')
        await s.command(b'conf write')
        await s.command(b'd rezero 0')
        print(f'[SetHome] Motor {jid} zeroed.')
        return True, 'Home configured securely.'
    except Exception as e:
        print(f'[SetHome] Failed to zero motor {jid}: {e}')
        return False, str(e)

async def can_query_loop():
    """Background coroutine that queries all 6 moteus controllers individually."""
    global can_connected, can_error_msg, transport, controllers
    global pid_read_done
    
    try:
        transport = moteus.get_singleton_transport(_args)
        print('[INFO] CAN transport initialized successfully')
    except Exception as e:
        can_error_msg = f'CAN init failed: {e}'
        print(f'[ERROR] {can_error_msg}')
        return

    # Enable q_current and motor_temperature in query resolution
    qr = moteus.QueryResolution()
    qr.q_current = moteus.INT16
    qr.motor_temperature = moteus.INT16

    controllers = {
        jid: moteus.Controller(id=jid, transport=transport,
                               query_resolution=qr)
        for jid in JOINT_IDS
    }

    # Send initial stop to clear any latched faults (no query, so no reply expected)
    async with transport_lock:
        for jid, ctrl in controllers.items():
            try:
                await transport.cycle([ctrl.make_stop()])
            except Exception as e:
                print(f'[WARN] Stop motor {jid} failed: {e}')

    # Read PID parameters from each motor at startup
    print('[INFO] Reading PID parameters from motors...')
    async with transport_lock:
        for jid, ctrl in controllers.items():
            await read_pid_from_motor(jid, ctrl)
    pid_read_done = True
    print('[INFO] PID parameter read complete')

    print('[INFO] Starting query loop...')
    any_motor_seen = False

    while True:
        # --- Process pending commands ---
        while command_queue:
            cmd = command_queue.popleft()
            try:
                if cmd[0] == 'stop':
                    target = cmd[1] if len(cmd) > 1 else 'all'
                    print(f'[CMD] Stopping motors (target={target})')
                    async with transport_lock:
                        for jid, ctrl in controllers.items():
                            if target != 'all' and jid != target:
                                continue
                            try:
                                await asyncio.wait_for(
                                    transport.cycle([ctrl.make_stop()]),
                                    timeout=QUERY_TIMEOUT_S)
                            except asyncio.TimeoutError:
                                pass
                elif cmd[0] == 'position':
                    if len(cmd) == 5:
                        target = 'all'
                        pos, torque, vel, accel = cmd[1], cmd[2], cmd[3], cmd[4]
                    else:
                        target = cmd[1]
                        pos, torque, vel, accel = cmd[2], cmd[3], cmd[4], cmd[5]
                        
                    print(f'[CMD] Position (target={target}): pos={pos} torque={torque} vel={vel} accel={accel}')
                    async with transport_lock:
                        for jid, ctrl in controllers.items():
                            if target != 'all' and jid != target:
                                continue
                            try:
                                await asyncio.wait_for(
                                    transport.cycle([
                                        ctrl.make_position(
                                            position=pos,
                                            velocity=0.0,
                                            maximum_torque=torque,
                                            velocity_limit=vel,
                                            accel_limit=accel,
                                            query=True)
                                    ]),
                                    timeout=QUERY_TIMEOUT_S)
                            except asyncio.TimeoutError:
                                pass
                elif cmd[0] == 'teach_start_cancel':
                    # Stop motors when teaching starts
                    async with transport_lock:
                        for jid, ctrl in controllers.items():
                            try:
                                await asyncio.wait_for(
                                    transport.cycle([ctrl.make_stop()]),
                                    timeout=QUERY_TIMEOUT_S)
                            except asyncio.TimeoutError:
                                pass
                elif cmd[0] == 'teach_stop_cancel':
                    # Signal during replay cancellation
                    async with transport_lock:
                        for jid, ctrl in controllers.items():
                            try:
                                await asyncio.wait_for(
                                    transport.cycle([ctrl.make_position(
                                            position=math.nan,
                                            velocity=0.0,
                                            maximum_torque=2.0,
                                            query=True)]),
                                    timeout=QUERY_TIMEOUT_S)
                            except asyncio.TimeoutError:
                                pass
                elif cmd[0] == 'pid_apply':
                    jid_apply = cmd[1]
                    pid_vals = cmd[2]
                    result_future = cmd[3]
                    async with transport_lock:
                        ok, msg = await write_pid_to_motor(jid_apply, pid_vals)
                    result_future.set_result((ok, msg))
                elif cmd[0] == 'pid_read':
                    jid_read = cmd[1]
                    result_future = cmd[2]
                    ctrl_r = controllers.get(jid_read)
                    if ctrl_r:
                        async with transport_lock:
                            await read_pid_from_motor(jid_read, ctrl_r)
                        result_future.set_result((True, 'OK'))
                    else:
                        result_future.set_result((False, f'Controller {jid_read} not found'))
                elif cmd[0] == 'set_home':
                    jid_home = cmd[1]
                    result_future = cmd[2]
                    async with transport_lock:
                        ok, msg = await set_home_offset_motor(jid_home)
                    result_future.set_result((ok, msg))
            except Exception as e:
                print(f'[ERROR] Command {cmd}: {e}')

        # --- Query all motors ---
        got_any = False

        for jid, ctrl in controllers.items():
            try:
                # Query one motor at a time with a timeout, using the lock
                async with transport_lock:
                    results = await asyncio.wait_for(
                        transport.cycle([ctrl.make_query()]),
                        timeout=QUERY_TIMEOUT_S)

                for result in results:
                    servo_id = result.id
                    if servo_id in motor_data:
                        vals = result.values
                        motor_data[servo_id]['position'] = vals.get(
                            Register.POSITION, None)
                        motor_data[servo_id]['velocity'] = vals.get(
                            Register.VELOCITY, None)
                        motor_data[servo_id]['torque'] = vals.get(
                            Register.TORQUE, None)
                        motor_data[servo_id]['q_current'] = vals.get(
                            Register.Q_CURRENT, None)
                        motor_data[servo_id]['voltage'] = vals.get(
                            Register.VOLTAGE, None)
                        motor_data[servo_id]['temperature'] = vals.get(
                            Register.TEMPERATURE, None)
                        motor_data[servo_id]['motor_temperature'] = vals.get(
                            Register.MOTOR_TEMPERATURE, None)
                        motor_data[servo_id]['mode'] = vals.get(
                            Register.MODE, None)
                        motor_data[servo_id]['fault'] = vals.get(
                            Register.FAULT, None)
                        motor_data[servo_id]['last_update'] = time.time()
                        motor_data[servo_id]['online'] = True
                        got_any = True

                        # Record history
                        if servo_id in history_data:
                            t = time.time()
                            h = history_data[servo_id]
                            for key, reg in [('pos', Register.POSITION), 
                                             ('vel', Register.VELOCITY), 
                                             ('torq', Register.TORQUE), 
                                             ('cur', Register.Q_CURRENT)]:
                                v = vals.get(reg, None)
                                if v is not None:
                                    h[key]['times'].append(t)
                                    h[key]['values'].append(v)

                        if not any_motor_seen:
                            print(f'[INFO] Motor {servo_id} responded: '
                                  f'pos={vals.get(Register.POSITION)}, '
                                  f'vel={vals.get(Register.VELOCITY)}, '
                                  f'torq={vals.get(Register.TORQUE)}, '
                                  f'cur={vals.get(Register.Q_CURRENT)}, '
                                  f'volt={vals.get(Register.VOLTAGE)}, '
                                  f'temp={vals.get(Register.TEMPERATURE)}')

            except asyncio.TimeoutError:
                motor_data[jid]['online'] = False
            except Exception as e:
                motor_data[jid]['online'] = False
                print(f'[WARN] Query motor {jid}: {e}')

        if got_any:
            can_connected = True
            can_error_msg = ''
            any_motor_seen = True
        else:
            can_connected = False
            if not any_motor_seen:
                can_error_msg = 'No motors responding'

        await asyncio.sleep(QUERY_INTERVAL_S)


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------
def fmt_pos(val):
    if val is None:
        return 'N/A'
    return f'{val * 360:.1f}°'

def fmt_volt(val):
    if val is None:
        return 'N/A'
    return f'{val:.1f} V'

def fmt_temp(val):
    if val is None:
        return 'N/A'
    return f'{val:.1f} \u00b0C'

def fmt_vel(val):
    if val is None:
        return 'N/A'
    return f'{val:.3f}'

def fmt_torque(val):
    if val is None:
        return 'N/A'
    return f'{val:.3f} Nm'

def fmt_current(val):
    if val is None:
        return 'N/A'
    return f'{val:.3f} A'

# Moteus fault code names (from fw/error.h)
FAULT_NAMES = {
    0: 'OK',
    1: 'DMA Stream Transfer',
    2: 'DMA Stream FIFO',
    3: 'UART Overrun',
    4: 'UART Framing',
    5: 'UART Noise',
    6: 'UART Buffer Overrun',
    7: 'UART Parity',
    32: 'Calibration Fault',
    33: 'Motor Driver Fault',
    34: 'Over Voltage',
    35: 'Encoder Fault',
    36: 'Motor Not Configured',
    37: 'PWM Cycle Overrun',
    38: 'Over Temperature',
    39: 'Start Outside Limit',
    40: 'Under Voltage',
    41: 'Config Changed',
    42: 'Theta Invalid',
    43: 'Position Invalid',
    44: 'Driver Enable Fault',
    45: 'Stop Position (deprecated)',
    46: 'Timing Violation',
    47: 'BEMF FF No Accel Limit',
    48: 'Invalid Limits',
    96: 'Limit: Max Velocity',
    97: 'Limit: Max Power',
    98: 'Limit: Max Voltage',
    99: 'Limit: Max Current',
    100: 'Limit: FET Temperature',
    101: 'Limit: Motor Temperature',
    102: 'Limit: Max Torque',
    103: 'Limit: Position Bounds',
    104: 'Limit: Flux Braking',
}

MODE_NAMES = {
    0: 'Stopped', 1: 'Fault', 5: 'PWM', 6: 'Voltage',
    7: 'Voltage FOC', 8: 'Voltage DQ', 9: 'Current',
    10: 'Position', 11: 'Timeout', 12: 'Zero Velocity',
    13: 'Stay Within', 14: 'Measure Ind', 15: 'Brake',
}

def fmt_fault(mode_val, fault_val):
    if mode_val is None and fault_val is None:
        return 'N/A', False
    mode_name = MODE_NAMES.get(mode_val, f'Unknown({mode_val})')
    fault_code = int(fault_val) if fault_val is not None else 0
    is_error = (mode_val == 1)  # Mode.FAULT
    is_limit = (96 <= fault_code <= 104)
    if fault_code == 0:
        return f'{mode_name}', False
    fault_name = FAULT_NAMES.get(fault_code, f'Unknown({fault_code})')
    if is_error:
        return f'FAULT: {fault_name}', True
    elif is_limit:
        return f'{mode_name} [{fault_name}]', False
    else:
        return f'{mode_name} (err:{fault_name})', True


# ---------------------------------------------------------------------------
# CSS styles
# ---------------------------------------------------------------------------
CUSTOM_CSS = """
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap');

:root {
    --bg-primary: #0f0f1a;
    --bg-card: #1a1a2e;
    --bg-card-hover: #1e1e35;
    --accent-blue: #4361ee;
    --accent-cyan: #4cc9f0;
    --accent-green: #06d6a0;
    --accent-red: #ef476f;
    --accent-amber: #ffd166;
    --text-primary: #e8e8f0;
    --text-secondary: #a0a0b8;
    --text-dim: #6b6b80;
    --border-color: #2a2a40;
    --shadow-card: 0 8px 32px rgba(0, 0, 0, 0.4);
}

body {
    font-family: 'Inter', sans-serif !important;
    background: var(--bg-primary) !important;
    color: var(--text-primary) !important;
}

.q-page {
    background: var(--bg-primary) !important;
}

.dashboard-header {
    background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
    border-bottom: 1px solid var(--border-color);
    padding: 20px 32px;
    display: flex;
    align-items: center;
    justify-content: space-between;
}

.header-title {
    font-size: 1.6rem;
    font-weight: 700;
    background: linear-gradient(90deg, var(--accent-blue), var(--accent-cyan));
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    background-clip: text;
    letter-spacing: -0.02em;
}

.header-subtitle {
    font-size: 0.85rem;
    color: var(--text-dim);
    margin-top: 2px;
}

.status-badge {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    padding: 6px 16px;
    border-radius: 20px;
    font-size: 0.8rem;
    font-weight: 500;
}

.status-connected {
    background: rgba(6, 214, 160, 0.12);
    color: var(--accent-green);
    border: 1px solid rgba(6, 214, 160, 0.25);
}

.status-disconnected {
    background: rgba(239, 71, 111, 0.12);
    color: var(--accent-red);
    border: 1px solid rgba(239, 71, 111, 0.25);
}

.status-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    display: inline-block;
    animation: pulse-glow 2s infinite;
}

.status-dot-green {
    background: var(--accent-green);
    box-shadow: 0 0 8px var(--accent-green);
}

.status-dot-red {
    background: var(--accent-red);
    box-shadow: 0 0 8px var(--accent-red);
}

@keyframes pulse-glow {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.5; }
}

.cards-grid {
    display: grid;
    grid-template-columns: repeat(6, 1fr);
    gap: 16px;
    padding: 28px 24px;
    overflow-x: auto;
}

.motor-card {
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 16px;
    padding: 24px;
    box-shadow: var(--shadow-card);
    transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
    position: relative;
    overflow: hidden;
}

.motor-card::before {
    content: '';
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    height: 3px;
    background: linear-gradient(90deg, var(--accent-blue), var(--accent-cyan));
    opacity: 0.7;
}

.motor-card:hover {
    background: var(--bg-card-hover);
    border-color: rgba(67, 97, 238, 0.3);
    transform: translateY(-2px);
    box-shadow: 0 12px 40px rgba(0, 0, 0, 0.5);
}

.card-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 20px;
}

.joint-label {
    font-size: 1.2rem;
    font-weight: 600;
    color: var(--text-primary);
}

.joint-id-badge {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    width: 36px;
    height: 36px;
    border-radius: 10px;
    background: linear-gradient(135deg, var(--accent-blue), var(--accent-cyan));
    color: #fff;
    font-weight: 700;
    font-size: 0.95rem;
    box-shadow: 0 4px 12px rgba(67, 97, 238, 0.3);
}

.data-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 0;
    border-bottom: 1px solid rgba(42, 42, 64, 0.6);
}

.data-row:last-child {
    border-bottom: none;
}

.data-label {
    display: flex;
    align-items: center;
    gap: 10px;
    font-size: 0.85rem;
    color: var(--text-secondary);
    font-weight: 400;
}

.data-icon {
    width: 32px;
    height: 32px;
    border-radius: 8px;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 1rem;
}

.icon-position {
    background: rgba(67, 97, 238, 0.15);
    color: var(--accent-blue);
}

.icon-voltage {
    background: rgba(255, 209, 102, 0.15);
    color: var(--accent-amber);
}

.icon-temp {
    background: rgba(239, 71, 111, 0.15);
    color: var(--accent-red);
}

.data-value {
    font-size: 1.05rem;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
    letter-spacing: 0.01em;
}

.value-position { color: var(--accent-cyan); }
.value-velocity { color: #a78bfa; }
.value-torque { color: #f97316; }
.value-current { color: #22d3ee; }
.value-voltage { color: var(--accent-amber); }
.value-temp { color: #ff8fa3; }
.value-motor-temp { color: #fb923c; }
.value-na { color: var(--text-dim); }

.icon-velocity {
    background: rgba(167, 139, 250, 0.15);
    color: #a78bfa;
}

.icon-torque {
    background: rgba(249, 115, 22, 0.15);
    color: #f97316;
}

.icon-current {
    background: rgba(34, 211, 238, 0.15);
    color: #22d3ee;
}

.icon-motor-temp {
    background: rgba(251, 146, 60, 0.15);
    color: #fb923c;
}

.icon-status {
    background: rgba(6, 214, 160, 0.15);
    color: var(--accent-green);
}

.icon-status-err {
    background: rgba(239, 71, 111, 0.15);
    color: var(--accent-red);
}

.value-status-ok { color: var(--accent-green); }
.value-status-err {
    color: var(--accent-red);
    font-weight: 700;
}

.card-footer {
    margin-top: 14px;
    padding-top: 10px;
    border-top: 1px solid var(--border-color);
    font-size: 0.72rem;
    color: var(--text-dim);
    text-align: right;
}

.error-bar {
    background: rgba(239, 71, 111, 0.1);
    border: 1px solid rgba(239, 71, 111, 0.25);
    border-radius: 10px;
    padding: 12px 20px;
    margin: 16px 32px 0;
    color: var(--accent-red);
    font-size: 0.85rem;
}

.nicegui-content {
    padding: 0 !important;
}

.control-panel {
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 16px;
    padding: 24px 32px;
    margin: 0 24px 28px;
    box-shadow: var(--shadow-card);
}

.control-panel-title {
    font-size: 1.1rem;
    font-weight: 600;
    color: var(--text-primary);
    margin-bottom: 16px;
}

.control-row {
    display: flex;
    align-items: center;
    gap: 12px;
    flex-wrap: wrap;
}

.control-divider {
    width: 1px;
    height: 36px;
    background: var(--border-color);
    margin: 0 8px;
}

.ctrl-btn {
    padding: 8px 20px;
    border: none;
    border-radius: 10px;
    font-family: 'Inter', sans-serif;
    font-size: 0.85rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.2s;
    outline: none;
}

.ctrl-btn:hover {
    transform: translateY(-1px);
    box-shadow: 0 4px 12px rgba(0,0,0,0.3);
}

.ctrl-btn:active {
    transform: translateY(0);
}

.btn-enable {
    background: linear-gradient(135deg, #06d6a0, #0ead83);
    color: #fff;
}

.btn-disable {
    background: linear-gradient(135deg, #ef476f, #d63a5c);
    color: #fff;
}

.btn-plus {
    background: linear-gradient(135deg, #4361ee, #3651d4);
    color: #fff;
}

.btn-home {
    background: linear-gradient(135deg, #ffd166, #e6b84d);
    color: #1a1a2e;
}

.btn-minus {
    background: linear-gradient(135deg, #a78bfa, #8b6fe0);
    color: #fff;
}

.pos-input {
    width: 100px;
    padding: 8px 12px;
    border-radius: 10px;
    border: 1px solid var(--border-color);
    background: var(--bg-primary);
    color: var(--text-primary);
    font-family: 'Inter', sans-serif;
    font-size: 0.9rem;
    font-weight: 500;
    text-align: center;
    outline: none;
    transition: border-color 0.2s;
}

.pos-input:focus {
    border-color: var(--accent-blue);
}

.control-label {
    font-size: 0.8rem;
    color: var(--text-dim);
    font-weight: 500;
}

.pid-toggle-btn {
    width: 100%;
    padding: 8px 16px;
    margin-top: 12px;
    border: 1px solid var(--border-color);
    border-radius: 10px;
    background: transparent;
    color: var(--text-secondary);
    font-family: 'Inter', sans-serif;
    font-size: 0.78rem;
    font-weight: 500;
    cursor: pointer;
    transition: all 0.25s ease;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
}

.pid-toggle-btn:hover {
    background: rgba(67, 97, 238, 0.08);
    border-color: rgba(67, 97, 238, 0.3);
    color: var(--accent-cyan);
}

.pid-panel {
    margin-top: 12px;
    padding: 14px;
    background: rgba(15, 15, 26, 0.6);
    border: 1px solid var(--border-color);
    border-radius: 12px;
}

.pid-panel-title {
    font-size: 0.8rem;
    font-weight: 600;
    color: var(--accent-cyan);
    margin-bottom: 10px;
    letter-spacing: 0.03em;
}

.pid-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 5px 0;
}

.pid-label {
    font-size: 0.78rem;
    color: var(--text-secondary);
    font-weight: 500;
    min-width: 100px;
}

.pid-input {
    width: 100px;
    padding: 5px 8px;
    border-radius: 8px;
    border: 1px solid var(--border-color);
    background: var(--bg-primary);
    color: var(--text-primary);
    font-family: 'Inter', sans-serif;
    font-size: 0.82rem;
    font-weight: 500;
    text-align: right;
    outline: none;
    transition: border-color 0.2s;
}

.pid-input:focus {
    border-color: var(--accent-cyan);
    box-shadow: 0 0 0 2px rgba(76, 201, 240, 0.1);
}

.pid-apply-btn {
    width: 100%;
    margin-top: 10px;
    padding: 8px 16px;
    border: none;
    border-radius: 10px;
    background: linear-gradient(135deg, #4361ee, #4cc9f0);
    color: #ffd166;
    font-family: 'Inter', sans-serif;
    font-size: 0.82rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.25s ease;
    letter-spacing: 0.02em;
}

.pid-apply-btn:hover {
    transform: translateY(-1px);
    box-shadow: 0 4px 16px rgba(67, 97, 238, 0.35);
}

.pid-apply-btn:active {
    transform: translateY(0);
}

.pid-status {
    font-size: 0.72rem;
    color: var(--text-dim);
    text-align: center;
    margin-top: 6px;
    min-height: 16px;
    transition: color 0.3s;
}

.pid-status-ok { color: var(--accent-green); }
.pid-status-err { color: var(--accent-red); }
"""

# ---------------------------------------------------------------------------
# UI builder
# ---------------------------------------------------------------------------
# Store label references for updates
ui_labels = {}


def build_page():
    """Construct the dashboard page."""
    ui.add_css(CUSTOM_CSS)

    # ---- Header ----
    with ui.element('div').classes('dashboard-header'):
        with ui.element('div'):
            ui.html('<div class="header-title">DummyX2 Dashboard</div>')
            ui.html('<div class="header-subtitle">Robot Real-Time Monitor &middot; CANFD &middot; BRS Disabled</div>')
        # Status badge - updated dynamically
        with ui.element('div').classes('status-badge status-disconnected') as status_badge:
            status_dot = ui.html('<span class="status-dot status-dot-red"></span>')
            status_text = ui.label('Disconnected').style(
                'font-size: 0.8rem; font-weight: 500; margin: 0; padding: 0;')

        ui_labels['status_badge'] = status_badge
        ui_labels['status_dot'] = status_dot
        ui_labels['status_text'] = status_text

    # ---- Error bar (hidden by default) ----
    error_bar = ui.label('').classes('error-bar')
    error_bar.set_visibility(False)
    ui_labels['error_bar'] = error_bar

    # ---- Motor cards grid ----
    with ui.element('div').classes('cards-grid'):
        for jid in JOINT_IDS:
            with ui.element('div').classes('motor-card'):
                # Card header
                with ui.element('div').classes('card-header'):
                    ui.html(f'<span class="joint-label">\u2699\ufe0f</span>')
                    ui.html(f'<span class="joint-id-badge">{jid}</span>')

                # Status row
                with ui.element('div').classes('data-row'):
                    with ui.element('div').classes('data-label'):
                        status_icon = ui.html('<div class="data-icon icon-status">\u2713</div>')
                        ui.label('Status').style('margin: 0; padding: 0;')
                        ui_labels[f'status_icon_{jid}'] = status_icon
                    status_label = ui.label('N/A').classes('data-value value-na')
                    status_label.style('margin: 0; padding: 0;')
                    ui_labels[f'status_{jid}'] = status_label

                # Position row (clickable for chart)
                with ui.element('div').classes('data-row').style('cursor: pointer;') as pos_row:
                    with ui.element('div').classes('data-label'):
                        ui.html('<div class="data-icon icon-position">\u27f3</div>')
                        ui.label('Position (°)').style('margin: 0; padding: 0;')
                    pos_label = ui.label('N/A').classes('data-value value-na')
                    pos_label.style('margin: 0; padding: 0;')
                    ui_labels[f'pos_{jid}'] = pos_label
                    ui.html('<span style="font-size:0.7rem; color: var(--text-dim); margin-left: 4px;">\u2197</span>')
                # Bind click event - capture jid in closure
                _jid = jid
                pos_row.on('click', lambda e, j=_jid: show_chart(j, 'pos', 'Position', 'rev', '#4cc9f0'))

                # Velocity row (clickable for chart)
                with ui.element('div').classes('data-row').style('cursor: pointer;') as vel_row:
                    with ui.element('div').classes('data-label'):
                        ui.html('<div class="data-icon icon-velocity">\u2192</div>')
                        ui.label('Velocity (rev/s)').style('margin: 0; padding: 0;')
                    vel_label = ui.label('N/A').classes('data-value value-na')
                    vel_label.style('margin: 0; padding: 0;')
                    ui_labels[f'vel_{jid}'] = vel_label
                    ui.html('<span style="font-size:0.7rem; color: var(--text-dim); margin-left: 4px;">\u2197</span>')
                vel_row.on('click', lambda e, j=_jid: show_chart(j, 'vel', 'Velocity', 'rev/s', '#a78bfa'))

                # Torque row (clickable for chart)
                with ui.element('div').classes('data-row').style('cursor: pointer;') as torq_row:
                    with ui.element('div').classes('data-label'):
                        ui.html('<div class="data-icon icon-torque">\u21bb</div>')
                        ui.label('Torque (Nm)').style('margin: 0; padding: 0;')
                    torq_label = ui.label('N/A').classes('data-value value-na')
                    torq_label.style('margin: 0; padding: 0;')
                    ui_labels[f'torq_{jid}'] = torq_label
                    ui.html('<span style="font-size:0.7rem; color: var(--text-dim); margin-left: 4px;">\u2197</span>')
                torq_row.on('click', lambda e, j=_jid: show_chart(j, 'torq', 'Torque', 'Nm', '#ef476f'))

                # Current row (clickable for chart)
                with ui.element('div').classes('data-row').style('cursor: pointer;') as cur_row:
                    with ui.element('div').classes('data-label'):
                        ui.html('<div class="data-icon icon-current">\u2301</div>')
                        ui.label('Current (A)').style('margin: 0; padding: 0;')
                    cur_label = ui.label('N/A').classes('data-value value-na')
                    cur_label.style('margin: 0; padding: 0;')
                    ui_labels[f'cur_{jid}'] = cur_label
                    ui.html('<span style="font-size:0.7rem; color: var(--text-dim); margin-left: 4px;">\u2197</span>')
                cur_row.on('click', lambda e, j=_jid: show_chart(j, 'cur', 'Current', 'A', '#22d3ee'))

                # Voltage row
                with ui.element('div').classes('data-row'):
                    with ui.element('div').classes('data-label'):
                        ui.html('<div class="data-icon icon-voltage">\u26a1</div>')
                        ui.label('Voltage').style('margin: 0; padding: 0;')
                    volt_label = ui.label('N/A').classes('data-value value-na')
                    volt_label.style('margin: 0; padding: 0;')
                    ui_labels[f'volt_{jid}'] = volt_label

                # MOSFET Temperature row
                with ui.element('div').classes('data-row'):
                    with ui.element('div').classes('data-label'):
                        ui.html('<div class="data-icon icon-temp">\U0001f321</div>')
                        ui.label('MOSFET Temp').style('margin: 0; padding: 0;')
                    temp_label = ui.label('N/A').classes('data-value value-na')
                    temp_label.style('margin: 0; padding: 0;')
                    ui_labels[f'temp_{jid}'] = temp_label

                # Motor Temperature row
                with ui.element('div').classes('data-row'):
                    with ui.element('div').classes('data-label'):
                        ui.html('<div class="data-icon icon-motor-temp">\U0001f525</div>')
                        ui.label('Motor Temp').style('margin: 0; padding: 0;')
                    mtemp_label = ui.label('N/A').classes('data-value value-na')
                    mtemp_label.style('margin: 0; padding: 0;')
                    ui_labels[f'mtemp_{jid}'] = mtemp_label

                # Card footer - last update time
                footer = ui.label('Waiting for data...').classes('card-footer')
                footer.style('margin: 0; padding: 0;')
                ui_labels[f'footer_{jid}'] = footer

                # ---- PID Tuning Collapsible Section ----
                pid_panel = ui.element('div').classes('pid-panel')
                pid_panel.set_visibility(False)
                ui_labels[f'pid_panel_{jid}'] = pid_panel

                def _make_pid_toggle(j, panel):
                    def toggle():
                        vis = not panel.visible
                        panel.set_visibility(vis)
                    return toggle

                ui.button('\u2699 PID Tuning', on_click=_make_pid_toggle(jid, pid_panel)).props(
                    'flat dense no-caps').classes('pid-toggle-btn')

                with pid_panel:
                    ui.html('<div class="pid-panel-title">\u2699 Position PID Parameters</div>')
                    for param in PID_PARAMS:
                        with ui.element('div').classes('pid-row'):
                            ui.label(param).classes('pid-label')
                            inp = ui.number(
                                value=0.0, format='%.6f', step=0.1
                            ).style(
                                'width: 110px; border-radius: 8px; font-size: 0.82rem;'
                            )
                            ui_labels[f'pid_{param}_{jid}'] = inp

                    pid_status = ui.label('').classes('pid-status')
                    ui_labels[f'pid_status_{jid}'] = pid_status

                    def _make_set_home(j):
                        async def set_home_offset():
                            st = ui_labels.get(f'pid_status_{j}')
                            if st:
                                st.set_text('Setting Home Offset...')
                                st._classes = ['pid-status']
                                st.update()
                            fut = asyncio.get_event_loop().create_future()
                            command_queue.append(('set_home', j, fut))
                            ok, msg = await fut
                            if st:
                                if ok:
                                    st.set_text('\u2713 Home Offset Applied')
                                    st._classes = ['pid-status', 'pid-status-ok']
                                else:
                                    st.set_text(f'\u2717 Error: {msg}')
                                    st._classes = ['pid-status', 'pid-status-err']
                                st.update()
                        return set_home_offset

                    def _make_apply(j):
                        async def apply_pid():
                            st = ui_labels.get(f'pid_status_{j}')
                            vals = {}
                            for p in PID_PARAMS:
                                inp = ui_labels.get(f'pid_{p}_{j}')
                                if inp and inp.value is not None:
                                    vals[p] = float(inp.value)
                            if st:
                                st.set_text('Applying...')
                                st._classes = ['pid-status']
                                st.update()
                            fut = asyncio.get_event_loop().create_future()
                            command_queue.append(('pid_apply', j, vals, fut))
                            ok, msg = await fut
                            if st:
                                if ok:
                                    st.set_text('\u2713 Applied successfully')
                                    st._classes = ['pid-status', 'pid-status-ok']
                                else:
                                    st.set_text(f'\u2717 Error: {msg}')
                                    st._classes = ['pid-status', 'pid-status-err']
                                st.update()
                            # Re-read to confirm
                            fut2 = asyncio.get_event_loop().create_future()
                            command_queue.append(('pid_read', j, fut2))
                            await fut2
                            _refresh_pid_inputs(j)
                        return apply_pid

                    with ui.element('div').style('display: flex; gap: 10px; margin-top: 5px;'):
                        ui.button('\u2714 Apply PID', on_click=_make_apply(jid)).props(
                            'flat dense no-caps').classes('pid-apply-btn').style(
                            'color: #ffd166 !important; flex: 1;')
                        
                        ui.button('\U0001f3e0 Set Home', on_click=_make_set_home(jid)).props(
                            'flat dense no-caps').classes('pid-apply-btn').style(
                            'color: #06d6a0 !important; flex: 1; font-weight: bold;')

    # ---- Control Panel ----
    with ui.element('div').classes('control-panel'):
        ui.html('<div class="control-panel-title">\u2699 Control</div>')
        with ui.element('div').classes('control-row'):
            # Target
            ui.html('<span class="control-label">Target:</span>')
            target_dropdown = ui.select(['all'] + JOINT_IDS, value='all').style(
                'width: 60px; font-weight: 500;')
            ui_labels['target_dropdown'] = target_dropdown

            # Enable / Disable
            ui.html('<span class="control-label">Mode:</span>')
            ui.button('Enable', on_click=lambda: on_enable()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 600; cursor: pointer; '
                'background: linear-gradient(135deg, #06d6a0, #0ead83); color: #fff;')
            ui.button('Disable', on_click=lambda: on_disable()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 600; cursor: pointer; '
                'background: linear-gradient(135deg, #ef476f, #d63a5c); color: #fff;')

            # Divider
            ui.element('div').classes('control-divider')

            # Position controls
            ui.html('<span class="control-label">Pos (°):</span>')
            pos_input = ui.number(value=0.0, step=10, format='%.1f').style(
                'width: 90px; border-radius: 10px; font-weight: 500; text-align: center;')
            ui_labels['pos_input'] = pos_input

            ui.html('<span class="control-label">Torque:</span>')
            torque_input = ui.number(value=6.0, min=1.0, step=0.1, format='%.2f').style(
                'width: 80px; border-radius: 10px; font-weight: 500; text-align: center;')
            ui_labels['torque_input'] = torque_input

            ui.html('<span class="control-label">Vel:</span>')
            vel_input = ui.number(value=0.1, min=0.01, step=0.05, format='%.2f').style(
                'width: 80px; border-radius: 10px; font-weight: 500; text-align: center;')
            ui_labels['vel_input'] = vel_input

            ui.html('<span class="control-label">Accel:</span>')
            accel_input = ui.number(value=0.2, min=0.02, step=0.05, format='%.2f').style(
                'width: 80px; border-radius: 10px; font-weight: 500; text-align: center;')
            ui_labels['accel_input'] = accel_input

            # Divider
            ui.element('div').classes('control-divider')

            ui.button('\u2212', on_click=lambda: on_move_minus()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 700; font-size: 1.1rem; cursor: pointer; '
                'background: linear-gradient(135deg, #a78bfa, #8b6fe0); color: #fff;')
            ui.button('Home', on_click=lambda: on_home()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 600; cursor: pointer; '
                'background: linear-gradient(135deg, #ffd166, #e6b84d); color: #1a1a2e;')
            ui.button('+', on_click=lambda: on_move_plus()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 700; font-size: 1.1rem; cursor: pointer; '
                'background: linear-gradient(135deg, #4361ee, #3651d4); color: #fff;')

    # ---- Teaching Panel ----
    with ui.element('div').classes('control-panel'):
        ui.html('<div class="control-panel-title">\U0001F3AC Teaching</div>')
        with ui.element('div').classes('control-row'):
            ui.button('▶ START', on_click=lambda: on_teach_start()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 600; cursor: pointer; '
                'background: linear-gradient(135deg, #06d6a0, #0ead83); color: #fff;')
            ui.button('⏹ STOP', on_click=lambda: on_teach_stop()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 600; cursor: pointer; '
                'background: linear-gradient(135deg, #ef476f, #d63a5c); color: #fff;')
            ui.button('🔁 REPEAT', on_click=lambda: on_teach_repeat()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 600; cursor: pointer; '
                'background: linear-gradient(135deg, #4361ee, #3651d4); color: #fff;')

            ui.element('div').classes('control-divider')

            teach_status = ui.label('Idle').style(
                'font-size: 0.85rem; color: var(--text-secondary); font-weight: 500;')
            ui_labels['teach_status'] = teach_status


def _refresh_pid_inputs(jid):
    """Update PID input fields from pid_data for a given joint."""
    for param in PID_PARAMS:
        inp = ui_labels.get(f'pid_{param}_{jid}')
        if inp and pid_data[jid][param] is not None:
            inp.value = pid_data[jid][param]
            inp.update()

# ---------------------------------------------------------------------------
# Control callbacks
# ---------------------------------------------------------------------------
def _get_ctrl_params():
    """Read torque/vel/accel from UI inputs."""
    tgt = ui_labels.get('target_dropdown')
    t = ui_labels.get('torque_input')
    v = ui_labels.get('vel_input')
    a = ui_labels.get('accel_input')
    target = tgt.value if tgt and tgt.value is not None else 'all'
    torque = float(t.value) if t and t.value is not None else 1.0
    vel = float(v.value) if v and v.value is not None else 0.1
    accel = float(a.value) if a and a.value is not None else 0.1
    return target, torque, vel, accel

def on_enable():
    """Enter position mode (hold current position)."""
    target, torque, vel, accel = _get_ctrl_params()
    command_queue.append(('position', target, math.nan, torque, 0.0, accel))
    print(f'[UI] Enable clicked (target={target})')

def on_disable():
    """Stop all motors."""
    tgt = ui_labels.get('target_dropdown')
    target = tgt.value if tgt and tgt.value is not None else 'all'
    command_queue.append(('stop', target))
    print(f'[UI] Disable clicked (target={target})')

def on_move_plus():
    """Move all motors to absolute pos_input degrees (+ direction)."""
    pos_input = ui_labels.get('pos_input')
    target, torque, vel, accel = _get_ctrl_params()
    if pos_input and pos_input.value is not None:
        target_deg = float(pos_input.value)
        target_rev = target_deg / 360.0
        command_queue.append(('position', target, target_rev, torque, vel, accel))
        print(f'[UI] Move to {target_deg:.1f}° ({target_rev:.4f} rev) tgt={target} t={torque} v={vel} a={accel}')

def on_home():
    """Move all motors to position 0."""
    target, torque, vel, accel = _get_ctrl_params()
    command_queue.append(('position', target, 0.0, torque, vel, accel))
    print(f'[UI] Home clicked (target={target})')

def on_move_minus():
    """Move all motors to absolute -pos_input degrees (- direction)."""
    pos_input = ui_labels.get('pos_input')
    target, torque, vel, accel = _get_ctrl_params()
    if pos_input and pos_input.value is not None:
        target_deg = -float(pos_input.value)
        target_rev = target_deg / 360.0
        command_queue.append(('position', target, target_rev, torque, vel, accel))
        print(f'[UI] Move to {target_deg:.1f}° ({target_rev:.4f} rev) tgt={target} t={torque} v={vel} a={accel}')


def on_teach_start():
    """Start teaching: disable motors and begin recording in a bg task."""
    global teaching_active, teaching_frames, _record_task, _replay_task, replaying_active
    
    if _replay_task and not _replay_task.done():
        _replay_task.cancel()
        replaying_active = False
        
    print('[UI] Teaching START')
    teaching_active = True
    teaching_frames = []
    
    # Send stop to motors
    command_queue.append(('teach_start_cancel',))
    
    # Start the async record task
    if _record_task and not _record_task.done():
        _record_task.cancel()
    _record_task = asyncio.create_task(teach_record_task())

def on_teach_stop():
    """Stop teaching: stop recording and save."""
    global teaching_active, _record_task, _replay_task, replaying_active
    
    print('[UI] Teaching STOP')
    
    # If we are teaching
    if teaching_active:
        teaching_active = False
        if _record_task and not _record_task.done():
            _record_task.cancel()
            
        print(f'[CMD] Teaching STOP: {len(teaching_frames)} frames recorded')
        if teaching_frames:
            with open(RECORDING_FILE, 'w') as f:
                json.dump(teaching_frames, f, indent=1)
            print(f'[CMD] Saved to {RECORDING_FILE}')
            
    # If we are replaying
    if replaying_active:
        replaying_active = False
        if _replay_task and not _replay_task.done():
            _replay_task.cancel()
        # Immediately send command to stop the motors (via CAN loop)
        command_queue.append(('teach_stop_cancel',))


def on_teach_repeat():
    """Replay recorded trajectory."""
    global replaying_active, _replay_task, _record_task, teaching_active
    
    if _record_task and not _record_task.done():
        _record_task.cancel()
        teaching_active = False
        
    print('[UI] Teaching REPEAT')
    replaying_active = True
    
    # Read torque from Control panel
    _, torque, _, _ = _get_ctrl_params()
    
    # Stop any old replay task
    if _replay_task and not _replay_task.done():
        _replay_task.cancel()
        
    # Start the new async replay task
    _replay_task = asyncio.create_task(teach_replay_task(replay_torque=torque))


def show_chart(jid, key, title_prefix, unit, color):
    """Show a dialog with a real-time history chart for the given joint and metric."""
    with ui.dialog() as dlg, ui.card().style(
            'width: 700px; max-width: 95vw; background: #1a1a2e; '
            'border: 1px solid #2a2a40; border-radius: 16px; padding: 20px;'):
        ui.label(f'Joint {jid} - {title_prefix} Monitoring').style(
            'font-size: 1.1rem; font-weight: 600; color: #e8e8f0; margin-bottom: 8px;')

        chart = ui.echart({
            'backgroundColor': 'transparent',
            'animation': False,
            'grid': {'left': 60, 'right': 20, 'top': 30, 'bottom': 40},
            'xAxis': {
                'type': 'category',
                'data': [],
                'axisLabel': {'color': '#6b6b80', 'fontSize': 10},
                'axisLine': {'lineStyle': {'color': '#2a2a40'}},
                'splitLine': {'show': False},
            },
            'yAxis': {
                'type': 'value',
                'axisLabel': {'color': '#6b6b80', 'fontSize': 10,
                              'formatter': '{value}'},
                'axisLine': {'lineStyle': {'color': '#2a2a40'}},
                'splitLine': {'lineStyle': {'color': '#2a2a40', 'type': 'dashed'}},
                'name': unit, 'nameTextStyle': {'color': '#6b6b80'},
            },
            'series': [{
                'type': 'line',
                'data': [],
                'smooth': True,
                'showSymbol': False,
                'lineStyle': {'color': color, 'width': 2},
                'areaStyle': {
                    'color': {
                        'type': 'linear', 'x': 0, 'y': 0, 'x2': 0, 'y2': 1,
                        'colorStops': [
                            {'offset': 0, 'color': color},
                            {'offset': 1, 'color': 'rgba(0, 0, 0, 0)'},
                        ]
                    }
                },
            }],
            'tooltip': {
                'trigger': 'axis',
                'backgroundColor': '#1a1a2e',
                'borderColor': '#2a2a40',
                'textStyle': {'color': '#e8e8f0'},
            },
        }).style('height: 320px; width: 100%;')

        with ui.row().classes('w-full items-center justify-between mt-2'):
            time_select = ui.select(
                {5: '5s', 10: '10s', 20: '20s', 40: '40s', 60: '60s', 120: '120s'}, 
                value=20, label='Window'
            ).props('dark outlined dense').style('width: 120px;')

            ui.button('Close', on_click=lambda: dlg.close()).style(
                'padding: 6px 24px; border: none; border-radius: 8px; '
                'background: #2a2a40; color: #e8e8f0; font-weight: 500; cursor: pointer;')

        def refresh_chart():
            hist = history_data.get(jid, {}).get(key)
            if not hist or len(hist['times']) == 0:
                return
            window = time_select.value if time_select.value else 20
            
            t_end = hist['times'][-1]
            t_start = t_end - window
            
            times = []
            values = []
            for t, v in zip(hist['times'], hist['values']):
                if t >= t_start:
                    times.append(f'{t - t_start:.1f}')
                    values.append(round(v, 4))
            
            chart.options['xAxis']['data'] = times
            chart.options['series'][0]['data'] = values
            chart.update()

        timer = ui.timer(0.2, refresh_chart)

    dlg.on('close', lambda: timer.cancel())
    dlg.open()


def update_ui():
    """Periodic UI refresh - called by ui.timer."""
    now = time.time()

    # Update connection status
    badge = ui_labels.get('status_badge')
    dot = ui_labels.get('status_dot')
    stxt = ui_labels.get('status_text')
    ebar = ui_labels.get('error_bar')

    if badge and dot and stxt:
        if can_connected:
            badge._classes = ['status-badge', 'status-connected']
            badge.update()
            dot._props['innerHTML'] = '<span class="status-dot status-dot-green"></span>'
            dot.update()
            stxt.set_text('Connected')
        else:
            badge._classes = ['status-badge', 'status-disconnected']
            badge.update()
            dot._props['innerHTML'] = '<span class="status-dot status-dot-red"></span>'
            dot.update()
            stxt.set_text('Disconnected')

    if ebar:
        if can_error_msg:
            ebar.set_text(f'\u26a0  {can_error_msg}')
            ebar.set_visibility(True)
        else:
            ebar.set_visibility(False)

    # Update each motor card
    for jid in JOINT_IDS:
        d = motor_data[jid]
        last = d['last_update']

        # Helper to update a label
        # Update status/fault
        sl = ui_labels.get(f'status_{jid}')
        si = ui_labels.get(f'status_icon_{jid}')
        if sl:
            status_str, is_err = fmt_fault(d['mode'], d['fault'])
            sl.set_text(status_str)
            sl._classes = ['data-value', 'value-status-err' if is_err else 'value-status-ok']
            sl.update()
        if si:
            if is_err:
                si._props['innerHTML'] = '<div class="data-icon icon-status-err">\u2717</div>'
            else:
                si._props['innerHTML'] = '<div class="data-icon icon-status">\u2713</div>'
            si.update()

        _key_map = {
            'pos': 'position', 'vel': 'velocity', 'torq': 'torque',
            'cur': 'q_current', 'volt': 'voltage', 'temp': 'temperature',
            'mtemp': 'motor_temperature',
        }

        def _upd(key, fmt_fn, css_class):
            lbl = ui_labels.get(f'{key}_{jid}')
            if lbl:
                v = d[_key_map[key]]
                lbl.set_text(fmt_fn(v))
                lbl._classes = ['data-value',
                                css_class if v is not None else 'value-na']
                lbl.update()

        _upd('pos', fmt_pos, 'value-position')
        _upd('vel', fmt_vel, 'value-velocity')
        _upd('torq', fmt_torque, 'value-torque')
        _upd('cur', fmt_current, 'value-current')
        _upd('volt', fmt_volt, 'value-voltage')
        _upd('temp', fmt_temp, 'value-temp')
        _upd('mtemp', fmt_temp, 'value-motor-temp')

        # Footer
        fl = ui_labels.get(f'footer_{jid}')
        if fl:
            if last > 0:
                age = now - last
                if age < 1.0:
                    fl.set_text('Updated just now')
                else:
                    fl.set_text(f'Updated {age:.1f}s ago')
            else:
                fl.set_text('Waiting for data...')

    # Update teaching status
    ts = ui_labels.get('teach_status')
    if ts:
        if teaching_active:
            ts.set_text(f'🔴 Recording... ({len(teaching_frames)} frames)')
        elif replaying_active:
            ts.set_text('▶️ Replaying...')
        else:
            ts.set_text('Idle')

    # Populate PID inputs once after startup read completes
    if pid_read_done:
        for jid in JOINT_IDS:
            for param in PID_PARAMS:
                inp = ui_labels.get(f'pid_{param}_{jid}')
                if inp and pid_data[jid][param] is not None and inp.value == 0.0:
                    inp.value = pid_data[jid][param]
                    inp.update()


# ---------------------------------------------------------------------------
# Application lifecycle
# ---------------------------------------------------------------------------
_can_task = None


async def on_startup():
    global _can_task
    _can_task = asyncio.create_task(can_query_loop())


async def on_shutdown():
    global _can_task
    if _can_task:
        _can_task.cancel()
        try:
            await _can_task
        except asyncio.CancelledError:
            pass


app.on_startup(on_startup)
app.on_shutdown(on_shutdown)


# ---------------------------------------------------------------------------
# Page
# ---------------------------------------------------------------------------
@ui.page('/')
def index():
    ui.page_title('DummyX2 Dashboard')
    build_page()
    ui.timer(UI_REFRESH_INTERVAL_S, update_ui)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ in {'__main__', '__mp_main__'}:
    ui.run(
        title='DummyX2 Dashboard',
        host='0.0.0.0',
        port=8080,
        dark=True,
        reload=False,
    )


