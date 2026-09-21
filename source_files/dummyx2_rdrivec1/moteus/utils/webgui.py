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
import re
import xml.etree.ElementTree as ET

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
GRIPPER_ID = 7                          # Gripper motor ID
ALL_MOTOR_IDS = JOINT_IDS + [GRIPPER_ID]  # All motors including gripper
QUERY_INTERVAL_S = 0.02                # 50 Hz CAN query cycle
UI_REFRESH_INTERVAL_S = 0.1            # 10 Hz UI update
QUERY_TIMEOUT_S = 0.05                 # 50ms timeout per motor query

# ---------------------------------------------------------------------------
# Shared state - written by CAN task, read by UI
# ---------------------------------------------------------------------------
motor_data = {
    mid: {
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
    for mid in ALL_MOTOR_IDS
}

# PID parameters for each joint
PID_PARAMS = ['kp', 'ki', 'kd', 'iratelimit', 'ilimit', 'max_desired_rate']
pid_data = {
    mid: {p: None for p in PID_PARAMS}
    for mid in ALL_MOTOR_IDS
}
pid_read_done = False
can_connected = False
can_error_msg = ''

# History data for charts (last 120s at ~50Hz = 6000 points)
HISTORY_LEN = 6000
def _new_hist():
    return {'times': deque(maxlen=HISTORY_LEN), 'values': deque(maxlen=HISTORY_LEN)}

history_data = {
    mid: {
        'pos': _new_hist(),
        'vel': _new_hist(),
        'torq': _new_hist(),
        'cur': _new_hist()
    }
    for mid in ALL_MOTOR_IDS
}

# Gripper online flag (set during CAN init scan)
gripper_online = False

# Persistent gripper command: when not None, the query loop sends make_position()
# to the gripper every cycle (50Hz) instead of make_query(), preventing moteus
# watchdog timeout (mode=11). Format: {'pos': float, 'torque': float, 'vel': float, 'accel': float}
gripper_cmd = None


# Command queue: UI thread pushes commands, CAN loop processes them
# Commands: ('stop',) ('position', pos, torque, velocity, accel)
#           ('teach_start',) ('teach_stop',) ('teach_repeat',)
command_queue = deque()

# ---------------------------------------------------------------------------
# IK (Inverse Kinematics) state
# ---------------------------------------------------------------------------
_URDF_XACRO = '/home/liyq/moteus/ros2/src/dummyx2_description/urdf/dummyx2.xacro'
_ik_chain    = None
_IK_READY    = False
_ik_last_result = None   # last solved joint angles, list[6] in degrees
_IK_ERROR    = ''        # human-readable init failure reason


def _rpy_from_mat(R):
    """Extract Roll-Pitch-Yaw (degrees) from a 3x3 rotation matrix (numpy)."""
    import math
    sy = math.sqrt(float(R[0, 0])**2 + float(R[1, 0])**2)
    if sy > 1e-6:
        rx = math.atan2(float(R[2, 1]), float(R[2, 2]))
        ry = math.atan2(-float(R[2, 0]), sy)
        rz = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        rx = math.atan2(-float(R[1, 2]), float(R[1, 1]))
        ry = math.atan2(-float(R[2, 0]), sy)
        rz = 0.0
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


def _ik_init():
    """Parse xacro URDF and build ikpy kinematic chain (ikpy >= 3.4)."""
    global _ik_chain, _IK_READY, _IK_ERROR
    try:
        import numpy as np
        from ikpy.chain import Chain
        from ikpy.link import OriginLink, URDFLink

        with open(_URDF_XACRO, 'r') as f:
            src = f.read()
        # Strip ROS-specific xacro macros (not needed for kinematics)
        src = re.sub(r'<xacro:include\b[^>]*/>', '', src)
        src = re.sub(r'\$\(find [^)]+\)', '/dev/null', src)

        root = ET.fromstring(src)
        revolutes = {j.get('name'): j
                     for j in root.findall('joint')
                     if j.get('type') == 'revolute'}

        links = [OriginLink()]
        for name in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']:
            j   = revolutes[name]
            o   = j.find('origin')
            xyz = np.array(list(map(float, o.get('xyz', '0 0 0').split())))
            rpy = np.array(list(map(float, o.get('rpy', '0 0 0').split())))
            ax  = np.array(list(map(float, j.find('axis').get('xyz', '0 0 1').split())))
            lm  = j.find('limit')
            links.append(URDFLink(
                name=name,
                origin_translation=xyz,
                origin_orientation=rpy,
                rotation=ax,
                bounds=(float(lm.get('lower')), float(lm.get('upper')))))
        # Passive end-effector link (same frame as link6_1)
        links.append(URDFLink(name='EE',
                              origin_translation=np.zeros(3),
                              origin_orientation=np.zeros(3),
                              rotation=np.array([0., 1., 0.])))

        _ik_chain = Chain(name='dummyx2', links=links)
        _IK_READY = True
        _IK_ERROR = ''
        # Sanity check: FK at zero config
        T = _ik_chain.forward_kinematics([0.0] * len(links))
        print(f'[IK] Chain ready ({len(links)-2} joints), '
              f'FK@zeros=[{T[0,3]*1000:.1f},{T[1,3]*1000:.1f},{T[2,3]*1000:.1f}]mm')
    except ImportError as e:
        _IK_ERROR = f'pip install ikpy  (missing: {e})'
        print(f'[IK] ikpy not installed: {e}')
        _IK_READY = False
    except Exception as e:
        _IK_ERROR = str(e)
        print(f'[IK] Init failed: {e}')
        import traceback; traceback.print_exc()
        _IK_READY = False

_ik_init()

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

async def change_can_id_motor(jid, new_id):
    """Change the CAN ID of a motor via diagnostic protocol and save to flash."""
    ctrl = controllers.get(jid)
    if not ctrl:
        return False, f'Controller {jid} not found'
    try:
        s = moteus.Stream(ctrl)
        await s.write_message(b'tel stop')
        await s.flush_read()
        await s.command(f'conf set id.id {new_id}'.encode('utf8'))
        await s.command(b'conf write')
        print(f'[CanID] Motor {jid} CAN ID changed to {new_id} and saved to flash.')
        return True, f'CAN ID changed to {new_id}. Please reconnect power.'
    except Exception as e:
        print(f'[CanID] Failed to change CAN ID of motor {jid}: {e}')
        return False, str(e)

async def set_max_position_motor(jid, max_pos_rev):
    """Set servo.position_max on a motor via diagnostic protocol.
    
    Args:
        jid: motor ID
        max_pos_rev: maximum position in revolutions (moteus native unit)
    """
    ctrl = controllers.get(jid)
    if not ctrl:
        return False, f'Controller {jid} not found'
    try:
        s = moteus.Stream(ctrl)
        await s.write_message(b'tel stop')
        await s.flush_read()
        await s.command(f'conf set servopos.position_max {max_pos_rev}'.encode('utf8'))
        print(f'[MaxPos] Motor {jid} position_max set to {max_pos_rev:.6f} rev ({max_pos_rev*360:.2f}°).')
        return True, f'position_max = {max_pos_rev:.4f} rev ({max_pos_rev*360:.1f}°)'
    except Exception as e:
        print(f'[MaxPos] Failed to set position_max on motor {jid}: {e}')
        return False, str(e)

async def set_min_position_motor(jid, min_pos_rev):
    """Set servo.position_min on a motor via diagnostic protocol.
    
    Args:
        jid: motor ID
        min_pos_rev: minimum position in revolutions. A margin of 0.01 is added to prevent exact boundary faults.
    """
    ctrl = controllers.get(jid)
    if not ctrl:
        return False, f'Controller {jid} not found'
    try:
        s = moteus.Stream(ctrl)
        await s.write_message(b'tel stop')
        await s.flush_read()
        target_min = min_pos_rev - 0.01
        await s.command(f'conf set servopos.position_min {target_min}'.encode('utf8'))
        print(f'[MinPos] Motor {jid} position_min set to {target_min:.6f} rev.')
        return True, f'position_min = {target_min:.4f} rev'
    except Exception as e:
        print(f'[MinPos] Failed to set position_min on motor {jid}: {e}')
        return False, str(e)

async def can_query_loop():
    """Background coroutine that queries all moteus controllers (joints 1-6 + gripper 7)."""
    global can_connected, can_error_msg, transport, controllers
    global pid_read_done, gripper_online
    
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
        mid: moteus.Controller(id=mid, transport=transport,
                               query_resolution=qr)
        for mid in ALL_MOTOR_IDS
    }

    # Send initial stop to clear any latched faults (no query, so no reply expected)
    async with transport_lock:
        for jid, ctrl in controllers.items():
            try:
                await transport.cycle([ctrl.make_stop()])
            except Exception as e:
                print(f'[WARN] Stop motor {jid} failed: {e}')

    # Auto-enable gravity-bearing joints immediately after fault-clear,
    # BEFORE PID read (which can take several seconds across 6 motors).
    # position=NaN → hold-in-place mode at current angle.
    _AUTO_ENABLE_JIDS   = [2, 3]
    _AUTO_ENABLE_TORQUE = 3.0   # Nm
    _AUTO_ENABLE_VEL    = 0.5   # rev/s
    _AUTO_ENABLE_ACCEL  = 0.5   # rev/s²
    print(f'[INFO] Auto-enabling joints {_AUTO_ENABLE_JIDS} (hold current position)...')
    async with transport_lock:
        for _jid in _AUTO_ENABLE_JIDS:
            _ctrl = controllers.get(_jid)
            if _ctrl:
                try:
                    await asyncio.wait_for(
                        transport.cycle([
                            _ctrl.make_position(
                                position=math.nan,
                                velocity=0.0,
                                maximum_torque=_AUTO_ENABLE_TORQUE,
                                velocity_limit=_AUTO_ENABLE_VEL,
                                accel_limit=_AUTO_ENABLE_ACCEL,
                                query=True)
                        ]),
                        timeout=QUERY_TIMEOUT_S)
                    print(f'[INFO]   J{_jid} enabled (hold)')
                except Exception as _e:
                    print(f'[WARN]   J{_jid} auto-enable failed: {_e}')
    print('[INFO] Auto-enable complete')

    # Read PID parameters from each motor at startup
    print('[INFO] Reading PID parameters from motors...')
    async with transport_lock:
        for mid, ctrl in controllers.items():
            await read_pid_from_motor(mid, ctrl)
    pid_read_done = True
    print('[INFO] PID parameter read complete')

    # Scan gripper (ID 7) to check if it's present
    print(f'[INFO] Scanning gripper (ID {GRIPPER_ID})...')
    gripper_ctrl = controllers.get(GRIPPER_ID)
    if gripper_ctrl:
        try:
            async with transport_lock:
                results = await asyncio.wait_for(
                    transport.cycle([gripper_ctrl.make_query()]),
                    timeout=QUERY_TIMEOUT_S * 2)
            if results:
                gripper_online = True
                print(f'[INFO] ✅ Gripper (ID {GRIPPER_ID}) detected and online')
            else:
                gripper_online = False
                print(f'[WARN] ⚠ Gripper (ID {GRIPPER_ID}) not responding')
        except (asyncio.TimeoutError, Exception) as e:
            gripper_online = False
            print(f'[WARN] ⚠ Gripper (ID {GRIPPER_ID}) scan failed: {e}')
    else:
        gripper_online = False
        print(f'[WARN] ⚠ Gripper controller not created')

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
                                print(f'[CMD]   Motor {jid}: stopped OK')
                            except asyncio.TimeoutError:
                                print(f'[CMD]   Motor {jid}: stop TIMEOUT')
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
                                results = await asyncio.wait_for(
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
                                # Log response for debugging
                                if results:
                                    for r in results:
                                        m = r.values.get(Register.MODE, None)
                                        f = r.values.get(Register.FAULT, None)
                                        p = r.values.get(Register.POSITION, None)
                                        print(f'[CMD]   Motor {r.id} response: mode={m} fault={f} pos={p}')
                                else:
                                    print(f'[CMD]   Motor {jid}: no response!')
                            except asyncio.TimeoutError:
                                print(f'[CMD]   Motor {jid}: TIMEOUT (no CAN response)')
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
                elif cmd[0] == 'change_can_id':
                    jid_chg = cmd[1]
                    new_id_chg = cmd[2]
                    result_future = cmd[3]
                    async with transport_lock:
                        ok, msg = await change_can_id_motor(jid_chg, new_id_chg)
                    result_future.set_result((ok, msg))
                elif cmd[0] == 'set_max_position':
                    jid_max = cmd[1]
                    max_pos_rev = cmd[2]
                    result_future = cmd[3]
                    async with transport_lock:
                        ok, msg = await set_max_position_motor(jid_max, max_pos_rev)
                    result_future.set_result((ok, msg))
                elif cmd[0] == 'set_min_position':
                    jid_min = cmd[1]
                    min_pos_rev = cmd[2]
                    result_future = cmd[3]
                    async with transport_lock:
                        ok, msg = await set_min_position_motor(jid_min, min_pos_rev)
                    result_future.set_result((ok, msg))
                elif cmd[0] == 'sync_position':
                    # joint_cmds: {jid: (pos_rev, torque, vel_limit, accel_limit)}
                    joint_cmds = cmd[1]
                    msgs = []
                    for _jid, (_pos, _torq, _vel, _acc) in joint_cmds.items():
                        _ctrl = controllers.get(_jid)
                        if _ctrl:
                            msgs.append(_ctrl.make_position(
                                position=_pos,
                                velocity=0.0,
                                maximum_torque=_torq,
                                velocity_limit=_vel,
                                accel_limit=_acc,
                                query=True))
                    if msgs:
                        async with transport_lock:
                            try:
                                await asyncio.wait_for(
                                    transport.cycle(msgs),
                                    timeout=QUERY_TIMEOUT_S * max(len(msgs), 1))
                            except asyncio.TimeoutError:
                                pass
                        print(f'[CMD] sync_position sent {len(msgs)} joints in one batch')
            except Exception as e:
                print(f'[ERROR] Command {cmd}: {e}')

        # --- Query all motors ---
        got_any = False

        for mid, ctrl in controllers.items():
            try:
                # For gripper with active command: send make_position() every cycle
                # to prevent moteus watchdog timeout (mode=11)
                if mid == GRIPPER_ID and gripper_cmd is not None:
                    async with transport_lock:
                        results = await asyncio.wait_for(
                            transport.cycle([ctrl.make_position(
                                position=gripper_cmd['pos'],
                                velocity=0.0,
                                maximum_torque=gripper_cmd['torque'],
                                velocity_limit=gripper_cmd['vel'],
                                accel_limit=gripper_cmd['accel'],
                                query=True)]),
                            timeout=QUERY_TIMEOUT_S)
                else:
                    # Regular query for joints and idle gripper
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
                motor_data[mid]['online'] = False
            except Exception as e:
                motor_data[mid]['online'] = False
                print(f'[WARN] Query motor {mid}: {e}')

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

.motor-card-gripper::before {
    background: linear-gradient(90deg, #f97316, #fbbf24) !important;
    opacity: 0.9;
}

.joint-id-badge-gripper {
    background: linear-gradient(135deg, #f97316, #fbbf24) !important;
    box-shadow: 0 4px 12px rgba(249, 115, 22, 0.3) !important;
    width: auto !important;
    padding: 0 10px !important;
    font-size: 0.78rem !important;
    white-space: nowrap;
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

.btn-ready {
    background: linear-gradient(135deg, #06d6a0, #00b386);
    color: #0f1a14;
    font-weight: 700;
    letter-spacing: 0.04em;
    box-shadow: 0 0 12px rgba(6, 214, 160, 0.35);
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

                    # ---- CAN ID Change Row ----
                    with ui.element('div').style(
                            'margin-top: 12px; padding-top: 10px; '
                            'border-top: 1px solid var(--border-color);'):
                        ui.html('<div style="font-size:0.75rem; font-weight:600; '
                                'color: var(--accent-amber); margin-bottom:6px; '
                                'letter-spacing:0.03em;">&#128279; CAN ID</div>')
                        with ui.element('div').classes('pid-row'):
                            ui.label('New CAN ID (1-9)').classes('pid-label')
                            can_id_inp = ui.number(
                                value=jid, min=1, max=9, step=1, format='%d'
                            ).style(
                                'width: 80px; border-radius: 8px; font-size: 0.82rem;'
                            )
                            ui_labels[f'can_id_inp_{jid}'] = can_id_inp

                        def _make_change_can_id(j):
                            async def change_can_id_click():
                                st = ui_labels.get(f'pid_status_{j}')
                                inp = ui_labels.get(f'can_id_inp_{j}')
                                if not inp or inp.value is None:
                                    return
                                new_id = int(inp.value)
                                if new_id < 1 or new_id > 9:
                                    if st:
                                        st.set_text('\u2717 CAN ID must be 1-9')
                                        st._classes = ['pid-status', 'pid-status-err']
                                        st.update()
                                    return
                                if st:
                                    st.set_text(f'Changing CAN ID to {new_id}...')
                                    st._classes = ['pid-status']
                                    st.update()
                                fut = asyncio.get_event_loop().create_future()
                                command_queue.append(('change_can_id', j, new_id, fut))
                                ok, msg = await fut
                                if st:
                                    if ok:
                                        st.set_text(f'\u2713 {msg}')
                                        st._classes = ['pid-status', 'pid-status-ok']
                                    else:
                                        st.set_text(f'\u2717 Error: {msg}')
                                        st._classes = ['pid-status', 'pid-status-err']
                                    st.update()
                            return change_can_id_click

                        ui.button('\U0001f4be Apply CAN ID',
                                  on_click=_make_change_can_id(jid)).props(
                            'flat dense no-caps').classes('pid-apply-btn').style(
                            'color: #ffd166 !important; width: 100%; margin-top: 6px; '
                            'background: linear-gradient(135deg, #b45309, #92400e) !important;')



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
            ui.button('Disable', on_click=lambda: confirm_disable()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 600; cursor: pointer; '
                'background: linear-gradient(135deg, #ef476f, #d63a5c); color: #fff;')
            ui.button('✅ Ready', on_click=lambda: on_ready()).style(
                'padding: 8px 22px; border: none; border-radius: 10px; '
                'font-weight: 700; cursor: pointer; '
                'background: linear-gradient(135deg, #06d6a0, #00b386); '
                'color: #0f1a14; box-shadow: 0 0 12px rgba(6,214,160,0.35);')

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
            vel_input = ui.number(value=0.05, min=0.01, max=0.5, step=0.01, format='%.2f').style(
                'width: 80px; border-radius: 10px; font-weight: 500; text-align: center;')
            ui_labels['vel_input'] = vel_input

            ui.html('<span class="control-label">Accel:</span>')
            accel_input = ui.number(value=0.1, min=0.01, max=0.5, step=0.01, format='%.2f').style(
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

    # ---- FK Sync Move Panel ----
    with ui.element('div').classes('control-panel'):
        ui.html('<div class="control-panel-title">\U0001f3af Sync Move — All Joints Simultaneously</div>')
        # 6-column grid: one cell per joint
        with ui.element('div').style(
                'display: grid; grid-template-columns: repeat(6, 1fr); '
                'gap: 14px; margin-bottom: 16px;'):
            for _jfk in JOINT_IDS:
                with ui.element('div').style(
                        'display: flex; flex-direction: column; '
                        'align-items: center; gap: 6px;'):
                    ui.html(
                        f'<div style="font-size:0.78rem; font-weight:700; '
                        f'color: var(--accent-cyan); letter-spacing:0.04em;">J{_jfk}</div>')
                    _fk_inp = ui.number(value=0.0, step=1.0, format='%.1f').style(
                        'width: 80px; border-radius: 8px; font-size: 0.9rem; '
                        'font-weight: 600; text-align: center;')
                    ui_labels[f'fk_pos_{_jfk}'] = _fk_inp
                    _curr_lbl = ui.label('--').style(
                        'font-size: 0.72rem; color: var(--text-dim); '
                        'font-variant-numeric: tabular-nums;')
                    ui_labels[f'fk_curr_{_jfk}'] = _curr_lbl

        with ui.element('div').classes('control-row'):
            ui.button('\U0001f4d0 Move All (Sync)', on_click=lambda: on_fk_move()).style(
                'padding: 8px 24px; border: none; border-radius: 10px; '
                'font-weight: 700; cursor: pointer; '
                'background: linear-gradient(135deg, #4361ee, #4cc9f0); color: #fff; '
                'box-shadow: 0 0 12px rgba(67,97,238,0.4);')
            ui.button('\U0001f4cb Copy Current', on_click=lambda: on_fk_copy_current()).style(
                'padding: 8px 20px; border: none; border-radius: 10px; '
                'font-weight: 600; cursor: pointer; '
                'background: rgba(67,97,238,0.15); color: var(--accent-cyan); '
                'border: 1px solid rgba(67,97,238,0.35);')
            _fk_status = ui.label('Input target angles (\u00b0) and press Move All').style(
                'font-size: 0.82rem; color: var(--text-dim); font-style: italic; flex: 1;')
            ui_labels['fk_status'] = _fk_status
            # Help link
            ui.html('<div style="margin-top:10px; text-align:right;">'
                    '<a href="#" onclick="return false;" '
                    'style="font-size:0.75rem; color:var(--accent-cyan); '
                    'text-decoration:none; opacity:0.7;" '
                    'id="fk-help-link">&#128218; 公式说明</a></div>').on(
                'click', lambda e: show_fk_help())

    # ---- IK Panel ----
    with ui.element('div').classes('control-panel'):
        if not _IK_READY:
            ui.html(f'<div style="color:#ef476f; font-weight:600;">'
                    f'&#9888; IK unavailable: <code>{_IK_ERROR or "unknown error"}</code></div>')
        else:
            ui.html('<div class="control-panel-title">&#128279; Inverse Kinematics (IK)</div>')

            # Position + Orientation inputs
            # Default = mid-workspace pose (arm bent, ~60% reach, away from singularities)
            # Max arm reach ≈ 504mm from J1; default dist ≈ 288mm (57%) — safely interior
            _IK_DEFAULTS = {
                'ik_x':  0.0, 'ik_y': 180.0, 'ik_z': 300.0,
                'ik_rx': 0.0, 'ik_ry':   0.0, 'ik_rz':  0.0,
            }
            with ui.element('div').style(
                    'display:grid; grid-template-columns:repeat(6,1fr); '
                    'gap:12px; margin-bottom:14px; align-items:end;'):
                for _lbl, _key, _clr, _step in [
                        ('X (mm)', 'ik_x', 'var(--accent-cyan)',  0.1),
                        ('Y (mm)', 'ik_y', 'var(--accent-cyan)',  0.1),
                        ('Z (mm)', 'ik_z', 'var(--accent-cyan)',  0.1),
                        ('Rx (\u00b0)', 'ik_rx', 'var(--accent-amber)', 1.0),
                        ('Ry (\u00b0)', 'ik_ry', 'var(--accent-amber)', 1.0),
                        ('Rz (\u00b0)', 'ik_rz', 'var(--accent-amber)', 1.0),
                ]:
                    with ui.element('div').style(
                            'display:flex; flex-direction:column; gap:4px;'):
                        ui.html(f'<div style="font-size:0.75rem; font-weight:600; '
                                f'color:{_clr};">{_lbl}</div>')
                        _ik_inp = ui.number(value=_IK_DEFAULTS[_key],
                                            step=_step, format='%.2f').style(
                            'width:100%; border-radius:8px; font-size:0.9rem;')
                        ui_labels[_key] = _ik_inp


            # Orient checkbox + buttons row
            with ui.element('div').classes('control-row').style('flex-wrap:wrap; gap:10px;'):
                _ik_orient_cb = ui.checkbox('Constrain orientation').props('dark').style(
                    'font-size:0.82rem; color:var(--text-secondary);')
                ui_labels['ik_orient_cb'] = _ik_orient_cb

                ui.button('\U0001f4cd FK\u2192Preview', on_click=lambda: on_ik_fk_preview()).style(
                    'padding:8px 18px; border:none; border-radius:10px; font-weight:600; '
                    'cursor:pointer; background:rgba(67,97,238,0.25); '
                    'color:var(--accent-cyan); border:1px solid rgba(67,97,238,0.4);')
                ui.button('\U0001f50d Solve IK', on_click=lambda: on_ik_solve()).style(
                    'padding:8px 20px; border:none; border-radius:10px; font-weight:700; '
                    'cursor:pointer; background:linear-gradient(135deg,#4361ee,#4cc9f0); color:#fff; '
                    'box-shadow:0 0 12px rgba(67,97,238,0.4);')
                ui.button('\u2705 Apply', on_click=lambda: on_ik_apply()).style(
                    'padding:8px 20px; border:none; border-radius:10px; font-weight:700; '
                    'cursor:pointer; background:linear-gradient(135deg,#06d6a0,#00b386); color:#0f1a14;')

            # Status label
            _ik_status = ui.label('Enter target position then click Solve IK.').style(
                'font-size:0.82rem; color:var(--text-dim); font-style:italic; margin-top:8px;')
            ui_labels['ik_status'] = _ik_status

            # Result: 6 joint angle boxes
            with ui.element('div').style(
                    'display:grid; grid-template-columns:repeat(6,1fr); '
                    'gap:10px; margin-top:12px;'):
                for _jik in JOINT_IDS:
                    with ui.element('div').style(
                            'display:flex; flex-direction:column; align-items:center; gap:4px; '
                            'background:rgba(15,15,26,0.6); border:1px solid var(--border-color); '
                            'border-radius:10px; padding:8px;'):
                        ui.html(f'<div style="font-size:0.72rem; font-weight:700; '
                                f'color:var(--accent-cyan);">J{_jik}</div>')
                        _ik_lbl = ui.label('--.--\u00b0').style(
                            'font-size:1rem; font-weight:700; color:var(--text-primary); '
                            'font-variant-numeric:tabular-nums;')
                        ui_labels[f'ik_result_{_jik}'] = _ik_lbl

            # Residual label
            _ik_resid = ui.label('').style(
                'font-size:0.72rem; color:var(--text-dim); margin-top:6px; text-align:right;')
            ui_labels['ik_residual'] = _ik_resid
            # Help link
            ui.html('<div style="margin-top:10px; text-align:right;">'
                    '<a href="#" onclick="return false;" '
                    'style="font-size:0.75rem; color:var(--accent-cyan); '
                    'text-decoration:none; opacity:0.7;" '
                    'id="ik-help-link">&#128218; 公式说明</a></div>').on(
                'click', lambda e: show_ik_help())

    # ---- Gripper Control Panel ----
    with ui.element('div').classes('control-panel'):
        ui.html('<div class="control-panel-title" style="color: #f97316;">'
                '\U0001f9be 夹爪控制 (Gripper Control)</div>')
        with ui.element('div').style('display: flex; gap: 40px; align-items: flex-start; flex-wrap: wrap;'):
            with ui.element('div').style('min-width: 300px; max-width: 380px; flex: 1;'):
                # ---- Gripper Card (ID 7) ----
                with ui.element('div').classes('motor-card motor-card-gripper'):
                    # Card header
                    with ui.element('div').classes('card-header'):
                        ui.html(f'<span class="joint-label">\U0001f9be</span>')
                        ui.html(f'<span class="joint-id-badge joint-id-badge-gripper">夹爪7</span>')
    
                    # Status row
                    with ui.element('div').classes('data-row'):
                        with ui.element('div').classes('data-label'):
                            status_icon = ui.html('<div class="data-icon icon-status">\u2713</div>')
                            ui.label('Status').style('margin: 0; padding: 0;')
                            ui_labels[f'status_icon_{GRIPPER_ID}'] = status_icon
                        status_label = ui.label('N/A').classes('data-value value-na')
                        status_label.style('margin: 0; padding: 0;')
                        ui_labels[f'status_{GRIPPER_ID}'] = status_label
    
                    # Position row (clickable for chart)
                    with ui.element('div').classes('data-row').style('cursor: pointer;') as gpos_row:
                        with ui.element('div').classes('data-label'):
                            ui.html('<div class="data-icon icon-position">\u27f3</div>')
                            ui.label('Position (°)').style('margin: 0; padding: 0;')
                        pos_label = ui.label('N/A').classes('data-value value-na')
                        pos_label.style('margin: 0; padding: 0;')
                        ui_labels[f'pos_{GRIPPER_ID}'] = pos_label
                        ui.html('<span style="font-size:0.7rem; color: var(--text-dim); margin-left: 4px;">\u2197</span>')
                    gpos_row.on('click', lambda e: show_chart(GRIPPER_ID, 'pos', 'Position', 'rev', '#4cc9f0'))
    
                    # Velocity row (clickable for chart)
                    with ui.element('div').classes('data-row').style('cursor: pointer;') as gvel_row:
                        with ui.element('div').classes('data-label'):
                            ui.html('<div class="data-icon icon-velocity">\u2192</div>')
                            ui.label('Velocity (rev/s)').style('margin: 0; padding: 0;')
                        vel_label = ui.label('N/A').classes('data-value value-na')
                        vel_label.style('margin: 0; padding: 0;')
                        ui_labels[f'vel_{GRIPPER_ID}'] = vel_label
                        ui.html('<span style="font-size:0.7rem; color: var(--text-dim); margin-left: 4px;">\u2197</span>')
                    gvel_row.on('click', lambda e: show_chart(GRIPPER_ID, 'vel', 'Velocity', 'rev/s', '#a78bfa'))
    
                    # Torque row (clickable for chart)
                    with ui.element('div').classes('data-row').style('cursor: pointer;') as gtorq_row:
                        with ui.element('div').classes('data-label'):
                            ui.html('<div class="data-icon icon-torque">\u21bb</div>')
                            ui.label('Torque (Nm)').style('margin: 0; padding: 0;')
                        torq_label = ui.label('N/A').classes('data-value value-na')
                        torq_label.style('margin: 0; padding: 0;')
                        ui_labels[f'torq_{GRIPPER_ID}'] = torq_label
                        ui.html('<span style="font-size:0.7rem; color: var(--text-dim); margin-left: 4px;">\u2197</span>')
                    gtorq_row.on('click', lambda e: show_chart(GRIPPER_ID, 'torq', 'Torque', 'Nm', '#ef476f'))
    
                    # Current row (clickable for chart)
                    with ui.element('div').classes('data-row').style('cursor: pointer;') as gcur_row:
                        with ui.element('div').classes('data-label'):
                            ui.html('<div class="data-icon icon-current">\u2301</div>')
                            ui.label('Current (A)').style('margin: 0; padding: 0;')
                        cur_label = ui.label('N/A').classes('data-value value-na')
                        cur_label.style('margin: 0; padding: 0;')
                        ui_labels[f'cur_{GRIPPER_ID}'] = cur_label
                        ui.html('<span style="font-size:0.7rem; color: var(--text-dim); margin-left: 4px;">\u2197</span>')
                    gcur_row.on('click', lambda e: show_chart(GRIPPER_ID, 'cur', 'Current', 'A', '#22d3ee'))
    
                    # Voltage row
                    with ui.element('div').classes('data-row'):
                        with ui.element('div').classes('data-label'):
                            ui.html('<div class="data-icon icon-voltage">\u26a1</div>')
                            ui.label('Voltage').style('margin: 0; padding: 0;')
                        volt_label = ui.label('N/A').classes('data-value value-na')
                        volt_label.style('margin: 0; padding: 0;')
                        ui_labels[f'volt_{GRIPPER_ID}'] = volt_label
    
                    # MOSFET Temperature row
                    with ui.element('div').classes('data-row'):
                        with ui.element('div').classes('data-label'):
                            ui.html('<div class="data-icon icon-temp">\U0001f321</div>')
                            ui.label('MOSFET Temp').style('margin: 0; padding: 0;')
                        temp_label = ui.label('N/A').classes('data-value value-na')
                        temp_label.style('margin: 0; padding: 0;')
                        ui_labels[f'temp_{GRIPPER_ID}'] = temp_label
    
                    # Motor Temperature row
                    with ui.element('div').classes('data-row'):
                        with ui.element('div').classes('data-label'):
                            ui.html('<div class="data-icon icon-motor-temp">\U0001f525</div>')
                            ui.label('Motor Temp').style('margin: 0; padding: 0;')
                        mtemp_label = ui.label('N/A').classes('data-value value-na')
                        mtemp_label.style('margin: 0; padding: 0;')
                        ui_labels[f'mtemp_{GRIPPER_ID}'] = mtemp_label
    
                    # Card footer - last update time
                    footer = ui.label('Waiting for data...').classes('card-footer')
                    footer.style('margin: 0; padding: 0;')
                    ui_labels[f'footer_{GRIPPER_ID}'] = footer
    
                    # ---- PID Tuning Collapsible Section (Gripper) ----
                    gpid_panel = ui.element('div').classes('pid-panel')
                    gpid_panel.set_visibility(False)
                    ui_labels[f'pid_panel_{GRIPPER_ID}'] = gpid_panel
    
                    def _make_gpid_toggle(panel):
                        def toggle():
                            vis = not panel.visible
                            panel.set_visibility(vis)
                        return toggle
    
                    ui.button('\u2699 PID Tuning', on_click=_make_gpid_toggle(gpid_panel)).props(
                        'flat dense no-caps').classes('pid-toggle-btn')
    
                    with gpid_panel:
                        ui.html('<div class="pid-panel-title">\u2699 Position PID Parameters</div>')
                        for param in PID_PARAMS:
                            with ui.element('div').classes('pid-row'):
                                ui.label(param).classes('pid-label')
                                inp = ui.number(
                                    value=0.0, format='%.6f', step=0.1
                                ).style(
                                    'width: 110px; border-radius: 8px; font-size: 0.82rem;'
                                )
                                ui_labels[f'pid_{param}_{GRIPPER_ID}'] = inp
    
                        gpid_status = ui.label('').classes('pid-status')
                        ui_labels[f'pid_status_{GRIPPER_ID}'] = gpid_status
    
                        def _make_gset_home():
                            async def set_home_offset():
                                st = ui_labels.get(f'pid_status_{GRIPPER_ID}')
                                if st:
                                    st.set_text('Setting Home Offset...')
                                    st._classes = ['pid-status']
                                    st.update()
                                fut = asyncio.get_event_loop().create_future()
                                command_queue.append(('set_home', GRIPPER_ID, fut))
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
    
                        def _make_gapply():
                            async def apply_pid():
                                st = ui_labels.get(f'pid_status_{GRIPPER_ID}')
                                vals = {}
                                for p in PID_PARAMS:
                                    inp = ui_labels.get(f'pid_{p}_{GRIPPER_ID}')
                                    if inp and inp.value is not None:
                                        vals[p] = float(inp.value)
                                if st:
                                    st.set_text('Applying...')
                                    st._classes = ['pid-status']
                                    st.update()
                                fut = asyncio.get_event_loop().create_future()
                                command_queue.append(('pid_apply', GRIPPER_ID, vals, fut))
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
                                command_queue.append(('pid_read', GRIPPER_ID, fut2))
                                await fut2
                                _refresh_pid_inputs(GRIPPER_ID)
                            return apply_pid
    
                        with ui.element('div').style('display: flex; gap: 10px; margin-top: 5px;'):
                            ui.button('\u2714 Apply PID', on_click=_make_gapply()).props(
                                'flat dense no-caps').classes('pid-apply-btn').style(
                                'color: #ffd166 !important; flex: 1;')
    
                            ui.button('\U0001f3e0 Set Home', on_click=_make_gset_home()).props(
                                'flat dense no-caps').classes('pid-apply-btn').style(
                                'color: #06d6a0 !important; flex: 1; font-weight: bold;')
            with ui.element('div').style('flex: 2; min-width: 500px; display: flex; flex-direction: column; gap: 24px;'):

                # Enable / Disable row & Status
                with ui.element('div').style('display: flex; gap: 16px; align-items: center; flex-wrap: wrap;'):
                    ui.button('ENABLE', on_click=lambda: on_gripper_enable()).style(
                        'padding: 8px 24px; border: none; border-radius: 10px; '
                        'font-weight: 700; cursor: pointer; letter-spacing: 0.05em; '
                        'background: linear-gradient(135deg, #f97316, #ea580c); color: #fff; '
                        'box-shadow: 0 0 12px rgba(249,115,22,0.3);')
                    ui.button('DISABLE', on_click=lambda: on_gripper_disable()).style(
                        'padding: 8px 24px; border: none; border-radius: 10px; '
                        'font-weight: 700; cursor: pointer; letter-spacing: 0.05em; '
                        'background: linear-gradient(135deg, #475569, #334155); color: #fff; '
                        'box-shadow: 0 0 12px rgba(0,0,0,0.3);')

                    ui.element('div').style('width: 2px; height: 24px; background: rgba(255,255,255,0.1); margin: 0 8px;')

                    # Gripper status label
                    ui.html('<span style="font-size: 0.85rem; color: var(--text-dim); font-weight: 600;">Status:</span>')
                    _grip_status = ui.label('Idle').style(
                        'font-size: 0.95rem; color: var(--text-primary); font-weight: 700; background: rgba(249,115,22,0.15); padding: 4px 12px; border-radius: 6px; border: 1px solid rgba(249,115,22,0.3);')
                    ui_labels['gripper_status'] = _grip_status

                # Sliders grid: 2 columns
                with ui.element('div').style(
                        'display: grid; grid-template-columns: repeat(2, 1fr); '
                        'gap: 30px; background: rgba(0,0,0,0.15); padding: 20px 24px 24px; border-radius: 12px; border: 1px solid rgba(255,255,255,0.05);'):

                    # Position slider
                    with ui.element('div').style('display: flex; flex-direction: column;'):
                        with ui.element('div').style('display: flex; justify-content: space-between; align-items: baseline;'):
                            ui.html('<span style="font-size: 0.85rem; font-weight: 700; color: var(--accent-cyan); letter-spacing: 0.04em;">Position (mm)</span>')
                            _grip_pos_lbl = ui.label('0.0 mm').style(
                                'font-size: 0.82rem; color: var(--text-primary); font-weight: 600;')
                            ui_labels['gripper_pos_label'] = _grip_pos_lbl
                        _grip_pos = ui.slider(min=0.0, max=95.0, step=0.5, value=0.0, on_change=lambda e: on_gripper_move()).props('label-always color="cyan"').style('width: 100%; margin-top: 28px;')
                        ui_labels['gripper_pos_slider'] = _grip_pos

                    # Torque slider
                    with ui.element('div').style('display: flex; flex-direction: column;'):
                        with ui.element('div').style('display: flex; justify-content: space-between; align-items: baseline;'):
                            ui.html('<span style="font-size: 0.85rem; font-weight: 700; color: #f97316; letter-spacing: 0.04em;">Torque (Nm)</span>')
                            _grip_torq_lbl = ui.label('0.10 Nm').style(
                                'font-size: 0.82rem; color: var(--text-primary); font-weight: 600;')
                            ui_labels['gripper_torq_label'] = _grip_torq_lbl
                        _grip_torq = ui.slider(min=0, max=10, step=0.01, value=0.1, on_change=lambda e: on_gripper_move()).props('label-always color="orange"').style('width: 100%; margin-top: 28px;')
                        ui_labels['gripper_torq_slider'] = _grip_torq

                    # Velocity slider
                    with ui.element('div').style('display: flex; flex-direction: column;'):
                        with ui.element('div').style('display: flex; justify-content: space-between; align-items: baseline;'):
                            ui.html('<span style="font-size: 0.85rem; font-weight: 700; color: #a78bfa; letter-spacing: 0.04em;">Velocity (rev/s)</span>')
                            _grip_vel_lbl = ui.label('1.00 rev/s').style(
                                'font-size: 0.82rem; color: var(--text-primary); font-weight: 600;')
                            ui_labels['gripper_vel_label'] = _grip_vel_lbl
                        _grip_vel = ui.slider(min=0, max=10, step=0.1, value=1.0).props('label-always color="purple"').style('width: 100%; margin-top: 28px;')
                        ui_labels['gripper_vel_slider'] = _grip_vel

                    # Accel slider
                    with ui.element('div').style('display: flex; flex-direction: column;'):
                        with ui.element('div').style('display: flex; justify-content: space-between; align-items: baseline;'):
                            ui.html('<span style="font-size: 0.85rem; font-weight: 700; color: #fbbf24; letter-spacing: 0.04em;">Accel (rev/s²)</span>')
                            _grip_acc_lbl = ui.label('1.00 rev/s²').style(
                                'font-size: 0.82rem; color: var(--text-primary); font-weight: 600;')
                            ui_labels['gripper_acc_label'] = _grip_acc_lbl
                        _grip_acc = ui.slider(min=0, max=5, step=0.1, value=1.0).props('label-always color="amber"').style('width: 100%; margin-top: 28px;')
                        ui_labels['gripper_acc_slider'] = _grip_acc

                # Move button row
                with ui.element('div').style('display: flex; gap: 24px; align-items: flex-end; flex-wrap: wrap;'):
                    
                    # Actions
                    with ui.element('div').style('display: flex; gap: 12px;'):
                        ui.button('📍 Move Gripper', on_click=lambda: on_gripper_move()).style(
                            'padding: 8px 28px; border: none; border-radius: 10px; '
                            'font-weight: 700; cursor: pointer; '
                            'background: linear-gradient(135deg, #f97316, #fbbf24); color: #1a1a2e; '
                            'box-shadow: 0 0 16px rgba(249,115,22,0.4); font-size: 0.95rem;')
                        ui.button('🏠 Home (0)', on_click=lambda: on_gripper_home()).style(
                            'padding: 8px 24px; border: none; border-radius: 10px; '
                            'font-weight: 700; cursor: pointer; '
                            'background: rgba(249,115,22,0.15); color: #f97316; '
                            'border: 1px solid rgba(249,115,22,0.35);')

                    ui.element('div').style('width: 2px; height: 32px; background: rgba(255,255,255,0.1);')

                    # Position Limits config
                    with ui.element('div').style('display: flex; align-items: center; gap: 12px; background: rgba(0,0,0,0.15); padding: 8px 20px; border-radius: 10px; border: 1px solid rgba(255,255,255,0.05);'):
                        ui.html('<span style="font-size: 0.85rem; font-weight: 600; color: var(--accent-cyan);">Current (mm):</span>')
                        _grip_angle_input = ui.number(
                            value=0.0, format='%.2f', step=0.1
                        ).style('width: 80px; font-weight: 600; text-align: center; font-size: 0.9rem;').props('dense dark filled')
                        ui_labels['gripper_angle_input'] = _grip_angle_input

                        ui.button('SET MIN', on_click=lambda: on_gripper_set_min_pos()).style(
                            'padding: 4px 12px; border-radius: 6px; font-weight: 700; cursor: pointer; font-size: 0.75rem; '
                            'background: #3b82f6; color: #fff; box-shadow: 0 0 8px rgba(59,130,246,0.3);')
                        
                        ui.button('SET MAX', on_click=lambda: on_gripper_set_max_pos()).style(
                            'padding: 4px 12px; border-radius: 6px; font-weight: 700; cursor: pointer; font-size: 0.75rem; '
                            'background: #06b6d4; color: #fff; box-shadow: 0 0 8px rgba(6,182,212,0.3);')



    def _refresh_pid_inputs(jid):
        """Update PID input fields from pid_data for a given joint."""
        for param in PID_PARAMS:
            inp = ui_labels.get(f'pid_{param}_{jid}')
            if inp and pid_data[jid][param] is not None:
                inp.value = pid_data[jid][param]
                inp.update()


    _HELP_STYLE = (
        'background:#1a1a2e; border:1px solid #2a2a40; border-radius:16px; '
        'padding:28px 32px; max-width:720px; width:95vw; '
        'color:#e8e8f0; font-family:Inter,sans-serif; overflow-y:auto; max-height:85vh;'
    )
    _H2 = 'font-size:1.1rem; font-weight:700; color:#4cc9f0; margin:18px 0 8px; border-bottom:1px solid #2a2a40; padding-bottom:4px;'
    _P  = 'font-size:0.88rem; color:#a0a0b8; line-height:1.7; margin:6px 0;'
    _CODE = 'background:#0f0f1a; border:1px solid #2a2a40; border-radius:8px; padding:10px 14px; font-size:0.82rem; color:#4cc9f0; font-family:monospace; white-space:pre; overflow-x:auto; display:block; margin:8px 0;'
    _EM   = 'color:#ffd166; font-weight:600;'


    def show_fk_help():
        """Show a popup dialog explaining FK Sync Move principles."""
        with ui.dialog() as dlg, ui.element('div').style(_HELP_STYLE):
            ui.html(f'''
    <h2 style="{_H2.replace("margin:18px 0 8px","margin:0 0 8px")}; font-size:1.3rem;">
      &#127917; Sync Move — 同步多关节运动原理
    </h2>

    <h3 style="{_H2}">&#9654; 核心问题</h3>
    <p style="{_P}">
      各关节移动距离不同，如何让它们<span style="{_EM}">同时到达</span>目标位置？<br>
      只需让所有关节共享同一个<span style="{_EM}">运动时间 T</span>。
    </p>

    <h3 style="{_H2}">&#9654; 梯形速度曲线时间公式</h3>
    <p style="{_P}">Moteus 内部采用梯形（或三角型）速度规划，每个关节的运动时间取决于：</p>
    <code style="{_CODE}">T = d / v_max + v_max / a_max     （梯形，峰速可达时）
    T = 2 × √(d / a_max)              （三角型，峰速不可达时）</code>
    <p style="{_P}">
      其中 <span style="{_EM}">d</span> 为运动距离（rev），
      <span style="{_EM}">v_max</span> 为速度限制（rev/s），
      <span style="{_EM}">a_max</span> 为加速度限制（rev/s²）。
    </p>

    <h3 style="{_H2}">&#9654; 同步到达的缩放策略</h3>
    <p style="{_P}">设最大移动距离的关节为参考关节（k=1），其余关节按距离比例缩放：</p>
    <code style="{_CODE}">k&#7522; = d&#7522; / d_max          （0 &lt; k ≤ 1）

    v_limit&#7522; = v_max × k&#7522;
    a_limit&#7522; = a_max × k&#7522;</code>
    <p style="{_P}">
      速度和加速度同比例缩放后，无论梯形还是三角型曲线，
      <span style="{_EM}">所有关节总时间 T 相同</span>，即同步到达。
    </p>
    <p style="{_P}">数学证明：</p>
    <code style="{_CODE}">梯形：T&#7522; = k&#7522;×d_max/(k&#7522;×v) + (k&#7522;×v)/(k&#7522;×a) = d_max/v + v/a = T_ref ✓
    三角：T&#7522; = 2×√(k&#7522;×d_max/(k&#7522;×a)) = 2×√(d_max/a) = T_ref ✓</code>

    <h3 style="{_H2}">&#9654; CAN 批量发送</h3>
    <p style="{_P}">
      所有关节的 <code style="background:#0f0f1a;padding:2px 6px;border-radius:4px;">make_position()</code> 命令
      打包进<span style="{_EM}">一次 transport.cycle()</span> 批量发送，
      最小化关节间的时间抖动（CANFD 6帧 &lt; 1ms）。
    </p>
    ''')
            ui.button('关闭', on_click=dlg.close).style(
                'margin-top:16px; padding:8px 28px; border:none; border-radius:10px; '
                'background:#2a2a40; color:#e8e8f0; font-weight:600; cursor:pointer;')
        dlg.open()


    def show_ik_help():
        """Show a popup dialog explaining IK principles."""
        with ui.dialog() as dlg, ui.element('div').style(_HELP_STYLE):
            ui.html(f'''
    <h2 style="{_H2.replace("margin:18px 0 8px","margin:0 0 8px")}; font-size:1.3rem;">
      &#128279; Inverse Kinematics — 逆运动学原理
    </h2>

    <h3 style="{_H2}">&#9654; 正运动学 FK（Forward Kinematics）</h3>
    <p style="{_P}">
      已知关节角度 <span style="{_EM}">θ = [θ₁, θ₂, ..., θ₆]</span>，求末端位姿 <span style="{_EM}">T ∈ SE(3)</span>：
    </p>
    <code style="{_CODE}">T = T₀₁(θ₁) × T₁₂(θ₂) × ... × T₅₆(θ₆)

                    [ R₃×₃ | p₃×₁ ]
    T =             [------+------]
                    [  0  |   1  ]</code>
    <p style="{_P}">
      每个 Tᵢⱼ(θⱼ) 是由 URDF 中 &lt;origin xyz rpy&gt; 和 &lt;axis&gt; 参数与关节角 θⱼ 组合而成的
      <span style="{_EM}">4×4 齐次变换矩阵</span>。
    </p>

    <h3 style="{_H2}">&#9654; 逆运动学 IK（Inverse Kinematics）</h3>
    <p style="{_P}">
      已知目标位姿 <span style="{_EM}">T_target</span>，求使 FK(θ) = T_target 的关节角 θ。
      本系统使用 <span style="{_EM}">ikpy</span> 库，基于 scipy 数值优化求解：
    </p>
    <code style="{_CODE}">θ* = argmin ‖FK(θ) - T_target‖²
            s.t.  θ_lower ≤ θ ≤ θ_upper</code>

    <h3 style="{_H2}">&#9654; 目标矩阵构造（RPY → SE(3)）</h3>
    <p style="{_P}">输入的 X/Y/Z（mm）和 Rx/Ry/Rz（°）转换为 4×4 变换矩阵：</p>
    <code style="{_CODE}">Rx = [[1,   0,    0   ]
          [0,  cosα, -sinα]
          [0,  sinα,  cosα]]

    R = Rz × Ry × Rx         （Yaw × Pitch × Roll）

    T_target = [[R,  p],
                [0,  1]]    p = [X/1000, Y/1000, Z/1000] m</code>

    <h3 style="{_H2}">&#9654; 残差验证</h3>
    <p style="{_P}">IK 求解后用 FK 正解验证精度：</p>
    <code style="{_CODE}">T_actual = FK(θ*)
    residual = ‖p_target - p_actual‖ × 1000  [mm]</code>
    <p style="{_P}">
      残差 &lt; 1mm 为优秀，&lt; 10mm 可接受，&gt; 10mm 表示目标可能在可达空间之外。
    </p>

    <h3 style="{_H2}">&#9654; 初始猜测与收敛</h3>
    <p style="{_P}">
      IK 以<span style="{_EM}">当前关节角度（截断至 URDF 限位内）</span>作为初始猜测，
      引导优化器快速收敛到接近当前构型的解。
      勾选 \u201cConstrain orientation\u201d 将同时约束末端 6 个自由度（位置+姿态），
      否则仅约束位置（3个自由度），通常更容易求解。
    </p>

    <h3 style="{_H2}">&#9654; URDF 运动链（DummyX2）</h3>
    <p style="{_P}">6 个旋转关节，参数来自 dummyx2.xacro：</p>
    <code style="{_CODE}">Joint  轴  origin (m)                  limits (rad)
      J1   -Z  (0, 0, 0.0755)           [−3.107, +3.107]
      J2   -X  (0.014,  0.035, 0.041)   [−1.396, +2.094]
      J3   +X  (0,      0,    0.151)    [−1.571, +1.571]
      J4   +Y  (−0.013, 0.020, 0.06)    [−3.107, +3.107]
      J5   -X  (−0.028, 0.141, 0)       [−1.833, +2.094]
      J6   +Y  (0.026,  0.096, 0)       [−3.107, +3.107]</code>
    ''')
            ui.button('关闭', on_click=dlg.close).style(
                'margin-top:16px; padding:8px 28px; border:none; border-radius:10px; '
                'background:#2a2a40; color:#e8e8f0; font-weight:600; cursor:pointer;')
        dlg.open()


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
        vel = float(v.value) if v and v.value is not None else 0.05
        accel = float(a.value) if a and a.value is not None else 0.1
        return target, torque, vel, accel

    def on_enable():
        """Enter position mode (hold current position)."""
        target, torque, vel, accel = _get_ctrl_params()
        command_queue.append(('position', target, math.nan, torque, 0.0, accel))
        print(f'[UI] Enable clicked (target={target})')

    def confirm_disable():
        """弹出确认框，用户确认后才执行 Disable。"""
        with ui.dialog() as dlg, ui.card().style(
            'background: #1e1e2e; border: 1px solid #ef476f55; border-radius: 14px; '
            'min-width: 320px; padding: 24px; box-shadow: 0 8px 32px rgba(239,71,111,0.25);'
        ):
            ui.html(
                '<div style="font-size:1.15rem;font-weight:700;color:#ef476f;margin-bottom:8px;">'
                '⚠ 确认 Disable？</div>'
                '<div style="color:#aab;font-size:0.95rem;line-height:1.5;">'
                '电机将关闭力矩，<b>机械臂可能因重力下落</b>，请确保已做好安全防护。</div>'
            )
            with ui.row().style('justify-content:flex-end; gap:12px; margin-top:18px;'):
                ui.button('取消', on_click=dlg.close).style(
                    'padding:7px 20px; border-radius:9px; border:none; cursor:pointer; '
                    'background:#2a2a3e; color:#ccc; font-weight:600;')
                ui.button('确认 Disable', on_click=lambda: (dlg.close(), on_disable())).style(
                    'padding:7px 20px; border-radius:9px; border:none; cursor:pointer; '
                    'background:linear-gradient(135deg,#ef476f,#d63a5c); color:#fff; font-weight:700;')
        dlg.open()


    def on_disable():
        """Stop all motors."""
        tgt = ui_labels.get('target_dropdown')
        target = tgt.value if tgt and tgt.value is not None else 'all'
        command_queue.append(('stop', target))
        print(f'[UI] Disable clicked (target={target})')

    def _get_gripper_params():
        """Read gripper-specific slider values."""
        pos_s = ui_labels.get('gripper_pos_slider')
        torq_s = ui_labels.get('gripper_torq_slider')
        vel_s = ui_labels.get('gripper_vel_slider')
        acc_s = ui_labels.get('gripper_acc_slider')
        pos_mm = float(pos_s.value) if pos_s and pos_s.value is not None else 0.0
        pos = pos_mm / 75.1339  # mm to rev
        torque = float(torq_s.value) if torq_s and torq_s.value is not None else 0.1
        vel = float(vel_s.value) if vel_s and vel_s.value is not None else 0.05
        accel = float(acc_s.value) if acc_s and acc_s.value is not None else 0.1
        return pos, torque, vel, accel

    def on_gripper_enable():
        """Enable gripper motor (hold current position via persistent command)."""
        global gripper_cmd
        _, torque, _, accel = _get_gripper_params()
        gripper_cmd = {'pos': math.nan, 'torque': torque, 'vel': 0.0, 'accel': accel}
        st = ui_labels.get('gripper_status')
        if st:
            st.set_text(f'Enabled (hold) torque={torque:.2f}Nm')
        print(f'[UI] Gripper Enable: torque={torque} (persistent)')

    def on_gripper_disable():
        """Disable (stop) gripper motor — clears persistent command."""
        global gripper_cmd
        gripper_cmd = None
        command_queue.append(('stop', GRIPPER_ID))
        st = ui_labels.get('gripper_status')
        if st:
            st.set_text('Disabled')
        print(f'[UI] Gripper Disable (persistent cleared)')

    def on_gripper_move():
        """Move gripper to the slider position (persistent command)."""
        global gripper_cmd
        pos, torque, vel, accel = _get_gripper_params()
        gripper_cmd = {'pos': pos, 'torque': torque, 'vel': vel, 'accel': accel}
        st = ui_labels.get('gripper_status')
        if st:
            st.set_text(f'Moving → {pos*75.1339:.1f} mm')
        print(f'[UI] Gripper Move: pos={pos:.2f}rev torque={torque:.2f}Nm vel={vel:.2f} accel={accel:.2f} (persistent)')

    def on_gripper_home():
        """Move gripper to position 0 (fully open, persistent command)."""
        global gripper_cmd
        _, torque, vel, accel = _get_gripper_params()
        gripper_cmd = {'pos': 0.0, 'torque': torque, 'vel': vel, 'accel': accel}
        # Also reset the slider
        pos_s = ui_labels.get('gripper_pos_slider')
        if pos_s:
            pos_s.value = 0.0
            pos_s.update()
        st = ui_labels.get('gripper_status')
        if st:
            st.set_text('Moving → Home (0)')
        print(f'[UI] Gripper Home: torque={torque:.2f}Nm vel={vel:.2f} accel={accel:.2f} (persistent)')

    async def on_gripper_set_max_pos():
        """Set the current angle input value as the gripper's max position in moteus."""
        angle_input = ui_labels.get('gripper_angle_input')
        if not angle_input or angle_input.value is None:
            return
        pos_mm = float(angle_input.value)
        max_pos_rev = pos_mm / 75.1339
        st = ui_labels.get('gripper_status')
        if st:
            st.set_text(f'Setting max position: {pos_mm:.1f} mm ({max_pos_rev:.4f} rev)...')
        fut = asyncio.get_event_loop().create_future()
        command_queue.append(('set_max_position', GRIPPER_ID, max_pos_rev, fut))
        ok, msg = await fut
        if st:
            if ok:
                st.set_text(f'\u2713 Max pos set: {msg}')
            else:
                st.set_text(f'\u2717 Error: {msg}')
        print(f'[UI] Set Max Position: {pos_mm:.1f} mm → {max_pos_rev:.4f} rev, result: {ok}, {msg}')

    async def on_gripper_set_min_pos():
        """Set the current angle input value as the gripper's min position in moteus."""
        angle_input = ui_labels.get('gripper_angle_input')
        if not angle_input or angle_input.value is None:
            return
        pos_mm = float(angle_input.value)
        min_pos_rev = pos_mm / 75.1339
        st = ui_labels.get('gripper_status')
        if st:
            st.set_text(f'Setting min position: {pos_mm:.1f} mm ({min_pos_rev:.4f} rev)...')
        fut = asyncio.get_event_loop().create_future()
        command_queue.append(('set_min_position', GRIPPER_ID, min_pos_rev, fut))
        ok, msg = await fut
        if st:
            if ok:
                st.set_text(f'\u2713 Min pos set: {msg}')
            else:
                st.set_text(f'\u2717 Error: {msg}')
        print(f'[UI] Set Min Position: {pos_mm:.1f} mm → {min_pos_rev:.4f} rev, result: {ok}, {msg}')

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


    def on_ready():
        """Ready pose: move motor 2 to -80° and motor 3 to -90°."""
        _, torque, vel, accel = _get_ctrl_params()  # use current UI torque/vel/accel
        READY_POSES = {
            2: -78.0,   # degrees
            3: -90.0,   # degrees
        }
        for jid, deg in READY_POSES.items():
            rev = deg / 360.0
            command_queue.append(('position', jid, rev, torque, vel, accel))
            print(f'[UI] Ready: Motor {jid} → {deg:.1f}° ({rev:.4f} rev)')


    def on_fk_move():
        """Synchronized multi-joint move: scale vel & accel so all joints arrive simultaneously.

        For each joint i with distance d_i:
            k_i = d_i / d_max          (scaling factor, 0 < k_i <= 1)
            velocity_limit_i = max_vel * k_i
            accel_limit_i    = max_accel * k_i

        Both trapezoidal and triangular profiles share the same total travel time T when
        vel and accel are scaled by the same factor k, so all joints finish together.

        All 6 CAN position commands are sent in one transport.cycle() batch for
        near-simultaneous execution (suitable for <=100 Hz with 6 joints on CANFD).
        """
        _, torque, max_vel, max_accel = _get_ctrl_params()

        # Read target positions from FK panel (degrees -> revolutions)
        targets = {}
        for jid in JOINT_IDS:
            inp = ui_labels.get(f'fk_pos_{jid}')
            if inp is not None and inp.value is not None:
                targets[jid] = float(inp.value) / 360.0

        if not targets:
            return

        # Distance each joint must travel
        distances = {}
        for jid, tgt_rev in targets.items():
            curr = motor_data[jid].get('position') or 0.0
            distances[jid] = abs(tgt_rev - curr)

        max_dist = max(distances.values()) if distances else 0.0
        MIN_DIST_REV = 0.5 / 360.0  # ignore moves < 0.5 degrees

        if max_dist < MIN_DIST_REV:
            st = ui_labels.get('fk_status')
            if st:
                st.set_text('\u26a0 All joints already near target (<0.5\u00b0)')
            return

        # Build per-joint commands with proportionally scaled vel/accel
        joint_cmds = {}
        for jid, tgt_rev in targets.items():
            k = distances[jid] / max_dist if max_dist > 0 else 1.0
            v_i = max(max_vel * k, 0.001)
            a_i = max(max_accel * k, 0.001)
            joint_cmds[jid] = (tgt_rev, torque, v_i, a_i)

        command_queue.append(('sync_position', joint_cmds))

        st = ui_labels.get('fk_status')
        if st:
            parts = [f'J{j}:{v[0]*360:.0f}\u00b0(k={distances[j]/max_dist:.2f})'
                     for j, v in joint_cmds.items()]
            st.set_text('\u2192 ' + '  '.join(parts))
        print(f'[UI] FK Sync Move dispatched: {[(j, f"{v[0]*360:.1f}deg", f"vel={v[2]:.3f}", f"acc={v[3]:.3f}") for j,v in joint_cmds.items()]}')


    def on_fk_copy_current():
        """Copy live motor positions into FK target input fields."""
        for jid in JOINT_IDS:
            pos = motor_data[jid].get('position')
            if pos is not None:
                inp = ui_labels.get(f'fk_pos_{jid}')
                if inp:
                    inp.value = round(pos * 360.0, 1)
                    inp.update()
        st = ui_labels.get('fk_status')
        if st:
            st.set_text('\u2713 Copied current positions into target fields')
        print('[UI] FK Copy Current: positions loaded into FK inputs')


    def on_ik_fk_preview():
        """Compute FK from current motor positions and fill IK target inputs."""
        if not _IK_READY:
            return
        try:
            import numpy as np
            angles = [0.0] + [
                (motor_data[jid].get('position') or 0.0) * 2 * math.pi
                for jid in JOINT_IDS
            ] + [0.0]
            T = _ik_chain.forward_kinematics(angles)
            rx, ry, rz = _rpy_from_mat(T[:3, :3])
            for key, val in [
                ('ik_x', T[0, 3] * 1000), ('ik_y', T[1, 3] * 1000), ('ik_z', T[2, 3] * 1000),
                ('ik_rx', rx), ('ik_ry', ry), ('ik_rz', rz),
            ]:
                inp = ui_labels.get(key)
                if inp:
                    inp.value = round(float(val), 2)
                    inp.update()
            st = ui_labels.get('ik_status')
            if st:
                st.set_text(f'\u2713 FK: [{T[0,3]*1000:.1f}, {T[1,3]*1000:.1f}, {T[2,3]*1000:.1f}] mm')
        except Exception as e:
            st = ui_labels.get('ik_status')
            if st:
                st.set_text(f'\u2717 FK error: {e}')


    def on_ik_solve():
        """Solve IK for current target inputs and display joint angles."""
        global _ik_last_result
        if not _IK_READY:
            return
        st = ui_labels.get('ik_status')
        try:
            import numpy as np
            # Read inputs
            vals = {}
            for k in ['ik_x', 'ik_y', 'ik_z', 'ik_rx', 'ik_ry', 'ik_rz']:
                inp = ui_labels.get(k)
                vals[k] = float(inp.value) if (inp and inp.value is not None) else 0.0

            # Build 4x4 target matrix (position in metres, orientation from RPY)
            x, y, z = vals['ik_x'] / 1000.0, vals['ik_y'] / 1000.0, vals['ik_z'] / 1000.0
            cr, sr = math.cos(math.radians(vals['ik_rx'])), math.sin(math.radians(vals['ik_rx']))
            cp, sp = math.cos(math.radians(vals['ik_ry'])), math.sin(math.radians(vals['ik_ry']))
            cy, sy = math.cos(math.radians(vals['ik_rz'])), math.sin(math.radians(vals['ik_rz']))
            Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            T = np.eye(4)
            T[:3, :3] = Rz @ Ry @ Rx
            T[:3, 3] = [x, y, z]

            # Seed from current motor positions, clamped within each joint's URDF limits.
            # If the motor is outside URDF bounds (e.g. different calibration origin),
            # scipy will refuse to start — so we clamp to [lower+eps, upper-eps].
            _EPS = 1e-6
            initial = [0.0]  # OriginLink (fixed, no bounds)
            for _i, _jid in enumerate(JOINT_IDS):
                _angle = (motor_data[_jid].get('position') or 0.0) * 2 * math.pi
                _link  = _ik_chain.links[_i + 1]  # +1 for OriginLink
                if getattr(_link, 'bounds', None) is not None:
                    _lo, _hi = _link.bounds
                    if _lo is not None: _angle = max(_lo + _EPS, _angle)
                    if _hi is not None: _angle = min(_hi - _EPS, _angle)
                initial.append(_angle)
            initial.append(0.0)  # EE passive link


            orient_cb = ui_labels.get('ik_orient_cb')
            orient_mode = 'all' if (orient_cb and orient_cb.value) else None

            if st:
                st.set_text('Solving\u2026')

            result = _ik_chain.inverse_kinematics_frame(
                target=T, initial_position=initial, orientation_mode=orient_mode)

            # Verify via FK
            T_fk = _ik_chain.forward_kinematics(result)
            residual_mm = math.sqrt(sum(
                (T[i, 3] - T_fk[i, 3])**2 for i in range(3))) * 1000.0

            # Indices 1..6 are the active joints (0=base, -1=EE)
            joint_deg = [math.degrees(result[i]) for i in range(1, 7)]
            _ik_last_result = joint_deg

            # Update result boxes
            for i, jid in enumerate(JOINT_IDS):
                lbl = ui_labels.get(f'ik_result_{jid}')
                if lbl:
                    lbl.set_text(f'{joint_deg[i]:.1f}\u00b0')
                    lbl.update()

            resid_lbl = ui_labels.get('ik_residual')
            if resid_lbl:
                resid_lbl.set_text(f'Position residual: {residual_mm:.2f} mm')
                resid_lbl.update()

            ok = residual_mm < 10.0
            status_icon = '\u2713' if ok else '\u26a0'
            color = 'var(--accent-green)' if ok else 'var(--accent-amber)'
            if st:
                st.set_text(f'{status_icon} Residual: {residual_mm:.2f} mm '
                            f'| {[f"J{i+1}:{d:.1f}\u00b0" for i, d in enumerate(joint_deg)]}')
            print(f'[IK] Solved: {[f"{d:.1f}deg" for d in joint_deg]}, '
                  f'residual={residual_mm:.2f}mm')
        except Exception as e:
            if st:
                st.set_text(f'\u2717 IK error: {e}')
            print(f'[IK] Error: {e}')


    def on_ik_apply():
        """Apply last IK solution via sync_position command."""
        global _ik_last_result
        st = ui_labels.get('ik_status')
        if not _IK_READY or _ik_last_result is None:
            if st:
                st.set_text('\u26a0 No IK solution yet. Press Solve IK first.')
            return
        _, torque, max_vel, max_accel = _get_ctrl_params()
        distances, targets = {}, {}
        for i, jid in enumerate(JOINT_IDS):
            tgt_rev = _ik_last_result[i] / 360.0
            curr = motor_data[jid].get('position') or 0.0
            distances[jid] = abs(tgt_rev - curr)
            targets[jid] = tgt_rev
        max_dist = max(distances.values()) if distances else 0.0
        if max_dist < 0.5 / 360.0:
            if st:
                st.set_text('\u26a1 Already near IK target (<0.5\u00b0)')
            return
        joint_cmds = {}
        for jid in JOINT_IDS:
            k = distances[jid] / max_dist if max_dist > 0 else 1.0
            joint_cmds[jid] = (targets[jid], torque, max(max_vel * k, 0.001), max(max_accel * k, 0.001))
        command_queue.append(('sync_position', joint_cmds))
        if st:
            st.set_text(f'\u2192 Moving to IK pose: '
                        f'{[f"J{j}:{targets[j]*360:.1f}\u00b0" for j in JOINT_IDS]}')
        print(f'[IK] Apply dispatched: {len(joint_cmds)} joints')


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

    # Update each motor card (joints + gripper)
    for jid in ALL_MOTOR_IDS:
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

        
        def _get_pos_fmt(v):
            if v is None: return 'N/A'
            return f'{v*75.1339:.1f} mm ({v:.3f} rev)' if jid == 7 else fmt_pos(v)
            
        _upd('pos', _get_pos_fmt, 'value-position')
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

    # Update FK sync panel: show live current position under each joint input
    for _jfk in JOINT_IDS:
        _fk_lbl = ui_labels.get(f'fk_curr_{_jfk}')
        if _fk_lbl:
            _fk_pos = motor_data[_jfk].get('position')
            _fk_lbl.set_text(f'Now: {_fk_pos*360:.1f}\u00b0' if _fk_pos is not None else '--')

    # Populate PID inputs once after startup read completes
    if pid_read_done:
        for jid in ALL_MOTOR_IDS:
            for param in PID_PARAMS:
                inp = ui_labels.get(f'pid_{param}_{jid}')
                if inp and pid_data[jid][param] is not None and inp.value == 0.0:
                    inp.value = pid_data[jid][param]
                    inp.update()

    # Update gripper control panel: slider labels + live position
    _gps = ui_labels.get('gripper_pos_slider')
    _gpl = ui_labels.get('gripper_pos_label')
    if _gps and _gpl:
        _gv_mm = float(_gps.value) if _gps.value is not None else 0.0
        _gpl.set_text(f'{_gv_mm:.1f} mm ({_gv_mm / 75.1339:.3f} rev)')

    _gts = ui_labels.get('gripper_torq_slider')
    _gtl = ui_labels.get('gripper_torq_label')
    if _gts and _gtl:
        _gtl.set_text(f'{float(_gts.value):.2f} Nm' if _gts.value is not None else '-- Nm')

    _gvs = ui_labels.get('gripper_vel_slider')
    _gvl = ui_labels.get('gripper_vel_label')
    if _gvs and _gvl:
        _gvl.set_text(f'{float(_gvs.value):.2f} rev/s' if _gvs.value is not None else '-- rev/s')

    _gas = ui_labels.get('gripper_acc_slider')
    _gal = ui_labels.get('gripper_acc_label')
    if _gas and _gal:
        _gal.set_text(f'{float(_gas.value):.2f} rev/s²' if _gas.value is not None else '-- rev/s²')

    # Update gripper angle input with current position (degrees)
    _gai = ui_labels.get('gripper_angle_input')
    if _gai:
        _gpos = motor_data[GRIPPER_ID].get('position')
        if _gpos is not None:
            _new_mm = round(_gpos * 75.1339, 1)
            # Only auto-update if user hasn't manually edited (avoid fighting user input)
            if _gai.value is None or abs(float(_gai.value) - _new_mm) > 0.5:
                _gai.value = _new_mm
                _gai.update()


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


