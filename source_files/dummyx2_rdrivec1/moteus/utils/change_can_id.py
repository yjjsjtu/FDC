#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Moteus CAN ID Change Tool
==========================
Interactive tool to view and change the CAN ID of moteus controllers on the bus.
Uses the same transport layer as webgui.py (auto-detects fdcanusb, gs_usb, socketcan).

Run from the project root directory:
    cd /path/to/moteus_2
    python utils/change_can_id.py

Usage:
    # Auto-detect transport (macOS fdcanusb / gs_usb)
    python utils/change_can_id.py

    # Force socketcan on Linux
    python utils/change_can_id.py --force-transport pythoncan --can-iface socketcan --can-chan can0

    # Specify CAN prefix
    python utils/change_can_id.py --can-prefix 0

    # Non-interactive: change ID 1 to ID 5, auto-save to flash
    python utils/change_can_id.py --id 1 --new-id 5 --auto-save
"""

import argparse
import asyncio
import sys
import time

sys.path.insert(0, './lib/python')
import moteus


def make_parser():
    parser = argparse.ArgumentParser(
        description='Moteus CAN ID Change Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python utils/change_can_id.py
  python utils/change_can_id.py --can-prefix 0
  python utils/change_can_id.py --force-transport pythoncan --can-iface socketcan --can-chan can0
  python utils/change_can_id.py --id 1 --new-id 5 --auto-save
        """,
    )
    parser.add_argument('--can-prefix', type=lambda x: int(x, 0), default=0,
                        help='CAN prefix (default: 0)')
    parser.add_argument('--id', type=int, default=None,
                        help='Target current CAN ID (skip discovery)')
    parser.add_argument('--new-id', type=int, default=None,
                        help='New CAN ID (non-interactive mode)')
    parser.add_argument('--auto-save', action='store_true',
                        help='Auto-save to flash without prompting')
    moteus.make_transport_args(parser)
    return parser


async def discover_devices(transport, can_prefix, timeout=1.0):
    """Discover all moteus controllers on the bus."""
    print(f'[INFO] Discovering devices on CAN bus (prefix=0x{can_prefix:04X})...')
    try:
        devices = await asyncio.wait_for(
            transport.discover(can_prefix=can_prefix),
            timeout=timeout,
        )
        return devices
    except asyncio.TimeoutError:
        print('[WARN] Discovery timed out. No devices found.')
        return []


async def get_device_info(transport, can_id, can_prefix, timeout=1.0):
    """Get extended info for a single device via query."""
    qr = moteus.QueryResolution()
    qr.mode = moteus.INT8
    qr.fault = moteus.INT8
    qr.voltage = moteus.INT8
    qr.temperature = moteus.INT8

    ctrl = moteus.Controller(
        id=can_id,
        transport=transport,
        query_resolution=qr,
    )

    try:
        results = await asyncio.wait_for(
            transport.cycle([ctrl.make_query()]),
            timeout=timeout,
        )
        for result in results:
            servo_id = (result.arbitration_id >> 8) & 0x7F
            if servo_id == can_id:
                vals = result.values
                return {
                    'mode': vals.get(moteus.Register.MODE, None),
                    'fault': vals.get(moteus.Register.FAULT, None),
                    'voltage': vals.get(moteus.Register.VOLTAGE, None),
                    'temperature': vals.get(moteus.Register.TEMPERATURE, None),
                }
    except asyncio.TimeoutError:
        pass
    return None


async def read_current_id(transport, can_id, timeout=2.0):
    """Read the current CAN ID from a controller via diagnostic stream."""
    ctrl = moteus.Controller(id=can_id, transport=transport)
    s = moteus.Stream(ctrl)

    try:
        await s.write_message(b'tel stop')
        await asyncio.sleep(0.1)
        await s.flush_read()

        raw = await asyncio.wait_for(
            s.command(b'conf get id.id', allow_any_response=True),
            timeout=timeout,
        )
        result = raw.decode('utf8').strip()
        return int(result) if result else None
    except asyncio.TimeoutError:
        return None
    except Exception as e:
        print(f'  [WARN] Failed to read id.id: {e}')
        return None


async def set_can_id(transport, old_id, new_id, save_to_flash, timeout=2.0):
    """Change CAN ID of a controller and optionally save to flash."""
    ctrl = moteus.Controller(id=old_id, transport=transport)
    s = moteus.Stream(ctrl)

    await s.write_message(b'tel stop')
    await asyncio.sleep(0.1)
    await s.flush_read()

    await asyncio.wait_for(
        s.command(f'conf set id.id {new_id}'.encode('utf8')),
        timeout=timeout,
    )

    if save_to_flash:
        await asyncio.wait_for(
            s.command(b'conf write'),
            timeout=timeout,
        )
        return True, f'CAN ID changed from {old_id} to {new_id} and saved to flash.'
    else:
        return True, f'CAN ID changed from {old_id} to {new_id} (RAM only, not saved to flash).'


async def verify_can_id(transport, old_id, new_id, timeout=2.0):
    """Verify the CAN ID was changed by reading it back."""
    returned_id = await read_current_id(transport, new_id, timeout)
    if returned_id == new_id:
        return True
    # Also try old ID in case the set didn't take
    returned_id_old = await read_current_id(transport, old_id, timeout)
    if returned_id_old == old_id:
        return False
    return returned_id == new_id


def print_device_table(devices, device_infos):
    """Print a formatted table of discovered devices."""
    print()
    print('=' * 72)
    print(f'{"CAN ID":>8}  {"UUID":>36}  {"Mode":>12}  {"Fault":>6}  {"Bus V":>6}')
    print('-' * 72)
    for dev in devices:
        can_id = dev.can_id
        uuid_short = dev.uuid[:8].hex() if dev.uuid else 'N/A'
        info = device_infos.get(can_id, {}) if device_infos else {}

        mode_map = {
            0: 'Stopped', 1: 'Fault', 5: 'PWM', 6: 'Voltage',
            7: 'VoltFOC', 8: 'VoltDQ', 9: 'Current', 10: 'Position',
            11: 'Timeout', 12: 'ZeroVel', 13: 'StayWithin',
            14: 'MeasInd', 15: 'Brake',
        }
        mode_val = info.get('mode')
        mode_str = mode_map.get(mode_val, str(mode_val)) if mode_val is not None else '?'
        fault_val = info.get('fault')
        fault_str = str(fault_val) if fault_val is not None else '?'
        voltage_val = info.get('voltage')
        voltage_str = f'{voltage_val:.1f}V' if voltage_val is not None else '?'

        print(f'  {can_id:4d}    {uuid_short}  {mode_str:>12}  {fault_str:>6}  {voltage_str:>6}')
    print('=' * 72)
    print()


async def interactive_mode(transport, args):
    """Interactive mode: discover, display, ask user to change ID."""
    can_prefix = args.can_prefix

    # Step 1: Discover devices
    devices = await discover_devices(transport, can_prefix)
    if not devices:
        print('[ERROR] No moteus controllers found on the bus.')
        print('        Check CAN connection, power, and bus termination.')
        return

    # Sort by CAN ID
    devices.sort(key=lambda d: d.can_id)

    # Step 2: Get quick status for each device
    device_infos = {}
    for dev in devices:
        info = await get_device_info(transport, dev.can_id, can_prefix)
        if info:
            device_infos[dev.can_id] = info

    # Step 3: Print table
    print_device_table(devices, device_infos)
    print(f'Found {len(devices)} controller(s) on the bus.\n')

    # Step 4: Ask which device to change
    while True:
        try:
            choice = input('Enter CAN ID to change (or "q" to quit): ').strip()
            if choice.lower() == 'q':
                print('Exiting.')
                return
            target_id = int(choice)
            if target_id not in [d.can_id for d in devices]:
                print(f'  [ERROR] CAN ID {target_id} not found on bus.')
                continue
            break
        except ValueError:
            print('  [ERROR] Please enter a valid number.')
        except (EOFError, KeyboardInterrupt):
            print('\nExiting.')
            return

    # Step 5: Read and confirm current ID via diagnostic stream
    print(f'\n[INFO] Reading current CAN ID from device {target_id}...')
    confirmed_id = await read_current_id(transport, target_id)
    if confirmed_id is not None:
        print(f'  Current CAN ID (id.id): {confirmed_id}')
    else:
        print(f'  [WARN] Could not read id.id from device {target_id}, proceeding anyway.')

    # Step 6: Ask for new ID
    while True:
        try:
            new_id_str = input(f'Enter new CAN ID for device {target_id} (1-127): ').strip()
            new_id = int(new_id_str)
            if new_id < 1 or new_id > 127:
                print('  [ERROR] CAN ID must be between 1 and 127.')
                continue
            if new_id in [d.can_id for d in devices] and new_id != target_id:
                print(f'  [WARN] CAN ID {new_id} is already in use on the bus!')
                confirm = input('  Continue anyway? (y/N): ').strip().lower()
                if confirm != 'y':
                    continue
            break
        except ValueError:
            print('  [ERROR] Please enter a valid number.')
        except (EOFError, KeyboardInterrupt):
            print('\nExiting.')
            return

    # Step 7: Ask about saving to flash
    if args.auto_save:
        save_to_flash = True
        print('  Auto-save to flash: YES')
    else:
        while True:
            try:
                save_choice = input('Save to flash? (y/N): ').strip().lower()
                if save_choice in ('y', 'yes'):
                    save_to_flash = True
                    break
                elif save_choice in ('n', 'no', ''):
                    save_to_flash = False
                    break
                else:
                    print('  Please enter y or n.')
            except (EOFError, KeyboardInterrupt):
                print('\nExiting.')
                return

    # Step 8: Confirm and execute
    print()
    print('-' * 50)
    print(f'  Summary:')
    print(f'    Target device: CAN ID {target_id}')
    print(f'    New CAN ID:    {new_id}')
    print(f'    Save to flash: {"YES" if save_to_flash else "NO (RAM only)"}')
    print('-' * 50)

    try:
        confirm = input('Proceed with change? (y/N): ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        print('\nExiting.')
        return

    if confirm not in ('y', 'yes'):
        print('Cancelled.')
        return

    print(f'\n[INFO] Changing CAN ID from {target_id} to {new_id}...')
    try:
        success, msg = await set_can_id(transport, target_id, new_id, save_to_flash)
        if success:
            print(f'  [OK] {msg}')

            # Verify
            print(f'[INFO] Verifying new CAN ID...')
            await asyncio.sleep(0.2)
            verified = await verify_can_id(transport, target_id, new_id)
            if verified:
                print(f'  [OK] CAN ID verified: {new_id}')
                if save_to_flash:
                    print()
                    print('  *** IMPORTANT: Power-cycle the controller for the change to take effect ***')
                    print('  *** After power-cycle, the device will respond at CAN ID {} ***'.format(new_id))
                else:
                    print()
                    print('  *** The ID change is temporary and will be lost after power-cycle ***')
            else:
                print(f'  [WARN] Verification failed. Device may still be at {target_id}.')
        else:
            print(f'  [ERROR] {msg}')
    except asyncio.TimeoutError:
        print(f'  [ERROR] Timeout communicating with device {target_id}')
    except Exception as e:
        print(f'  [ERROR] {e}')


async def non_interactive_mode(transport, args):
    """Non-interactive mode: change ID directly from CLI args."""
    target_id = args.id
    new_id = args.new_id
    can_prefix = args.can_prefix
    save_to_flash = args.auto_save

    if target_id is None or new_id is None:
        print('[ERROR] --id and --new-id are required in non-interactive mode.')
        return

    print(f'[INFO] Reading current CAN ID from device {target_id}...')
    confirmed_id = await read_current_id(transport, target_id)
    if confirmed_id is not None:
        print(f'  Current CAN ID (id.id): {confirmed_id}')
    else:
        print(f'  [WARN] Could not read id.id from device {target_id}.')

    print(f'[INFO] Changing CAN ID from {target_id} to {new_id}...')
    if save_to_flash:
        print('  Save to flash: YES')

    try:
        success, msg = await set_can_id(transport, target_id, new_id, save_to_flash)
        if success:
            print(f'[OK] {msg}')
            await asyncio.sleep(0.2)
            verified = await verify_can_id(transport, target_id, new_id)
            if verified:
                print(f'[OK] CAN ID verified: {new_id}')
                if save_to_flash:
                    print('*** Power-cycle the controller for the change to take effect ***')
            else:
                print(f'[WARN] Verification failed.')
        else:
            print(f'[ERROR] {msg}')
    except Exception as e:
        print(f'[ERROR] {e}')


async def main():
    parser = make_parser()
    args = parser.parse_args()

    # Initialize transport (same as webgui.py)
    print('[INFO] Initializing CAN transport...')
    try:
        transport = moteus.get_singleton_transport(args)
    except Exception as e:
        print(f'[ERROR] Failed to initialize CAN transport: {e}')
        print('  Make sure CAN interface is up:')
        print('    sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on')
        return

    print('[INFO] CAN transport ready.\n')

    # Check if we have enough args for non-interactive mode
    if args.id is not None and args.new_id is not None:
        await non_interactive_mode(transport, args)
    else:
        await interactive_mode(transport, args)


if __name__ == '__main__':
    asyncio.run(main())

