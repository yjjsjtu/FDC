#!/usr/bin/python3 -B

"""
Rezero the dual encoder Source 1 offset.

Steps:
  0. Stop telemetry emissions (tel stop) to avoid flooding.
  1. Set motor_position.sources.1.offset to 0 (clear existing offset).
  2. Read motor_position.sources.1.sign to determine offset polarity.
  3. Read motor_position telemetry → sources[1].filtered_value.
  4. Compute offset based on sign, write it back to config.
  5. conf write to persist.
  6. Verify: read motor_position telemetry → sources[1].offset_value ≈ 0.
"""

import argparse
import asyncio
import moteus
import sys


COMMAND_TIMEOUT_S = 3.0
MAX_RETRIES = 3


async def robust_command(stream, data, timeout=COMMAND_TIMEOUT_S,
                         retries=MAX_RETRIES,
                         allow_any_response=False):
    """Send a diagnostic stream command with timeout and retry.

    For 'conf get' commands, set allow_any_response=True because the
    device returns just the value without an 'OK' terminator.
    For 'conf set' / 'conf write', leave it False (wait for 'OK').
    """
    for attempt in range(1, retries + 1):
        try:
            return await asyncio.wait_for(
                stream.command(data, allow_any_response=allow_any_response),
                timeout)
        except asyncio.TimeoutError:
            if attempt < retries:
                print(f"  ⏳ Timeout on attempt {attempt}/{retries}, retrying ...")
                await stream.flush_read()
            else:
                print(f"  ❌ Command timed out after {retries} attempts.")
                raise


async def main():
    parser = argparse.ArgumentParser(
        description="Rezero dual-encoder Source 1 offset on a Moteus controller.")
    parser.add_argument('--target', '-t', type=int, default=1,
                        help="Moteus target ID (default: 1)")
    parser.add_argument('--verbose', '-v', action='store_true',
                        help="Verbose output")
    parser.add_argument('--disable-brs', action='store_true',
                        help="Disable CAN FD Bit Rate Switching (run at 1 Mbps only)")

    moteus.make_transport_args(parser)

    args = parser.parse_args()

    if args.disable_brs:
        args.can_disable_brs = True

    transport = moteus.get_singleton_transport(args)
    m = moteus.Controller(id=args.target, transport=transport)
    s = moteus.Stream(m, verbose=args.verbose)

    brs_status = "DISABLED (1 Mbps only)" if args.disable_brs else "ENABLED (5 Mbps data)"
    print(f"Connecting to Moteus (ID: {args.target}), BRS: {brs_status} ...")

    # ── Step 0: Stop telemetry and flush ─────────────────────────
    print("\n[Step 0] Stopping telemetry emissions ...")
    await s.write_message(b"tel stop")
    await asyncio.sleep(0.5)
    await s.flush_read()

    await m.set_stop()
    await asyncio.sleep(0.2)

    # ── Step 1: Clear Source 1 offset ────────────────────────────
    print("[Step 1] Setting motor_position.sources.1.offset to 0 ...")
    await robust_command(s, b"conf set motor_position.sources.1.offset 0")
    await asyncio.sleep(0.3)

    # ── Step 2: Read Source 1 sign ───────────────────────────────
    print("[Step 2] Reading motor_position.sources.1.sign ...")
    sign_str = await robust_command(
        s, b"conf get motor_position.sources.1.sign",
        allow_any_response=True)
    sign_value = int(sign_str.decode().strip())
    print(f"         Source 1 sign = {sign_value}")

    # ── Step 3: Read filtered_value from telemetry ───────────────
    print("[Step 3] Reading motor_position telemetry (sources[1].filtered_value) ...")
    mp_data = await s.read_data("motor_position")
    filtered_value = mp_data.sources[1].filtered_value
    print(f"         sources[1].filtered_value = {filtered_value:.2f}")

    # ── Step 4: Compute offset based on sign and write ───────────
    # sign < 0  =>  offset = +|filtered_value|  (positive)
    # sign > 0  =>  offset = -|filtered_value|  (negative)
    if sign_value < 0:
        new_offset = abs(filtered_value)
    else:
        new_offset = -abs(filtered_value)

    # offset is an integer in the config (uint32-like count)
    new_offset_int = int(round(new_offset))

    print(f"\n[Step 4] Computed offset = {new_offset_int}  "
          f"(sign={sign_value}, filtered_value={filtered_value:.2f})")
    await robust_command(
        s,
        f"conf set motor_position.sources.1.offset {new_offset_int}"
        .encode('utf8'))

    # ── Step 5: Persist with conf write ──────────────────────────
    print("[Step 5] Saving configuration (conf write) ...")
    await robust_command(s, b"conf write")
    await asyncio.sleep(0.5)

    # ── Step 6: Verify via telemetry offset_value ────────────────
    print("\n[Step 6] Verifying (reading sources[1].offset_value) ...")
    mp_verify = await s.read_data("motor_position")
    offset_val = mp_verify.sources[1].offset_value
    filtered_val = mp_verify.sources[1].filtered_value

    # Also read back the config offset for display
    offset_cfg = await robust_command(
        s, b"conf get motor_position.sources.1.offset",
        allow_any_response=True)
    cpr_str = await robust_command(
        s, b"conf get motor_position.sources.1.cpr",
        allow_any_response=True)
    cpr = int(cpr_str.decode().strip())

    print(f"         Saved config offset            = {offset_cfg.decode().strip()}")
    print(f"         sources[1].offset_value         = {offset_val}")
    print(f"         sources[1].filtered_value        = {filtered_val:.2f}")
    print(f"         sources[1].cpr                   = {cpr}")

    # offset_value is a uint32 count in [0, cpr).  Near zero means
    # either very small or very close to cpr.
    wrapped = offset_val if offset_val <= cpr // 2 else cpr - offset_val
    threshold = cpr * 0.0005  # within 1% of a full revolution

    if wrapped < threshold:
        print(f"\n✅ Success! offset_value ({offset_val}) is near 0 "
              f"(wrapped distance = {wrapped}, threshold = {threshold:.0f}). "
              "Rezero completed. You can turn off power of motor then turn it back on to use it now!!!")
    else:
        print(f"\n⚠️  Warning: offset_value ({offset_val}) is NOT near 0 "
              f"(wrapped distance = {wrapped}, threshold = {threshold:.0f}). "
              "Please inspect the setup.")


if __name__ == '__main__':
    asyncio.run(main())

