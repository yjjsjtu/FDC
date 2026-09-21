#!/usr/bin/env python3

import argparse
import asyncio
import moteus

async def main():
    parser = argparse.ArgumentParser(description="Loop motor between pos +-0.8")
    moteus.make_transport_args(parser)
    parser.add_argument('--target', '-t', type=int, default=1, help='moteus ID to control')
    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)
    #c = moteus.Controller(id=args.target, transport=transport)
    c = moteus.Controller()

    await c.set_stop()
    print("Starting loop... Press Ctrl+C to stop.")

    try:
        while True:
            print("Moving to pos = 0.8 (velocity=0.2, accel=0.4)")
            await c.set_position(
                position=0.8,
                velocity_limit=0.3,
                accel_limit=0.1,
                query=False
            )
            await asyncio.sleep(8.0)

            print("Moving to pos = 0 (velocity=0.2, accel=0.4)")
            await c.set_position(
                position=0.0,
                velocity_limit=0.3,
                accel_limit=0.1,
                query=False
            )
            await asyncio.sleep(11.0)
    except asyncio.CancelledError:
        pass
    finally:
        print("\nStopping motor...")
        await c.set_stop()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

