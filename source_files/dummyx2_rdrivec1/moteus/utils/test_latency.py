#!/usr/bin/env python3
import asyncio
import time
import argparse
import moteus

async def main():
    parser = argparse.ArgumentParser(description="Test CAN FD latency for Moteus")
    parser.add_argument('--id', type=int, default=1, help='CAN ID of the motor to test (default: 1)')
    parser.add_argument('--count', type=int, default=10, help='Number of samples to collect (default: 10)')
    args = parser.parse_args()

    print(f"Testing CAN FD latency for Motor ID {args.id} with {args.count} samples...\n")
    
    # Configure the query resolution to explicitly ask for voltage
    qr = moteus.QueryResolution()
    qr.voltage = moteus.INT8
    
    # Initialize the controller
    c = moteus.Controller(id=args.id, query_resolution=qr)
    
    # Optional: Clear any existing faults before testing
    await c.set_stop()
    
    latencies = []
    
    for i in range(args.count):
        start_time = time.perf_counter()
        
        # Requesting a state update which includes the queried registers (voltage)
        state = await c.query()
        
        end_time = time.perf_counter()
        
        # Calculate latency in milliseconds
        latency_ms = (end_time - start_time) * 1000.0
        latencies.append(latency_ms)
        
        # Extract voltage
        voltage = state.values.get(moteus.Register.VOLTAGE, None)
        v_str = f"{voltage:.2f} V" if voltage is not None else "Unknown"
        
        print(f"[{i+1:02d}/{args.count}] Voltage: {v_str:>8}, Latency: {latency_ms:6.2f} ms")
        
        # Small delay between queries to avoid flooding
        await asyncio.sleep(0.01)
        
    if latencies:
        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)
        min_latency = min(latencies)
        
        print("\n--- Latency Summary ---")
        print(f"Total Samples: {args.count}")
        print(f"Average:       {avg_latency:.3f} ms")
        print(f"Minimum:       {min_latency:.3f} ms")
        print(f"Maximum:       {max_latency:.3f} ms")
    else:
        print("\nNo samples collected.")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
