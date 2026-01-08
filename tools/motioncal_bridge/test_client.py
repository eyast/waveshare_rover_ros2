#!/usr/bin/env python3
"""
WebSocket Test Client - Simulates ESP32 sending 50 messages/second

Usage:
    python test_client.py [--host HOST] [--port PORT] [--duration SECONDS]
"""

import asyncio
import argparse
import time

async def main(host: str, port: int, duration: float):
    try:
        import websockets
    except ImportError:
        print("Error: websockets not installed. Run: pip install websockets")
        return
    
    uri = f"ws://{host}:{port}/ws"
    print(f"Connecting to {uri}...")
    
    try:
        async with websockets.connect(uri) as ws:
            print(f"Connected! Sending for {duration} seconds at 50 Hz...")
            
            counter = 0
            start_time = time.time()
            interval = 0.02  # 20ms = 50 Hz
            
            while time.time() - start_time < duration:
                await ws.send(str(counter))
                counter += 1
                
                # Print progress every second
                if counter % 50 == 0:
                    elapsed = time.time() - start_time
                    rate = counter / elapsed
                    print(f"  Sent: {counter}, Rate: {rate:.1f} msg/s")
                
                # Maintain 50 Hz timing
                next_time = start_time + (counter * interval)
                sleep_time = next_time - time.time()
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
            
            elapsed = time.time() - start_time
            print(f"\nDone! Sent {counter} messages in {elapsed:.2f}s ({counter/elapsed:.1f} msg/s)")
            
    except ConnectionRefusedError:
        print(f"Error: Could not connect to {uri}")
        print("Is the server running?")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds to run")
    args = parser.parse_args()
    
    asyncio.run(main(args.host, args.port, args.duration))
