#!/usr/bin/env python3

"""
WebSocket Cient - forwards Raw IMU messages via Websocket to a computer
running Motioncal

Usage:
    python ws_streamer.py [--host HOST] [--port PORT] [--device DEVICE]

Where:
    Host:   Hostname or IP address of the machine running 
            both WebSocket server and MotionCal.
    Port:   Port on which the server is listening 
            default 8080).
    Device: Name of the local device on which the ESP IMU is installed
            default: /dev/serial0.
"""
from collections import deque
import asyncio
import argparse
import time
import websockets
from pyrover import PyRover, RoverCallbacks

duration = 0.02

async def main(host: str, port: int, device: str):
    queue = deque(maxlen=1000)
    last_time = time.time()
    def raw_cb(message):
        queue.append(message)

    
    cb = RoverCallbacks(on_raw=raw_cb)
    rover = PyRover(device, 115200, callbacks=cb, auto_connect=True)
    uri = f"ws://{host}:{port}/ws"
    print(f"Connecting to {uri}...")
    with PyRover('/dev/serial0') as rover:
        rover.stream_on()
        rover.set_format_raw()
        print("Rover connected")
        try:
            async with websockets.connect(uri) as ws:
                print(f"Connected!")         
                counter = 0
                start_time = time.time()              
                while time.time() - last_time > duration:
                    last_time = time.time()
                    if queue:
                        await ws.send(queue.pop())
                    counter += 1               
                    # Print progress every second
                    if counter % 50 == 0:
                        elapsed = time.time() - start_time
                        rate = counter / elapsed
                        print(f"  Sent: {counter}, Rate: {rate:.1f} msg/s")                
                elapsed = time.time() - start_time
                print(f"\nDone! Sent {counter} messages in {elapsed:.2f}s ({counter/elapsed:.1f} msg/s)")
            
        except ConnectionRefusedError:
            print(f"Error: Could not connect to {uri}")
            print("Is the server running?")
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.10.64")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--device", type=str, default="/dev/serial0")
    args = parser.parse_args()
    
    asyncio.run(main(args.host, args.port, args.device))
