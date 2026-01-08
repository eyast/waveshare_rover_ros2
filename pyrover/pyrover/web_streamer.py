#!/usr/bin/env python3

"""
HTTPS Client - forwards Raw IMU messages via HTTPS POST to a computer
running Motioncal

Usage:
    python https_streamer.py [--host HOST] [--port PORT] [--device DEVICE]

Where:
    Host:   Hostname or IP address of the machine running 
            the HTTP server and MotionCal.
    Port:   Port on which the server is listening 
            (default 8080).
    Device: Name of the local device on which the ESP IMU is installed
            (default: /dev/serial0).
"""
from collections import deque
import asyncio
import argparse
import time
import random
import httpx
from pyrover import PyRover, RoverCallbacks

duration = 0.02


async def main(host: str, port: int, device: str):
    firstboot = True
    items = deque(maxlen=10000)
    last_time = time.time()
    
    def raw_cb(message: str):
        if message.startswith("Raw"):
            items.append(message)

    cb = RoverCallbacks(on_raw=raw_cb)
    rover = PyRover(device, 115200, callbacks=cb, auto_connect=True)
    url = f"http://{host}:{port}/data"
    print(f"Connecting to {url}...")
    
    with PyRover('/dev/serial0', auto_connect=True) as rover:
        if firstboot == True:
            rover.calib_off()
            firstboot = False
        time.sleep(5)
        rover.stream_on()
        rover.set_format_raw()
        print("Rover connected")
        
        try:
            # Create HTTP client with connection pooling
            async with httpx.AsyncClient(
                timeout=httpx.Timeout(1.0),
                limits=httpx.Limits(max_keepalive_connections=5, max_connections=10)
            ) as client:
                print(f"Connected!")
                counter = 0
                start_time = time.time()
                
                while True:
                    last_time = time.time()
                    if items:
                        item = items.popleft()
                        try:
                            # Send as POST request
                            response = await client.post(
                                url,
                                content=item,
                                headers={"Content-Type": "text/plain"}
                            )
                            counter += 1
                            if response.status_code != 200:
                                print(f"Warning: Server returned {response.status_code}")
                        except httpx.RequestError as e:
                            print(f"Request failed: {e}")
                            continue
                    # Print progress every second
                    if counter % 1000 == 0:
                        elapsed = time.time() - start_time
                        rate = counter / elapsed
                        print(f"  Sent: {counter}, Rate: {rate:.1f} msg/s")
                
                elapsed = time.time() - start_time
                print(f"\nDone! Sent {counter} messages in {elapsed:.2f}s ({counter/elapsed:.1f} msg/s)")
            
        except httpx.ConnectError:
            print(f"Error: Could not connect to {url}")
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