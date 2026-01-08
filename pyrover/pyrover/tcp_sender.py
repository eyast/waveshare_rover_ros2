#!/usr/bin/env python3
"""
Simple Serial Forwarder - Client
Reads from local serial and sends to remote computer via TCP
"""
from collections import deque
import socket
import argparse
import time
from pyrover import PyRover, RoverCallbacks

items = deque(maxlen=10000)

def raw_cb(message: str):
    if message.startswith("Raw"):
        items.append(message)

def main(host: str, port: int, device: str):
    # Connect to rover
    print(f"Opening {device}...")
    cb = RoverCallbacks(on_raw=raw_cb)
    rover = PyRover(device, 115200, auto_connect=True, callbacks=cb)
    rover.calib_off()
    time.sleep(2)
    rover.stream_on()
    rover.set_format_raw()
    print("Rover ready")
    
    # Connect to server
    print(f"Connecting to {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("Connected!")
    time.sleep(2)
    counter = 0
    try:
        while True:
            # Read a line from serial
            if items:
                line = items.popleft()
                # Send it over TCP
                sock.sendall(line)
                counter += 1
                if counter % 500 == 0:
                    print(f"Sent {counter} messages")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        sock.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.10.64")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--device", default="/dev/serial0")
    args = parser.parse_args()
    
    main(args.host, args.port, args.device)