#!/usr/bin/env python3
"""
Simple Serial Forwarder - Client (Optimized)
"""
import socket
import argparse
import time
from collections import deque
from pyrover import PyRover, RoverCallbacks

items = deque(maxlen=100)

def raw_cb(message: str):
    if message.startswith("Raw"):
        items.append(message)

def main(host: str, port: int, device: str):
    # Connect to rover
    print(f"Opening {device}...")
    cb = RoverCallbacks(on_raw=raw_cb, on_orientation=raw_cb)
    rover = PyRover(device, 115200, auto_connect=True, callbacks=cb)
    time.sleep(2)
    
    rover.calib_off()
    rover.stream_on()
    rover.set_format_raw()
    print("Rover ready")
    
    # Connect to server with optimizations
    print(f"Connecting to {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # 64KB send buffer
    sock.settimeout(0.05)  # 50ms timeout
    sock.connect((host, port))
    print("Connected!")
    
    counter = 0
    dropped = 0
    start_time = time.time()
    
    try:
        while True:
            # Read from rover (non-blocking check)
            if items:
                line = items.popleft()
                try:
                    # Try to send immediately
                    sock.sendall((line + '\n').encode('utf-8'))
                    counter += 1
                    
                    if counter % 500 == 0:
                        elapsed = time.time() - start_time
                        rate = counter / elapsed
                        print(f"Sent: {counter}, Rate: {rate:.1f} msg/s, Dropped: {dropped}")
                        
                except socket.timeout:
                    # Can't send fast enough - drop this frame
                    dropped += 1
                    if dropped % 100 == 0:
                        print(f"Warning: Network slow, dropped {dropped} frames")
                    
            else:
                # No data available, tiny sleep to avoid busy loop
                time.sleep(0.001)
                
    except KeyboardInterrupt:
        print("\nStopping...")
        elapsed = time.time() - start_time
        print(f"Total sent: {counter}, Rate: {counter/elapsed:.1f} msg/s, Dropped: {dropped}")
    finally:
        sock.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.10.64")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--device", default="/dev/serial0")
    args = parser.parse_args()
    
    main(args.host, args.port, args.device)