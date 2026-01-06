#!/usr/bin/env python3
"""
Virtual Serial Port Reader

Utility to test reading from the virtual serial port created by the server.
Run this after starting the server to see incoming messages.

Usage:
    python reader.py <device_path> [--baud BAUD]

Example:
    python reader.py /dev/ttys001 --baud 115200
"""

import argparse
import sys
import signal

def main():
    parser = argparse.ArgumentParser(
        description="Read from a serial port (real or virtual)"
    )
    parser.add_argument(
        "--device",
        default='/dev/ttys214',
        help="Serial device path (e.g., /dev/ttys001)"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--rate-window",
        type=int,
        default=1,
        help="Seconds to average rate over (default: 1)"
    )
    
    args = parser.parse_args()
    
    try:
        import serial
    except ImportError:
        print("Error: pyserial not installed. Run: pip install pyserial")
        sys.exit(1)
    
    # Graceful shutdown
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        running = False
        print("\nShutting down...")
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print(f"Opening {args.device} at {args.baud} baud...")
    
    try:
        ser = serial.Serial(
            args.device,
            args.baud,
            timeout=0.1  # Non-blocking with short timeout
        )
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    
    print(f"Connected! Reading messages...\n")
    
    # Statistics
    import time
    msg_count = 0
    start_time = time.time()
    last_print_time = start_time
    
    try:
        while running:
            line = ser.readline()
            if line:
                msg_count += 1
                decoded = line.decode('utf-8', errors='replace').strip()
                
                # Calculate rate every second
                now = time.time()
                if now - last_print_time >= args.rate_window:
                    elapsed = now - start_time
                    rate = msg_count / elapsed if elapsed > 0 else 0
                    print(f"[{msg_count:>8}] {decoded:<15} Rate: {rate:.1f} msg/s")
                    last_print_time = now
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()
        total_time = time.time() - start_time
        avg_rate = msg_count / total_time if total_time > 0 else 0
        print(f"\nTotal: {msg_count} messages in {total_time:.1f}s ({avg_rate:.1f} msg/s)")


if __name__ == "__main__":
    main()
