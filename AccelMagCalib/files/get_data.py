#!/usr/bin/env python3
"""Simple serial read/write script with single keypress sampling"""

import serial
import threading
import time
import sys
import tty
import termios

output_fn = 'accelerator_samples.txt'

PORT = '/dev/cu.usbserial-110'  
BAUDRATE = 115200

samples = []
cntr = 0
collect_next = False
running = True

def filter_data(data: str):
    global cntr, collect_next
    
    if data[:3] == 'Raw' and collect_next:
        try:
            values = data.split(":")[1].split(",")[0:3]
            values = [float(x)/8192 for x in values]
            samples.append(tuple(values))
            cntr += 1
            collect_next = False
            print(f"\r{cntr} samples collected (press 'a' for next, 'q' to quit)    ", end='', flush=True)
        except:
            pass

# Connect
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)  # Short timeout
    print(f"Connected to {PORT}")
    time.sleep(0.5)
    
    ser.write(b"STREAM:ON\n")
    print("Sent: STREAM:ON")
    time.sleep(0.2)
    ser.reset_input_buffer()
    print("Ready to receive data...")
    
except serial.SerialException as e:
    print(f"Error: Could not open {PORT}")
    print(f"Details: {e}")
    exit(1)

# Read thread - SIMPLIFIED
def read_serial():
    while running:
        try:
            # Just try to read, don't check in_waiting
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                filter_data(data)
        except Exception as e:
            pass
        time.sleep(0.001)  # Small delay to prevent CPU spinning

# Start reading in background
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

def tuples_to_strings(data):
    return [tuple(str(x) for x in row) for row in data]

# Get single character without Enter
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

print("\n" + "="*60)
print("Press 'a' to capture next sample")
print("Press 'q' to quit and save")
print("="*60)
print("Waiting for keypress...\n")

try:
    while running:
        ch = getch()
        if ch == 'a':
            collect_next = True
            print(f"\rCapturing next sample...                    ", end='', flush=True)
        elif ch == 'q':
            running = False
            break
except KeyboardInterrupt:
    running = False

# Cleanup
print("\n\nSaving data...")
time.sleep(0.2)  # Let thread finish
ser.close()

result = tuples_to_strings(samples)
with open(output_fn, 'w') as f:
    f.write('\n'.join('\t'.join(row) for row in result))

print(f"Saved {len(samples)} samples to {output_fn}")
print("Closed")