#!/usr/bin/env python3
"""Simple serial read/write script"""

import serial
import threading
import time

output_fn = 'accelerator_samples.txt'

PORT = '/dev/cu.usbserial-110'  
BAUDRATE = 115200

samples = []
cntr = 0
collecting = False  # Only collect after STREAM:OFF sent

def filter_data(data: str):
    global cntr
    if not collecting:
        return
    if data[:3] == 'Raw': 
            try:
                values = data.split(":")[1].split(",")[0:3]
                values = [float(x)/8192 for x in values]
                samples.append(tuple(values))
                cntr += 1
                if cntr % 10 == 0:
                    print(f"\r{cntr} data points collected", end='', flush=True)
            except:
                pass  # Silently ignore parse errors

# Connect
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"Connected to {PORT}")
    time.sleep(0.5)  # Wait for connection to stabilizea
    
    # Send STREAM:OFF immediately
    ser.write(b"STREAM:OFF\n")
    print("Sent: STREAM:OFF")
    time.sleep(0.1)
    ser.reset_input_buffer()  # Clear any buffered data
    collecting = True  # Now start collecting
    
except serial.SerialException as e:
    print(f"Error: Could not open {PORT}")
    print(f"Details: {e}")
    exit(1)

# Read thread
def read_serial():
    while True:
        try:
            if ser.in_waiting:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    filter_data(data)
        except:
            pass

# Start reading in background
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# Send commands
map = {
    "a": "STREAM:ON",
    "z": "STREAM:OFF"
}

def tuples_to_strings(data):
    return [tuple(str(x) for x in row) for row in data]

try:
    while True:
        print("\na: stream | z: stop | Ctrl+C: exit\n>> ", end='', flush=True)
        order = input()
        
        if order in map:
            cmd = map[order]
            ser.write(f"{cmd}\n".encode())
            print(f"Sent: {cmd}", end='', flush=True)
        else:
            print("Invalid command", end='', flush=True)
            
except KeyboardInterrupt:
    print("\n\nSaving data...")
    ser.close()
    result = tuples_to_strings(samples)
    with open(output_fn, 'w') as f:
        f.write('\n'.join('\t'.join(row) for row in result))
    
    print(f"Saved {len(samples)} samples to {output_fn}")
    print("Closed")