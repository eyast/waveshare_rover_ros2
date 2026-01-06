#!/usr/bin/env python3
"""
Serial Data Collector for Sensor Calibration

This script captures data from ESP32 over serial port and saves it to a file.
Works on macOS, Linux, and Windows.

Usage:
    python serial_data_collector.py --port /dev/cu.usbserial-* --output data.txt
    python serial_data_collector.py --list  # List available ports
"""

import serial
import serial.tools.list_ports
import argparse
import time
import sys
import re


def list_serial_ports():
    """List all available serial ports."""
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("No serial ports found!")
        return []
    
    print("\nAvailable serial ports:")
    print("-" * 60)
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.device}")
        print(f"   Description: {port.description}")
        print(f"   Hardware ID: {port.hwid}")
        print()
    
    return [port.device for port in ports]


def is_data_line(line):
    """
    Check if a line contains valid sensor data (3 tab/space-separated numbers).
    
    Returns True if line matches format: number number number
    """
    # Remove any leading/trailing whitespace
    line = line.strip()
    
    # Skip empty lines
    if not line:
        return False
    
    # Skip lines that are clearly not data
    if any(keyword in line.lower() for keyword in 
           ['calibration', 'sensor', 'starting', 'complete', 'collected',
            'instructions', 'begin', 'copy', 'save', 'warming', 'setting']):
        return False
    
    # Check if line contains exactly 3 numbers separated by whitespace/tabs
    parts = re.split(r'\s+', line)
    
    if len(parts) != 3:
        return False
    
    # Try to parse as floats
    try:
        float(parts[0])
        float(parts[1])
        float(parts[2])
        return True
    except ValueError:
        return False


def collect_data(port, output_file, baud_rate=115200, duration=None, verbose=True):
    """
    Collect data from serial port and save to file.
    
    Parameters
    ----------
    port : str
        Serial port name (e.g., '/dev/cu.usbserial-1420' or 'COM3')
    output_file : str
        Path to output file
    baud_rate : int
        Serial baud rate (default: 115200)
    duration : int, optional
        Collection duration in seconds (None = until interrupted)
    verbose : bool
        Print progress messages
    """
    
    if verbose:
        print("="*70)
        print("SENSOR DATA COLLECTION")
        print("="*70)
        print(f"Port: {port}")
        print(f"Baud rate: {baud_rate}")
        print(f"Output file: {output_file}")
        if duration:
            print(f"Duration: {duration} seconds")
        else:
            print("Duration: Until Ctrl+C pressed")
        print("="*70)
    
    try:
        # Open serial port
        ser = serial.Serial(port, baud_rate, timeout=1)
        if verbose:
            print(f"\n✓ Connected to {port}")
        time.sleep(2)  # Wait for ESP32 to reset
        
        # Clear any startup messages
        ser.reset_input_buffer()
        
        # Open output file
        with open(output_file, 'w') as f:
            if verbose:
                print(f"✓ Opened {output_file} for writing")
                print("\nCollecting data... (Press Ctrl+C to stop)")
                print("-"*70)
            
            start_time = time.time()
            line_count = 0
            data_count = 0
            
            try:
                while True:
                    # Check duration
                    if duration and (time.time() - start_time) > duration:
                        break
                    
                    # Read line from serial
                    if ser.in_waiting > 0:
                        try:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                        except:
                            continue
                        
                        line_count += 1
                        
                        # Echo to console if verbose
                        if verbose and line:
                            print(line)
                        
                        # Check if this is a data line
                        if is_data_line(line):
                            f.write(line + '\n')
                            f.flush()  # Ensure data is written immediately
                            data_count += 1
                    else:
                        time.sleep(0.01)
                        
            except KeyboardInterrupt:
                if verbose:
                    print("\n" + "-"*70)
                    print("\n✓ Collection stopped by user")
        
        ser.close()
        
        if verbose:
            print("\n" + "="*70)
            print("COLLECTION COMPLETE")
            print("="*70)
            print(f"Total lines read: {line_count}")
            print(f"Data lines saved: {data_count}")
            print(f"Output file: {output_file}")
            print("\nNext step:")
            print(f"  python ellipsoid_calibration.py {output_file}")
            print("="*70)
        
        return data_count
        
    except serial.SerialException as e:
        print(f"\n✗ Error: Could not open serial port {port}")
        print(f"  {e}")
        print("\nTry:")
        print("  1. Check port name with --list option")
        print("  2. Ensure no other program is using the port")
        print("  3. Check USB cable connection")
        sys.exit(1)
    
    except FileNotFoundError:
        print(f"\n✗ Error: Could not create output file {output_file}")
        sys.exit(1)


def interactive_mode():
    """Interactive mode for selecting port and collecting data."""
    print("\n" + "="*70)
    print("INTERACTIVE DATA COLLECTION")
    print("="*70)
    
    # List ports
    ports = list_serial_ports()
    
    if not ports:
        print("No serial ports found. Connect ESP32 and try again.")
        sys.exit(1)
    
    # Select port
    if len(ports) == 1:
        port = ports[0]
        print(f"\nAuto-selected only available port: {port}")
    else:
        while True:
            try:
                choice = int(input(f"\nSelect port (1-{len(ports)}): "))
                if 1 <= choice <= len(ports):
                    port = ports[choice - 1]
                    break
                else:
                    print(f"Please enter a number between 1 and {len(ports)}")
            except ValueError:
                print("Please enter a valid number")
    
    # Get output filename
    default_name = f"sensor_data_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    output = input(f"\nOutput filename [{default_name}]: ").strip()
    if not output:
        output = default_name
    
    # Get duration
    duration_input = input("\nCollection duration in seconds [60]: ").strip()
    try:
        duration = int(duration_input) if duration_input else 60
    except ValueError:
        duration = 60
    
    # Confirm
    print("\n" + "-"*70)
    print("Ready to collect data:")
    print(f"  Port: {port}")
    print(f"  Duration: {duration} seconds")
    print(f"  Output: {output}")
    print("-"*70)
    input("\nPress Enter to start (or Ctrl+C to cancel)...")
    
    # Collect
    collect_data(port, output, duration=duration)


def main():
    parser = argparse.ArgumentParser(
        description='Collect sensor calibration data from serial port',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Interactive mode
  python serial_data_collector.py
  
  # List available ports
  python serial_data_collector.py --list
  
  # Collect from specific port
  python serial_data_collector.py --port /dev/cu.usbserial-1420 --output accel_data.txt
  
  # Collect for 90 seconds
  python serial_data_collector.py --port COM3 --duration 90
        '''
    )
    
    parser.add_argument('--port', '-p',
                       help='Serial port (e.g., /dev/cu.usbserial-* or COM3)')
    parser.add_argument('--output', '-o',
                       help='Output filename (default: sensor_data_TIMESTAMP.txt)')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--duration', '-d', type=int,
                       help='Collection duration in seconds (default: until Ctrl+C)')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List available serial ports and exit')
    parser.add_argument('--quiet', '-q', action='store_true',
                       help='Quiet mode (minimal output)')
    
    args = parser.parse_args()
    
    # List ports mode
    if args.list:
        list_serial_ports()
        sys.exit(0)
    
    # Interactive mode if no port specified
    if not args.port:
        interactive_mode()
        sys.exit(0)
    
    # Command-line mode
    output = args.output or f"sensor_data_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    
    collect_data(
        port=args.port,
        output_file=output,
        baud_rate=args.baud,
        duration=args.duration,
        verbose=not args.quiet
    )


if __name__ == '__main__':
    main()
