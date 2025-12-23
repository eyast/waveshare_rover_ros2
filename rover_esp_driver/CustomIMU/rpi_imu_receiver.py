#!/usr/bin/env python3
"""
=============================================================================
RPI IMU Data Receiver
=============================================================================

Receives orientation data (roll, pitch, yaw, temperature) from the ESP32/ESP8266
running the QMI8658C + AK09918C driver.

Features:
- Binary protocol parsing with checksum validation
- Command sending to ESP
- Real-time data display
- Data logging to CSV
- Callback-based architecture for easy integration

Usage:
    python3 rpi_imu_receiver.py [options]
    
Options:
    --port PORT       Serial port (default: /dev/ttyUSB0)
    --baud BAUD       Baud rate (default: 921600)
    --log FILE        Log data to CSV file
    --text            Use text mode instead of binary
    --callback        Demo callback mode
    
Commands (type while running):
    cal:acc    - Calibrate accelerometer
    cal:gyro   - Calibrate gyroscope  
    cal:mag    - Calibrate magnetometer (30 seconds)
    status     - Get sensor status
    text       - Switch to text output
    bin        - Switch to binary output
    stop       - Stop streaming
    start      - Resume streaming
    quit       - Exit program

Author: Generated for RPI-ESP IMU project
License: MIT
=============================================================================
"""

import serial
import struct
import time
import threading
import argparse
import sys
from collections import deque
from datetime import datetime

class IMUReceiver:
    """
    Receives and parses IMU data from ESP32/ESP8266.
    
    The binary protocol is:
        Byte 0:     Sync byte (0xAA)
        Byte 1:     Packet type (0x01 = orientation)
        Bytes 2-5:  Timestamp (uint32_t, microseconds)
        Bytes 6-9:  Roll (float, degrees)
        Bytes 10-13: Pitch (float, degrees)
        Bytes 14-17: Yaw (float, degrees, 0-360)
        Bytes 18-21: Temperature (float, Celsius)
        Bytes 22-23: Status flags (uint16_t)
        Byte 24:    Checksum (XOR of bytes 1-23)
    """
    
    PACKET_SIZE = 25
    SYNC_BYTE = 0xAA
    PACKET_TYPE_ORIENTATION = 0x01
    
    # Status flag definitions
    STATUS_IMU_OK = 1 << 0
    STATUS_MAG_OK = 1 << 1
    STATUS_ACCEL_CAL = 1 << 2
    STATUS_GYRO_CAL = 1 << 3
    STATUS_MAG_CAL = 1 << 4
    STATUS_MAG_OVERFLOW = 1 << 5
    
    def __init__(self, port='/dev/cu.usbserial110', baud=115200, text_mode=True):
        """
        Initialize the IMU receiver.
        
        Args:
            port: Serial port path
            baud: Baud rate (must match ESP configuration)
            text_mode: If True, expect text format instead of binary
        """
        self.port = port
        self.baud = baud
        self.text_mode = text_mode
        self.serial = None
        self.running = False
        self.callback = None
        self.log_file = None
        
        # Latest data
        self.timestamp = 0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.temperature = 0.0
        self.status = 0
        
        # Statistics
        self.packets_received = 0
        self.packets_errors = 0
        self.last_rate_check = time.time()
        self.rate_counter = 0
        self.current_rate = 0.0
        
        # Buffer for synchronization
        self.buffer = bytearray()
        
    def connect(self):
        """Open serial connection to ESP."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"Connected to {self.port} at {self.baud} baud")
            time.sleep(0.5)  # Wait for connection to stabilize
            self.serial.reset_input_buffer()
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")
    
    def send_command(self, cmd):
        """
        Send a command to the ESP.
        
        Args:
            cmd: Command string (without newline)
        """
        if self.serial and self.serial.is_open:
            self.serial.write(f"{cmd}\n".encode())
            print(f"Sent: {cmd}")
    
    def set_callback(self, callback_func):
        """
        Set a callback function to be called on each new data packet.
        
        The callback receives: (timestamp, roll, pitch, yaw, temperature, status)
        """
        self.callback = callback_func
    
    def start_logging(self, filename):
        """Start logging data to a CSV file."""
        self.log_file = open(filename, 'w')
        self.log_file.write("time,timestamp_us,roll,pitch,yaw,temperature,status\n")
        print(f"Logging to {filename}")
    
    def stop_logging(self):
        """Stop logging data."""
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            print("Logging stopped")
    
    def _verify_checksum(self, packet):
        """Verify packet checksum."""
        checksum = 0
        for i in range(1, 24):
            checksum ^= packet[i]
        return checksum == packet[24]
    
    def _parse_binary_packet(self, packet):
        """
        Parse a binary orientation packet.
        
        Returns True if valid, False otherwise.
        """
        if len(packet) != self.PACKET_SIZE:
            return False
        
        if packet[0] != self.SYNC_BYTE:
            return False
        
        if packet[1] != self.PACKET_TYPE_ORIENTATION:
            return False
        
        if not self._verify_checksum(packet):
            self.packets_errors += 1
            return False
        
        # Parse data
        self.timestamp = struct.unpack('<I', packet[2:6])[0]
        self.roll = struct.unpack('<f', packet[6:10])[0]
        self.pitch = struct.unpack('<f', packet[10:14])[0]
        self.yaw = struct.unpack('<f', packet[14:18])[0]
        self.temperature = struct.unpack('<f', packet[18:22])[0]
        self.status = struct.unpack('<H', packet[22:24])[0]
        
        self.packets_received += 1
        self.rate_counter += 1
        
        return True
    
    def _parse_text_line(self, line):
        """
        Parse a text format line.
        
        Format: "R:-0.12 P:1.23 Y:45.67 T:23.4 S:0x3"
        """
        try:
            parts = line.strip().split()
            for part in parts:
                if part.startswith('R:'):
                    self.roll = float(part[2:])
                elif part.startswith('P:'):
                    self.pitch = float(part[2:])
                elif part.startswith('Y:'):
                    self.yaw = float(part[2:])
                elif part.startswith('T:'):
                    self.temperature = float(part[2:])
                elif part.startswith('S:'):
                    self.status = int(part[2:], 16)
            
            self.timestamp = int(time.time() * 1000000)
            self.packets_received += 1
            self.rate_counter += 1
            return True
        except:
            return False
    
    def _update_rate(self):
        """Update data rate calculation."""
        now = time.time()
        elapsed = now - self.last_rate_check
        if elapsed >= 1.0:
            self.current_rate = self.rate_counter / elapsed
            self.rate_counter = 0
            self.last_rate_check = now
    
    def _read_binary(self):
        """Read and process binary data."""
        # Read available data
        if self.serial.in_waiting > 0:
            data = self.serial.read(self.serial.in_waiting)
            self.buffer.extend(data)
        
        # Process complete packets
        while len(self.buffer) >= self.PACKET_SIZE:
            # Find sync byte
            try:
                sync_idx = self.buffer.index(self.SYNC_BYTE)
                if sync_idx > 0:
                    # Discard bytes before sync
                    del self.buffer[:sync_idx]
            except ValueError:
                # No sync byte found, clear buffer
                self.buffer.clear()
                break
            
            if len(self.buffer) < self.PACKET_SIZE:
                break
            
            # Extract packet
            packet = bytes(self.buffer[:self.PACKET_SIZE])
            del self.buffer[:self.PACKET_SIZE]
            
            # Parse packet
            if self._parse_binary_packet(packet):
                self._handle_new_data()
    
    def _read_text(self):
        """Read and process text data."""
        if self.serial.in_waiting > 0:
            try:
                line = self.serial.readline().decode('utf-8', errors='ignore')
                if line.startswith('R'):
                    if self._parse_text_line(line):
                        self._handle_new_data()
                elif line.strip():
                    ...
                    # Print non-data lines (status messages, etc.)
                    print(f"[ESP] {line.strip()}")
            except:
                pass
    
    def _handle_new_data(self):
        """Handle new data packet - call callback and log."""
        # Call user callback if set
        if self.callback:
            self.callback(
                self.timestamp,
                self.roll,
                self.pitch,
                self.yaw,
                self.temperature,
                self.status
            )
        
        # Log to file if enabled
        if self.log_file:
            now = datetime.now().isoformat()
            self.log_file.write(
                f"{now},{self.timestamp},{self.roll:.4f},{self.pitch:.4f},"
                f"{self.yaw:.4f},{self.temperature:.2f},{self.status}\n"
            )
    
    def _receiver_thread(self):
        """Background thread for receiving data."""
        while self.running:
            try:
                if self.text_mode:
                    self._read_text()
                else:
                    self._read_binary()
                self._update_rate()
            except Exception as e:
                print(f"Receiver error: {e}")
            time.sleep(0.0001)  # Small sleep to prevent CPU hogging
    
    def start(self):
        """Start receiving data in background thread."""
        if not self.serial or not self.serial.is_open:
            print("Not connected!")
            return False
        
        self.running = True
        self.receiver_thread = threading.Thread(target=self._receiver_thread)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        print("Receiver started")
        return True
    
    def stop(self):
        """Stop receiving data."""
        self.running = False
        if hasattr(self, 'receiver_thread'):
            self.receiver_thread.join(timeout=1.0)
        print("Receiver stopped")
    
    def get_status_string(self):
        """Get human-readable status string."""
        parts = []
        if self.status & self.STATUS_IMU_OK:
            parts.append("IMU:OK")
        else:
            parts.append("IMU:ERR")
        if self.status & self.STATUS_MAG_OK:
            parts.append("MAG:OK")
        else:
            parts.append("MAG:ERR")
        if self.status & self.STATUS_ACCEL_CAL:
            parts.append("A_CAL")
        if self.status & self.STATUS_GYRO_CAL:
            parts.append("G_CAL")
        if self.status & self.STATUS_MAG_CAL:
            parts.append("M_CAL")
        if self.status & self.STATUS_MAG_OVERFLOW:
            parts.append("MAG_OVF!")
        return " ".join(parts)


def demo_callback(timestamp, roll, pitch, yaw, temp, status):
    """Example callback function - called for each new data packet."""
    print(f"Roll: {roll:7.2f}°  Pitch: {pitch:7.2f}°  Yaw: {yaw:7.2f}°  Temp: {temp:5.1f}°C")


def main():
    parser = argparse.ArgumentParser(description='RPI IMU Data Receiver')
    parser.add_argument('--port', default='/dev/cu.usbserial-110', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--log', help='Log data to CSV file')
    parser.add_argument('--text', default=True, action='store_true', help='Use text mode')
    parser.add_argument('--callback', action='store_true', help='Demo callback mode')
    args = parser.parse_args()
    
    # Create receiver
    receiver = IMUReceiver(
        port=args.port,
        baud=args.baud,
        text_mode=args.text
    )
    
    # Connect
    if not receiver.connect():
        sys.exit(1)
    
    # Set up callback if requested
    if args.callback:
        receiver.set_callback(demo_callback)
    
    # Start logging if requested
    if args.log:
        receiver.start_logging(args.log)
    
    # Switch to text mode if requested
    if args.text:
        receiver.send_command("SET:TEXT")
    
    # Start receiving
    receiver.start()
    
    print("\nCommands: cal:acc, cal:gyro, cal:mag, status, text, bin, stop, start, quit")
    print("Press Ctrl+C to exit\n")
    
    # Main loop - display data and process commands
    try:
        last_display = time.time()
        while True:
            # Display status periodically
            now = time.time()
            if now - last_display >= 0.5:
                if not args.callback:  # Don't duplicate if callback is showing data
                    print(f"\rR:{receiver.roll:7.2f}° P:{receiver.pitch:7.2f}° "
                          f"Y:{receiver.yaw:7.2f}° T:{receiver.temperature:5.1f}°C "
                          f"[{receiver.current_rate:.0f} Hz] [{receiver.get_status_string()}]    ",
                          end='', flush=True)
                last_display = now
            
            # Check for user input (non-blocking)
            # This is a simple implementation - for production, use select() or similar
            import select
            if select.select([sys.stdin], [], [], 0.01)[0]:
                cmd = input().strip().lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'cal:acc':
                    receiver.send_command("CAL:ACC")
                elif cmd == 'cal:gyro':
                    receiver.send_command("CAL:GYRO")
                elif cmd == 'cal:mag':
                    receiver.send_command("CAL:MAG")
                elif cmd == 'status':
                    receiver.send_command("GET:STATUS")
                elif cmd == 'text':
                    receiver.text_mode = True
                    receiver.send_command("SET:TEXT")
                elif cmd == 'bin':
                    receiver.text_mode = False
                    receiver.send_command("SET:BIN")
                elif cmd == 'stop':
                    receiver.send_command("STOP")
                elif cmd == 'start':
                    receiver.send_command("START")
                elif cmd == 'raw':
                    receiver.send_command("GET:RAW")
                elif cmd:
                    # Send any other command directly
                    receiver.send_command(cmd.upper())
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    
    finally:
        print(f"\nTotal packets: {receiver.packets_received}, Errors: {receiver.packets_errors}")
        receiver.stop()
        receiver.stop_logging()
        receiver.disconnect()


if __name__ == '__main__':
    main()
