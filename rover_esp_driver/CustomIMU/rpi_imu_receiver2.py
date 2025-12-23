#!/usr/bin/env python3
"""
=============================================================================
RPI IMU Data Receiver v2 - MotionCal Raw Format Compatible
=============================================================================

Receives raw IMU data from the ESP32 running the QMI8658C + AK09918C driver
in MotionCal-compatible format.

Data Format (from ESP32):
    "Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r\n"

    Where:
    - ax, ay, az: Raw accelerometer values (int16)
    - gx, gy, gz: Raw gyroscope values (int16)
    - mx, my, mz: Raw magnetometer values (int16)

Features:
- Parses MotionCal ASCII "Raw:" format
- Converts raw values to physical units
- Real-time data display
- Data logging to CSV
- Callback-based architecture for easy integration

Usage:
    python3 rpi_imu_receiver2.py [options]

Options:
    --port PORT       Serial port (default: /dev/ttyUSB0)
    --baud BAUD       Baud rate (default: 115200)
    --log FILE        Log data to CSV file
    --callback        Demo callback mode
    --raw             Display raw values instead of physical units

Author: Generated for RPI-ESP IMU project
License: MIT
=============================================================================
"""

import serial
import time
import threading
import argparse
import sys
from datetime import datetime


class IMUReceiverMotionCal:
    """
    Receives and parses IMU data from ESP32 in MotionCal Raw format.

    The ASCII protocol is:
        "Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r\n"

    All values are raw int16 sensor readings.
    """

    # Scale factors (must match ESP32 configuration)
    # Accelerometer: +/-2g at 16-bit resolution
    ACCEL_SCALE = 2.0 / 32768.0  # g/LSB
    # Gyroscope: +/-512 dps at 16-bit resolution
    GYRO_SCALE = 512.0 / 32768.0  # dps/LSB
    # Magnetometer: 0.15 uT/LSB (AK09918C)
    MAG_SCALE = 0.15  # uT/LSB

    def __init__(self, port='/dev/cu.usbserial-110', baud=115200):
        """
        Initialize the IMU receiver.

        Args:
            port: Serial port path
            baud: Baud rate (default: 115200, must match ESP configuration)
        """
        self.port = port
        self.baud = baud
        self.serial = None
        self.running = False
        self.callback = None
        self.log_file = None

        # Raw sensor data (int16)
        self.accel_raw = [0, 0, 0]
        self.gyro_raw = [0, 0, 0]
        self.mag_raw = [0, 0, 0]

        # Calibrated/scaled data
        self.accel = [0.0, 0.0, 0.0]  # g
        self.gyro = [0.0, 0.0, 0.0]   # dps
        self.mag = [0.0, 0.0, 0.0]    # uT

        # Statistics
        self.packets_received = 0
        self.packets_errors = 0
        self.last_rate_check = time.time()
        self.rate_counter = 0
        self.current_rate = 0.0

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

        The callback receives:
            (accel_raw, gyro_raw, mag_raw, accel, gyro, mag)
        Where each is a list of [x, y, z] values.
        """
        self.callback = callback_func

    def start_logging(self, filename):
        """Start logging data to a CSV file."""
        self.log_file = open(filename, 'w')
        self.log_file.write(
            "time,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,mx_raw,my_raw,mz_raw,"
            "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT\n"
        )
        print(f"Logging to {filename}")

    def stop_logging(self):
        """Stop logging data."""
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            print("Logging stopped")

    def _parse_raw_line(self, line):
        """
        Parse a MotionCal Raw format line.

        Format: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz"

        Returns True if valid, False otherwise.
        """
        try:
            # Remove whitespace and check prefix
            line = line.strip()
            if not line.startswith('Raw:'):
                return False

            # Extract data part
            data_part = line[4:]  # Skip "Raw:"
            values = data_part.split(',')

            if len(values) != 9:
                self.packets_errors += 1
                return False

            # Parse raw values
            self.accel_raw[0] = int(values[0])
            self.accel_raw[1] = int(values[1])
            self.accel_raw[2] = int(values[2])
            self.gyro_raw[0] = int(values[3])
            self.gyro_raw[1] = int(values[4])
            self.gyro_raw[2] = int(values[5])
            self.mag_raw[0] = int(values[6])
            self.mag_raw[1] = int(values[7])
            self.mag_raw[2] = int(values[8])

            # Convert to physical units
            self.accel[0] = self.accel_raw[0] * self.ACCEL_SCALE
            self.accel[1] = self.accel_raw[1] * self.ACCEL_SCALE
            self.accel[2] = self.accel_raw[2] * self.ACCEL_SCALE

            self.gyro[0] = self.gyro_raw[0] * self.GYRO_SCALE
            self.gyro[1] = self.gyro_raw[1] * self.GYRO_SCALE
            self.gyro[2] = self.gyro_raw[2] * self.GYRO_SCALE

            self.mag[0] = self.mag_raw[0] * self.MAG_SCALE
            self.mag[1] = self.mag_raw[1] * self.MAG_SCALE
            self.mag[2] = self.mag_raw[2] * self.MAG_SCALE

            self.packets_received += 1
            self.rate_counter += 1
            return True

        except (ValueError, IndexError) as e:
            self.packets_errors += 1
            return False

    def _update_rate(self):
        """Update data rate calculation."""
        now = time.time()
        elapsed = now - self.last_rate_check
        if elapsed >= 1.0:
            self.current_rate = self.rate_counter / elapsed
            self.rate_counter = 0
            self.last_rate_check = now

    def _handle_new_data(self):
        """Handle new data packet - call callback and log."""
        # Call user callback if set
        if self.callback:
            self.callback(
                self.accel_raw.copy(),
                self.gyro_raw.copy(),
                self.mag_raw.copy(),
                self.accel.copy(),
                self.gyro.copy(),
                self.mag.copy()
            )

        # Log to file if enabled
        if self.log_file:
            now = datetime.now().isoformat()
            self.log_file.write(
                f"{now},"
                f"{self.accel_raw[0]},{self.accel_raw[1]},{self.accel_raw[2]},"
                f"{self.gyro_raw[0]},{self.gyro_raw[1]},{self.gyro_raw[2]},"
                f"{self.mag_raw[0]},{self.mag_raw[1]},{self.mag_raw[2]},"
                f"{self.accel[0]:.6f},{self.accel[1]:.6f},{self.accel[2]:.6f},"
                f"{self.gyro[0]:.4f},{self.gyro[1]:.4f},{self.gyro[2]:.4f},"
                f"{self.mag[0]:.2f},{self.mag[1]:.2f},{self.mag[2]:.2f}\n"
            )

    def _receiver_thread(self):
        """Background thread for receiving data."""
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore')
                    if line.startswith('Raw:'):
                        if self._parse_raw_line(line):
                            self._handle_new_data()
                    elif line.strip():
                        # Print non-data lines (status messages, etc.)
                        print(f"[ESP] {line.strip()}")
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

    def get_accel_magnitude(self):
        """Get accelerometer magnitude in g."""
        return (self.accel[0]**2 + self.accel[1]**2 + self.accel[2]**2)**0.5

    def get_gyro_magnitude(self):
        """Get gyroscope magnitude in dps."""
        return (self.gyro[0]**2 + self.gyro[1]**2 + self.gyro[2]**2)**0.5

    def get_mag_magnitude(self):
        """Get magnetometer magnitude in uT."""
        return (self.mag[0]**2 + self.mag[1]**2 + self.mag[2]**2)**0.5


def demo_callback(accel_raw, gyro_raw, mag_raw, accel, gyro, mag):
    """Example callback function - called for each new data packet."""
    print(f"Accel: [{accel[0]:6.3f}, {accel[1]:6.3f}, {accel[2]:6.3f}] g  "
          f"Gyro: [{gyro[0]:7.2f}, {gyro[1]:7.2f}, {gyro[2]:7.2f}] dps  "
          f"Mag: [{mag[0]:6.1f}, {mag[1]:6.1f}, {mag[2]:6.1f}] uT")


def demo_callback_raw(accel_raw, gyro_raw, mag_raw, accel, gyro, mag):
    """Example callback function showing raw values."""
    print(f"Raw - Accel: [{accel_raw[0]:6d}, {accel_raw[1]:6d}, {accel_raw[2]:6d}]  "
          f"Gyro: [{gyro_raw[0]:6d}, {gyro_raw[1]:6d}, {gyro_raw[2]:6d}]  "
          f"Mag: [{mag_raw[0]:6d}, {mag_raw[1]:6d}, {mag_raw[2]:6d}]")


def main():
    parser = argparse.ArgumentParser(description='RPI IMU Data Receiver v2 - MotionCal Format')
    parser.add_argument('--port', default='/dev/cu.usbserial-110', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--log', help='Log data to CSV file')
    parser.add_argument('--callback', action='store_true', help='Demo callback mode')
    parser.add_argument('--raw', action='store_true', help='Display raw values')
    args = parser.parse_args()

    # Create receiver
    receiver = IMUReceiverMotionCal(
        port=args.port,
        baud=args.baud
    )

    # Connect
    if not receiver.connect():
        sys.exit(1)

    # Set up callback if requested
    if args.callback:
        if args.raw:
            receiver.set_callback(demo_callback_raw)
        else:
            receiver.set_callback(demo_callback)

    # Start logging if requested
    if args.log:
        receiver.start_logging(args.log)

    # Start receiving
    receiver.start()

    print("\nReceiving MotionCal Raw format data...")
    print("Data format: Raw:ax,ay,az,gx,gy,gz,mx,my,mz")
    print("")
    print("Fields:")
    print("  A = Accelerometer (x,y,z) - measures linear acceleration in g")
    print("  G = Gyroscope (x,y,z)     - measures angular velocity in degrees/sec")
    print("  M = Magnetometer (x,y,z)  - measures magnetic field in microtesla (uT)")
    print("")
    print("Commands: quit")
    print("Press Ctrl+C to exit\n")

    # Main loop - display data and process commands
    try:
        last_display = time.time()
        while True:
            # Display status periodically
            now = time.time()
            if now - last_display >= 0.1:  # 10 Hz display update
                if not args.callback:  # Don't duplicate if callback is showing data
                    if args.raw:
                        print(f"\rA:[{receiver.accel_raw[0]:6d},{receiver.accel_raw[1]:6d},{receiver.accel_raw[2]:6d}] "
                              f"G:[{receiver.gyro_raw[0]:6d},{receiver.gyro_raw[1]:6d},{receiver.gyro_raw[2]:6d}] "
                              f"M:[{receiver.mag_raw[0]:6d},{receiver.mag_raw[1]:6d},{receiver.mag_raw[2]:6d}] "
                              f"[{receiver.current_rate:.0f} Hz]    ",
                              end='', flush=True)
                    else:
                        print(f"\rA:[{receiver.accel[0]:6.3f},{receiver.accel[1]:6.3f},{receiver.accel[2]:6.3f}]g "
                              f"G:[{receiver.gyro[0]:7.2f},{receiver.gyro[1]:7.2f},{receiver.gyro[2]:7.2f}]dps "
                              f"M:[{receiver.mag[0]:6.1f},{receiver.mag[1]:6.1f},{receiver.mag[2]:6.1f}]uT "
                              f"[{receiver.current_rate:.0f} Hz]    ",
                              end='', flush=True)
                last_display = now

            # Check for user input (non-blocking)
            import select
            if select.select([sys.stdin], [], [], 0.01)[0]:
                cmd = input().strip().lower()

                if cmd == 'quit':
                    break
                elif cmd:
                    # Send any command directly to ESP
                    receiver.send_command(cmd)

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
