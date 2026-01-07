#!/usr/bin/env python3
"""
Example: Using PyRover with the new line-based protocol
=======================================================

This example shows how to use the pyrover library standalone (no ROS2).
"""

import time
import logging
from pyrover import PyRover, RoverCallbacks, IMUData, PowerData


def on_imu(data: IMUData):
    """Called when IMU data is received."""
    print(f"IMU: yaw={data.yaw:.1f}° pitch={data.pitch:.1f}° roll={data.roll:.1f}° temp={data.temp:.1f}°C")


def on_power(data: PowerData):
    """Called when power data is received."""
    print(f"Power: {data.voltage:.2f}V {data.current:.0f}mA {data.power:.0f}mW")


def on_system(msg):
    """Called for system messages."""
    print(f"System: [{msg.module}] {msg.message}")


def on_error(msg: str):
    """Called for error messages."""
    print(f"Error: {msg}")


def main():
    # Set up callbacks
    callbacks = RoverCallbacks(
        on_imu=on_imu,
        on_power=on_power,
        on_system=on_system,
        on_error=on_error,
    )
    
    # Connect to rover
    with PyRover('/dev/serial0', callbacks=callbacks) as rover:
        print("Connected to rover!")
        
        # Enable telemetry streaming
        rover.stream_on()
        rover.set_format_imu()
        
        # Wait for some data
        time.sleep(2)
        
        # Move forward
        print("Moving forward...")
        rover.move(100, 100)  # ~40% speed
        time.sleep(2)
        
        # Turn left
        print("Turning left...")
        rover.move(-50, 50)
        time.sleep(1)
        
        # Stop
        print("Stopping...")
        rover.stop()
        
        # Read latest data
        imu = rover._latest_imu
        power = rover._latest_power
        
        if imu:
            print(f"Final orientation: {imu.yaw:.1f}°")
        if power:
            print(f"Battery: {power.voltage:.2f}V")
        
        print("Done!")


if __name__ == '__main__':
    main()