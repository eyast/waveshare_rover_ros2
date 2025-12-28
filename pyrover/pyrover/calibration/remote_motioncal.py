"""
Remote MotionCal Calibration (New Line-Based Protocol)
=======================================================

Configure the PyRover to stream raw IMU data via WebSocket to a remote
MotionCal server for magnetometer calibration.

The rover streams in MotionCal format:
    Raw:ax,ay,az,gx,gy,gz,mx,my,mz
    Ori:yaw,pitch,roll

Usage Example
-------------
>>> from pyrover import PyRover
>>>
>>> with PyRover('/dev/serial0') as rover:
...     # Connect to WiFi and WebSocket server
...     rover.connect_wifi("MySSID", "MyPassword", "192.168.1.100", 8080)
...     
...     # Set format to MotionCal (Raw: and Ori:)
...     rover.set_format_raw()
...     
...     # Enable streaming
...     rover.stream_on()

CLI Usage
---------
    pyrover-calibrate-motioncal --ssid MyWiFi --password secret --server 192.168.1.100
"""

import time
import sys
import argparse


def wait_for_connection(rover, timeout: float = 30.0) -> bool:
    """
    Wait for system messages indicating connection status.
    
    Returns True if connected successfully.
    """
    from pyrover import SystemMessage
    
    connected = {'wifi': False, 'ws': False}
    start = time.time()
    
    original_callback = rover.callbacks.on_system
    
    def check_status(msg: SystemMessage):
        if 'WiFi' in msg.module and 'Connected' in msg.message:
            connected['wifi'] = True
            print(f"  ✓ WiFi connected")
        if 'WS' in msg.module and 'Connected' in msg.message:
            connected['ws'] = True
            print(f"  ✓ WebSocket connected")
        if original_callback:
            original_callback(msg)
    
    rover.callbacks.on_system = check_status
    
    while time.time() - start < timeout:
        if connected['wifi'] and connected['ws']:
            rover.callbacks.on_system = original_callback
            return True
        time.sleep(0.1)
    
    rover.callbacks.on_system = original_callback
    return False


def run_calibrator_cli():
    """
    Command-line interface for MotionCal streaming.
    
    Entry point for `pyrover-calibrate-motioncal` command.
    """
    parser = argparse.ArgumentParser(
        description="PyRover Remote MotionCal Streaming",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    pyrover-calibrate-motioncal --ssid MyWiFi --password secret --server 192.168.10.64

This will:
    1. Connect to the specified WiFi network
    2. Connect to a WebSocket server running MotionCal
    3. Stream raw IMU data for magnetometer calibration

The WebSocket server should forward data to MotionCal's expected format.
        """
    )
    parser.add_argument('--port', '-p', default='/dev/serial0',
                        help='Serial port (default: /dev/serial0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='Baudrate (default: 115200)')
    parser.add_argument('--ssid', '-s', required=True,
                        help='WiFi network SSID')
    parser.add_argument('--password', '-w', required=True,
                        help='WiFi password')
    parser.add_argument('--server', '-i', required=True,
                        help='WebSocket server IP address')
    parser.add_argument('--ws-port', type=int, default=8080,
                        help='WebSocket server port (default: 8080)')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Connection timeout in seconds (default: 30)')
    
    args = parser.parse_args()
    
    from pyrover import PyRover, RoverCallbacks
    
    print("=" * 50)
    print("PyRover MotionCal Streaming")
    print("=" * 50)
    
    # Set up callbacks to show connection status
    def on_system(msg):
        print(f"  [{msg.module}] {msg.message}")
    
    def on_error(msg):
        print(f"  ERROR: {msg}")
    
    callbacks = RoverCallbacks(
        on_system=on_system,
        on_error=on_error,
    )
    
    try:
        rover = PyRover(port=args.port, baudrate=args.baudrate, callbacks=callbacks)
        rover.connect()
        print(f"✓ Connected to {args.port}")
        
        # Connect to WiFi and WebSocket
        print(f"\nConnecting to WiFi '{args.ssid}'...")
        rover.connect_wifi(args.ssid, args.password, args.server, args.ws_port)
        
        # Wait for connection
        print("Waiting for connection...")
        time.sleep(5)  # Give time for connection
        
        # Set format to MotionCal (Raw: + Ori:)
        print("\nSetting format to MotionCal...")
        rover.set_format_raw()
        
        # Enable streaming
        print("Enabling streaming...")
        rover.stream_on()
        
        print("\n" + "=" * 50)
        print("STREAMING ACTIVE")
        print("=" * 50)
        print("\nData is being streamed to the WebSocket server.")
        print("Move the robot in a figure-8 pattern for calibration.")
        print("\nPress Ctrl+C to stop.\n")
        
        # Keep running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\nStopping stream...")
    except Exception as e:
        print(f"\nERROR: {e}")
        sys.exit(1)
    finally:
        try:
            rover.stream_off()
            rover.disconnect()
            print("Disconnected.")
        except:
            pass


if __name__ == "__main__":
    run_calibrator_cli()