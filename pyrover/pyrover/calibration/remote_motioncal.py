"""
Remote MotionCal Calibration
============================

The PyRover can be configured to stream raw IMU data to a remote
WebSocket python server that emulates a local tty device.

This way, the pyrover can be installed in its chassis, along all electric
components, and send data for gyroscope calibration


Usage Example
-------------
>>> from pyrover import PyRover
>>> from pyrover.calibration import RemoteMotionCal
>>>
>>> rover = PyRover('/dev/serial0', baudrate=115200)
>>> motioncal_calibrator = RemoteMotionCal(server= "192.168.10.64")
>>> motioncal_calibrator.configure(rover)
>>> motioncal_calibrator.start()
>>> motioncal_calibrator.status()
>>> motioncal_calibrator.stop()
"""

def run_calibrator_cli():
    """
    Command-line interface for motor calibration.
    
    Entry point for `pyrover-calibrate-motioncal` command.
    """
    import argparse
    import time
    import sys
    
    parser = argparse.ArgumentParser(
        description="PyRover Remote Streamer to WebSockets",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    pyrover-calibrate-motioncal --server 192.168.10.64

        """
    )
    parser.add_argument('--port', '-p', default='/dev/serial0',
                        help='Serial port (default: /dev/serial0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='Baudrate (default: 115200)')
    parser.add_argument('--server', '-s', default="192.168.10.64",
                        help='Remote WebSockets server running MotionCal')
    
    args = parser.parse_args()
    from pyrover import PyRover
    rover = PyRover(port=args.port, baudrate=args.baudrate)
    rover.connect()
    print(f"✓ Connected to {args.port}")
    wifi_out = rover.connect_to_wifi(server_ip=args.server)
    print(wifi_out)
    time.sleep(3)
    rover.start_ws_streaming()


if __name__ == "__main__":
    run_calibrator_cli()