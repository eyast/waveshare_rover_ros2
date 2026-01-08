"""
WAVE ROVER Serial Communication Library (New Line-Based Protocol)
==================================================================

A Python library for controlling Waveshare WAVE ROVER robots via
simple line-based commands over serial communication.

Example:
    >>> from pyrover import PyRover
    >>> 
    >>> with PyRover('/dev/serial0') as rover:
    ...     rover.move(100, 100)  # Forward at ~40% speed
    ...     time.sleep(2)
    ...     rover.stop()

Protocol:
    Commands are simple line-based strings:
        M:left,right    - Motor control (-255 to 255)
        STOP            - Stop motors
        HB              - Heartbeat
        STREAM:ON/OFF   - Enable/disable telemetry
        FMT:RAW/IMU     - Set output format
    
    Responses have prefixes:
        I:...   - IMU telemetry
        P:...   - Power data
        Raw:... - MotionCal raw data
        Ori:... - MotionCal orientation
        S:...   - System messages
        A:...   - Acknowledgments
        E:...   - Errors
"""
import logging
import serial
import time
import threading
from typing import Optional, Callable, Dict
from dataclasses import dataclass

from .commands import OutputPrefix, MAX_PWM, MIN_PWM

from .tools import log_exceptions



class RoverCallbacks:
    """Callback functions for different message types."""
    def __init__(self,
                 on_imu = None,
                 on_power = None,
                 on_raw = None,
                 on_orientation = None,
                 on_system = None,
                 on_ack = None,
                 on_error = None,
                 on_line = None
                 ):
        self._on_imu: Optional[Callable[[str], None]] = on_imu
        self._on_power: Optional[Callable[[str], None]] = on_power
        self._on_raw: Optional[Callable[[str], None]] = on_raw
        self._on_orientation: Optional[Callable[[str], None]] = on_orientation
        self._on_system: Optional[Callable[[str], None]] = on_system
        self._on_ack: Optional[Callable[[str], None]] = on_ack
        self._on_error: Optional[Callable[[str], None]] = on_error
        self._on_line: Optional[Callable[[str], None]] = on_line

        self._dispatch: Dict[str, Optional[Callable]] = {
            OutputPrefix.IMU: self._on_imu,
            OutputPrefix.POWER: self._on_power,
            OutputPrefix.RAW: self._on_raw,
            OutputPrefix.ORI: self._on_orientation,
            OutputPrefix.SYSTEM: self._on_system,
            OutputPrefix.ACK: self._on_ack,
            OutputPrefix.ERROR: self._on_error,
        }

    @log_exceptions
    def _parse_line(self, line: str):
        """Sends a message to the appropriate callback"""
        if self._on_line:
            self._on_line(line)
        
        # Find and call the appropriate callback
        for prefix, callback in self._dispatch.items():
            if line.startswith(prefix):
                if callback:  
                    callback(line)
                break  

class PyRover:
    """
    WAVE ROVER robot controller using line-based protocol.
    
    Attributes:
        MAX_PWM: Maximum PWM value (255)
        MIN_PWM: Minimum PWM value (-255)
        HEARTBEAT_TIMEOUT: Robot stops if no command received (3.0s)
    """
    
    MAX_PWM = MAX_PWM
    MIN_PWM = MIN_PWM
    HEARTBEAT_TIMEOUT = 3.0
    
    def __init__(
        self,
        port: str = "/dev/serial0",
        baudrate: int = 115200,
        callbacks: Optional[RoverCallbacks] = None,
        auto_connect: bool = False,
        timeout: float = 0.1
    ):
        """
        Initialize WAVE ROVER controller.
        
        Args:
            port: Serial port path
            baudrate: Serial baudrate (default: 115200)
            callbacks: RoverCallbacks instance with handler functions
            auto_connect: Automatically connect on initialization
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.callbacks = callbacks or RoverCallbacks()
        self.timeout = timeout
        
        self._ser: Optional[serial.Serial] = None
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        

        self._logger = logging.getLogger(f"PyRover")
        
        if auto_connect:
            self.connect()
    
    def __enter__(self) -> 'PyRover':
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.stop()
        self.disconnect()
    
    # =========================================================================
    # Connection Management
    # =========================================================================
    
    @log_exceptions
    def connect(self) -> None:
        """Open serial connection and start reading thread."""
        if self._ser is not None and self._ser.is_open:
            return
        
        self._ser = serial.Serial(
            self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )
        self._ser.rts = False
        self._ser.dtr = False
        
        self._running = True
        self._read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._read_thread.start()
        
        time.sleep(0.1)
    
    @log_exceptions
    def disconnect(self) -> None:
        """Close serial connection and stop reading thread."""
        self._running = False
        
        if self._read_thread is not None:
            self._read_thread.join(timeout=1.0)
            self._read_thread = None
        
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            self._ser = None
    
    @property
    def is_connected(self) -> bool:
        """Check if serial connection is open."""
        return self._ser is not None and self._ser.is_open
    
    # =========================================================================
    # Serial Communication
    # =========================================================================
    
    @log_exceptions
    def _read_serial(self) -> None:
        """Background thread for reading and parsing serial data."""
        while self._running:
            try:
                if self._ser and self._ser.in_waiting:
                    line = self._ser.readline()
                    if line:
                        try:
                            decoded = line.decode('utf-8').strip()
                            if decoded:
                                self.callbacks._parse_line(decoded)
                        except UnicodeDecodeError:
                            pass
                else:
                    time.sleep(0.001)
            except Exception:
                if self._running:
                    time.sleep(0.01)
    
    def send(self, command: str) -> None:
        """
        Send a command string to the rover.
        
        Args:
            command: Command string (newline will be appended)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to WAVE ROVER")
        
        with self._lock:
            self._ser.write(command.encode() + b'\n')  # type: ignore
        
    # =========================================================================
    # Motion Control
    # =========================================================================
    
    @log_exceptions
    def move(self, left: int, right: int) -> None:
        """
        Set motor speeds.
        
        Args:
            left: Left motor speed (-255 to 255)
            right: Right motor speed (-255 to 255)
        """
        left = max(self.MIN_PWM, min(self.MAX_PWM, int(left)))
        right = max(self.MIN_PWM, min(self.MAX_PWM, int(right)))
        self.send(f"M:{left},{right}")
    
    @log_exceptions
    def stop(self) -> None:
        """Stop all motors."""
        self.send("STOP")
    
    @log_exceptions
    def emergency_stop(self) -> None:
        """Emergency stop - requires enable() to resume."""
        self.send("ESTOP")
    
    @log_exceptions
    def enable(self) -> None:
        """Re-enable motors after emergency stop."""
        self.send("ENABLE")
    
    @log_exceptions
    def heartbeat(self) -> None:
        """Send heartbeat to reset timeout."""
        self.send("HB")
    
    # =========================================================================
    # Calibartion Control
    # =========================================================================

    @log_exceptions
    def calib_on(self) -> None:
        """Load calibration data from memory"""
        self.send("CALIB:ON")

    @log_exceptions
    def calib_off(self) -> None:
        """Do not load calibration data from memory"""
        self.send("CALIB:OFF")

    # =========================================================================
    # WDT Control
    # =========================================================================

    @log_exceptions
    def wdt_status(self) -> None:
        """Get Status of ESP32 Watchdog
        
        Returns:
        S:WDT,HEALTH:<task_name>,
                     <status>,
                     <ms_since_last_heartbeat>,
                     <failure_count>
        """
        self.send("WDT:STATUS")

    @log_exceptions
    def wdt_stack(self) -> None:
        """Get Stack information
        
        Returns:
        S:WDT,STACK:<task_name>,
                    <used_bytes>,
                    <total_bytes>,
                    <percent_used>,
                    <free_bytes>
        """
        self.send("WDT:STACK")

    # =========================================================================
    # Streaming Control
    # =========================================================================
    
    @log_exceptions
    def stream_on(self) -> None:
        """Enable telemetry streaming."""
        self.send("STREAM:ON")
    
    @log_exceptions
    def stream_off(self) -> None:
        """Disable telemetry streaming."""
        self.send("STREAM:OFF")
    
    @log_exceptions
    def set_format_imu(self) -> None:
        """Set output format to IMU telemetry (I: messages)."""
        self.send("FMT:IMU")
    
    @log_exceptions
    def set_format_raw(self) -> None:
        """Set output format to MotionCal (Raw: and Ori: messages)."""
        self.send("FMT:RAW")
    
    # =========================================================================
    # System
    # =========================================================================
   
    @log_exceptions
    def reboot(self) -> None:
        """Reboot the ESP32."""
        self.send("REBOOT")