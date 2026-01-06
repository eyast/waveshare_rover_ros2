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
from typing import Optional, Callable
from dataclasses import dataclass

from .commands import OutputPrefix, MAX_PWM, MIN_PWM
from .data_types import (IMUData,
                         PowerData,
                         RawSensorData,
                         Orientation,
                         SystemMessage)


@dataclass
class RoverCallbacks:
    """Callback functions for different message types."""
    on_imu: Optional[Callable[[IMUData], None]] = None
    on_power: Optional[Callable[[PowerData], None]] = None
    on_raw: Optional[Callable[[RawSensorData], None]] = None
    on_orientation: Optional[Callable[[Orientation], None]] = None
    on_system: Optional[Callable[[SystemMessage], None]] = None
    on_ack: Optional[Callable[[str], None]] = None
    on_error: Optional[Callable[[str], None]] = None
    on_line: Optional[Callable[[str], None]] = None  # Raw line callback


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
        
        # Latest data (updated by read thread)
        self._latest_imu: Optional[IMUData] = None
        self._latest_power: Optional[PowerData] = None
        self._imu_lock = threading.Lock()
        self._power_lock = threading.Lock()
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
                                self._parse_line(decoded)
                        except UnicodeDecodeError:
                            pass
                else:
                    time.sleep(0.001)
            except Exception:
                if self._running:
                    time.sleep(0.01)
    
    def _parse_line(self, line: str) -> None:
        """Parse a received line and dispatch to appropriate callback."""
        
        # Always call raw line callback if set
        if self.callbacks.on_line:
            self.callbacks.on_line(line)
        
        # IMU telemetry
        if line.startswith(OutputPrefix.IMU):
            data = IMUData.from_line(line)
            if data:
                with self._imu_lock:
                    self._latest_imu = data
                if self.callbacks.on_imu:
                    self.callbacks.on_imu(data)
        
        # Power data
        elif line.startswith(OutputPrefix.POWER):
            data = PowerData.from_line(line)
            if data:
                with self._power_lock:
                    self._latest_power = data
                if self.callbacks.on_power:
                    self.callbacks.on_power(data)
        
        # MotionCal raw data
        elif line.startswith(OutputPrefix.RAW):
            data = RawSensorData.from_line(line)
            if data and self.callbacks.on_raw:
                self.callbacks.on_raw(data)
        
        # MotionCal orientation
        elif line.startswith(OutputPrefix.ORI):
            data = Orientation.from_line(line)
            if data and self.callbacks.on_orientation:
                self.callbacks.on_orientation(data)
        
        # System message
        elif line.startswith(OutputPrefix.SYSTEM):
            data = SystemMessage.from_line(line)
            if data and self.callbacks.on_system:
                self.callbacks.on_system(data)
        
        # Acknowledgment
        elif line.startswith(OutputPrefix.ACK):
            if self.callbacks.on_ack:
                self.callbacks.on_ack(line[2:])
        
        # Error
        elif line.startswith(OutputPrefix.ERROR):
            if self.callbacks.on_error:
                self.callbacks.on_error(line[2:])
    
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
    # Data Access
    # =========================================================================
    
    @property
    def imu(self) -> Optional[IMUData]:
        """Get latest IMU data (thread-safe)."""
        with self._imu_lock:
            return self._latest_imu
    
    @property
    def power(self) -> Optional[PowerData]:
        """Get latest power data (thread-safe)."""
        with self._power_lock:
            return self._latest_power
    
    # =========================================================================
    # Motion Control
    # =========================================================================
    
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
    
    def stop(self) -> None:
        """Stop all motors."""
        self.send("STOP")
    
    def emergency_stop(self) -> None:
        """Emergency stop - requires enable() to resume."""
        self.send("ESTOP")
    
    def enable(self) -> None:
        """Re-enable motors after emergency stop."""
        self.send("ENABLE")
    
    def heartbeat(self) -> None:
        """Send heartbeat to reset timeout."""
        self.send("HB")
    
    # =========================================================================
    # Streaming Control
    # =========================================================================
    
    def stream_on(self) -> None:
        """Enable telemetry streaming."""
        self.send("STREAM:ON")
    
    def stream_off(self) -> None:
        """Disable telemetry streaming."""
        self.send("STREAM:OFF")
    
    def set_format_imu(self) -> None:
        """Set output format to IMU telemetry (I: messages)."""
        self.send("FMT:IMU")
    
    def set_format_raw(self) -> None:
        """Set output format to MotionCal (Raw: and Ori: messages)."""
        self.send("FMT:RAW")
    
   
    # =========================================================================
    # System
    # =========================================================================
   
    def reboot(self) -> None:
        """Reboot the ESP32."""
        self.send("REBOOT")