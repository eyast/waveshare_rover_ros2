"""
WAVE ROVER Serial Communication Library
=======================================

A Python library for controlling Waveshare WAVE ROVER robots via JSON commands
over serial communication.

This is a pure Python library with no ROS2 dependencies. It can be used
standalone or integrated with ROS2 via the pyrover_driver package.

Example:
    >>> from pyrover import WaveRover
    >>> 
    >>> # Using context manager (recommended)
    >>> with PyRover('/dev/serial0') as rover:
    ...     rover.move(0.3, 0.3)  # Forward
    ...     time.sleep(2)
    ...     rover.stop()
    >>> 
    >>> # Manual connection
    >>> rover = PyRover('/dev/serial0')
    >>> rover.connect()
    >>> rover.move(0.3, 0.3)
    >>> rover.stop()
    >>> rover.disconnect()

Reference: https://www.waveshare.com/wiki/WAVE_ROVER
"""

import serial
import time
import threading
import json
import time
from typing import Optional, Callable, Dict, Any, Union

from .commands import CommandType, WiFiMode
from .data_types import IMUData, ChassisInfo


class PyRover:
    """
    WAVE ROVER robot controller.
    
    Provides a high-level interface for controlling Waveshare WAVE ROVER
    robots via JSON commands over serial communication.
    
    Attributes:
        MAX_SPEED: Maximum speed value (0.5 = 100% PWM)
        MIN_SPEED: Minimum speed value (-0.5 = 100% PWM reverse)
        MAX_PWM: Maximum PWM value (255)
        MIN_PWM: Minimum PWM value (-255)
        HEARTBEAT_TIMEOUT: Robot stops if no command received for this long
    """
    
    # Speed limits for WAVE ROVER (no encoders)
    MAX_SPEED = 0.5
    MIN_SPEED = -0.5
    
    # PWM limits
    MAX_PWM = 255
    MIN_PWM = -255
    
    # Heartbeat timeout (robot stops if no command received)
    HEARTBEAT_TIMEOUT = 3.0  # seconds
    
    def __init__(
        self,
        port: str = "/dev/serial0",
        baudrate: int = 115200,
        callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        auto_connect: bool = False,
        timeout: float = 0.1
    ):
        """
        Initialize WAVE ROVER controller.
        
        Args:
            port: Serial port path (e.g., '/dev/serial0', '/dev/ttyUSB0', 'COM3')
            baudrate: Serial baudrate (default: 115200)
            callback: Optional callback function for received data. Called with
                     parsed JSON dictionary whenever data is received.
            auto_connect: Automatically connect on initialization
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self.timeout = timeout
        
        self._ser: Optional[serial.Serial] = None
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self._response_buffer: list = []
        self._response_event = threading.Event()
        self._lock = threading.Lock()
        
        if auto_connect:
            self.connect()
    
    @property
    def now(self):
        return time.time()

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
        """
        Open serial connection and start reading thread.
        
        Raises:
            serial.SerialException: If connection fails
        """
        if self._ser is not None and self._ser.is_open:
            return
            
        self._ser = serial.Serial(
            self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            #rtscts=False,
            #dsrdtr=False
        )
        # Disable hardware flow control signals that might interfere
        self._ser.rts = False
        self._ser.dtr = False
        
        self._running = True
        self._read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._read_thread.start()
        
        # Small delay to let things settle
        time.sleep(0.1)
        
        # Hardcoding polling 
        self.set_continuous_feedback(False)
        self.set_serial_echo(False)
    
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
        """Check if serial connection is open.
        Serial connection can be open even if the Raspberry pi
        is not connected to the Waveshare board. Therefore,
        this function also sends a query to the board
        """
        return self._ser is not None and self._ser.is_open #and self.test_connection()
    
    def test_connection(self) -> bool:
        """Test the connection by sending an IMU request"""
        return True if self.get_imu_data(timeout=1) else False

    def _read_serial(self) -> None:
        """Background thread for reading serial data."""
        while self._running:
            try:
                if self._ser and self._ser.in_waiting:
                    line = self._ser.readline()
                    if line:
                        try:
                            decoded = line.decode('utf-8').strip()
                            if decoded:
                                # Try to parse as JSON
                                try:
                                    data = json.loads(decoded)
                                    self._response_buffer.append(data)
                                    self._response_event.set()
                                    
                                    if self.callback:
                                        self.callback(data)
                                except json.JSONDecodeError:
                                    # Not JSON, might be debug output
                                    if self.callback:
                                        self.callback({'raw': decoded})
                        except UnicodeDecodeError:
                            pass
            except Exception:
                if self._running:
                    time.sleep(0.01)
    
    def send_command(
        self,
        command: Dict[str, Any],
        wait_response: bool = False,
        timeout: float = 1.0
    ) -> Optional[Dict[str, Any]]:
        """
        Send a JSON command to the robot.
        
        Args:
            command: Dictionary containing the command (must include 'T' key)
            wait_response: Wait for and return response
            timeout: Response timeout in seconds
            
        Returns:
            Response dictionary if wait_response=True, else None
            
        Raises:
            ConnectionError: If not connected
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to WAVE ROVER")
        
        with self._lock:
            # Clear response buffer
            if wait_response:
                self._response_buffer.clear()
                self._response_event.clear()
            
            # Send command
            cmd_str = json.dumps(command, separators=(',', ':'))
            self._ser.write(cmd_str.encode() + b'\n')
            
            if wait_response:
                if self._response_event.wait(timeout):
                    if self._response_buffer:
                        return self._response_buffer[-1]
                return None
    
    def send_raw(self, data: str) -> None:
        """
        Send raw string command.
        
        Args:
            data: Raw string to send (newline will be appended)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to WAVE ROVER")
        
        self._ser.write(data.encode() + b'\n')

    # =========================================================================
    # Motion Control
    # =========================================================================
    
    def move(self, left: float, right: float) -> None:
        """
        Set wheel speeds using SPEED_CTRL command.
        
        This is the recommended command for controlling WAVE ROVER movement.
        Speed value of 0.5 = 100% PWM, 0.25 = 50% PWM.
        
        Args:
            left: Left wheel speed (-0.5 to 0.5)
            right: Right wheel speed (-0.5 to 0.5)
        """
        left = max(self.MIN_SPEED, min(self.MAX_SPEED, left))
        right = max(self.MIN_SPEED, min(self.MAX_SPEED, right))
        
        self.send_command({
            "T": CommandType.SPEED_CTRL,
            "L": left,
            "R": right
        })
    
    def stop(self) -> None:
        """Stop all wheel movement."""
        self.move(0, 0)
    
    def set_motor_pid(
        self,
        p: float = 200,
        i: float = 2500,
        d: float = 0,
        windup_limit: int = 255
    ) -> None:
        """
        Configure motor PID parameters.
        
        Note: Only applicable for encoder-equipped models (UGV01).
        
        Args:
            p: Proportional coefficient
            i: Integral coefficient
            d: Derivative coefficient
            windup_limit: Reserved for anti-windup (not used currently)
        """
        self.send_command({
            "T": CommandType.SET_MOTOR_PID,
            "P": p,
            "I": i,
            "D": d,
            "L": windup_limit
        })

    # =========================================================================
    # OLED Display
    # =========================================================================
    
    def oled_print(self, line: int, text: str) -> None:
        """
        Display text on OLED screen.
        
        Args:
            line: Line number (0-3)
            text: Text to display
        """
        line = max(0, min(3, line))
        self.send_command({
            "T": CommandType.OLED_CTRL,
            "lineNum": line,
            "Text": str(text)
        })
    
    def oled_clear(self) -> None:
        """Clear OLED and restore default display (robot info)."""
        self.send_command({"T": CommandType.OLED_DEFAULT})
    
    def oled_display(self, lines: list) -> None:
        """
        Display multiple lines on OLED.
        
        Args:
            lines: List of strings (up to 4 lines)
        """
        for i, text in enumerate(lines[:4]):
            self.oled_print(i, text)

    # =========================================================================
    # IMU & Sensors
    # =========================================================================
    
    def get_imu_data(self, timeout: float = 1.0) -> Optional[IMUData]:
        """
        Retrieve IMU sensor data.
        
        Returns:
            IMUData object with sensor readings, or None on timeout
        """
        response = self.send_command(
            {"T": CommandType.GET_IMU_DATA},
            wait_response=True,
            timeout=timeout
        )
        
        if response:
            if response["T"] == 1002:
                return IMUData.from_dict(response)
        return None

    # =========================================================================
    # Chassis Information
    # =========================================================================
    
    def get_chassis_info(self, timeout: float = 1.0) -> Optional[ChassisInfo]:
        """
        Get chassis feedback information.
        
        Returns:
            ChassisInfo with voltage, current, speeds
        """
        response = self.send_command(
            {"T": CommandType.BASE_FEEDBACK},
            wait_response=True,
            timeout=timeout
        )
        
        if response:
            return ChassisInfo.from_dict(response)
        return None
    
    def set_continuous_feedback(self, enabled: bool) -> None:
        """
        Enable/disable continuous serial feedback.
        
        When enabled, chassis continuously sends feedback without polling.
        Useful for ROS integration.
        
        Args:
            enabled: True to enable, False to disable
        """
        self.send_command({
            "T": CommandType.BASE_FEEDBACK_FLOW,
            "cmd": 1 if enabled else 0
        })
    
    def set_serial_echo(self, enabled: bool) -> None:
        """
        Enable/disable serial command echo.
        
        When enabled, all sent commands are echoed back.
        
        Args:
            enabled: True to enable, False to disable
        """
        self.send_command({
            "T": CommandType.SERIAL_ECHO,
            "cmd": 1 if enabled else 0
        })

    # =========================================================================
    # WiFi Configuration
    # =========================================================================
    
    def set_wifi_mode(self, mode: Union[WiFiMode, int]) -> None:
        """
        Set WiFi operating mode.
        
        Args:
            mode: WiFiMode enum (OFF, AP, STA, AP_STA)
        """
        self.send_command({
            "T": CommandType.WIFI_MODE,
            "cmd": int(mode)
        })
    
    def configure_wifi_ap(self, ssid: str, password: str) -> None:
        """
        Configure WiFi Access Point mode.
        
        Args:
            ssid: Network name
            password: Network password
        """
        self.send_command({
            "T": CommandType.WIFI_AP_CONFIG,
            "ssid": ssid,
            "password": password
        })
    
    def configure_wifi_sta(
        self,
        sta_ssid: str,
        sta_password: str,
        ap_ssid: str = "UGV",
        ap_password: str = "12345678"
    ) -> None:
        """
        Configure WiFi Station mode (connect to existing network).
        
        Args:
            sta_ssid: Network to connect to
            sta_password: Network password
            ap_ssid: Fallback AP name
            ap_password: Fallback AP password
        """
        self.send_command({
            "T": CommandType.WIFI_STA_CONFIG,
            "ap_ssid": ap_ssid,
            "ap_password": ap_password,
            "sta_ssid": sta_ssid,
            "sta_password": sta_password
        })
    
    def get_wifi_info(self, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """Get current WiFi configuration."""
        return self.send_command(
            {"T": CommandType.WIFI_INFO},
            wait_response=True,
            timeout=timeout
        )
    
    def save_wifi_config(self) -> None:
        """Save current WiFi settings to config file."""
        self.send_command({"T": CommandType.WIFI_CONFIG_CREATE})
    
    def disconnect_wifi(self) -> None:
        """Disconnect WiFi connection."""
        self.send_command({"T": CommandType.WIFI_DISCONNECT})

    # =========================================================================
    # ESP-NOW Communication
    # =========================================================================
    
    def set_espnow_mode(self, receive_enabled: bool) -> None:
        """
        Enable/disable ESP-NOW command receiving.
        
        Args:
            receive_enabled: True to receive commands, False to ignore
        """
        self.send_command({
            "T": CommandType.ESPNOW_MODE,
            "mode": 3 if receive_enabled else 0
        })
    
    def espnow_add_peer(self, mac_address: str) -> None:
        """
        Add a peer for ESP-NOW communication.
        
        Args:
            mac_address: MAC address (e.g., "CC:DB:A7:5C:1C:40")
        """
        self.send_command({
            "T": CommandType.ESPNOW_ADD_PEER,
            "mac": mac_address
        })
    
    def espnow_remove_peer(self, mac_address: str) -> None:
        """
        Remove a peer from ESP-NOW communication.
        
        Args:
            mac_address: MAC address to remove
        """
        self.send_command({
            "T": CommandType.ESPNOW_DEL_PEER,
            "mac": mac_address
        })
    
    def espnow_send(self, mac_address: str, command: Dict[str, Any]) -> None:
        """
        Send a command to a specific device via ESP-NOW (unicast).
        
        Args:
            mac_address: Target MAC address
            command: JSON command to send
        """
        self.send_command({
            "T": CommandType.ESPNOW_UNICAST,
            "mac": mac_address,
            "dev": 0,
            "b": 0,
            "s": 0,
            "e": 0,
            "h": 0,
            "cmd": 1,
            "megs": json.dumps(command, separators=(',', ':'))
        })
    
    def espnow_broadcast(self, command: Dict[str, Any]) -> None:
        """
        Broadcast a command to all devices via ESP-NOW.
        
        Args:
            command: JSON command to broadcast
        """
        # First add broadcast address as peer
        self.espnow_add_peer("FF:FF:FF:FF:FF:FF")
        time.sleep(0.05)
        
        self.espnow_send("FF:FF:FF:FF:FF:FF", command)
    
    def espnow_multicast(self, command: Dict[str, Any]) -> None:
        """
        Send command to all added peers via ESP-NOW (multicast).
        
        Args:
            command: JSON command to send
        """
        self.send_command({
            "T": CommandType.ESPNOW_MULTICAST,
            "dev": 0,
            "b": 0,
            "s": 0,
            "e": 0,
            "h": 0,
            "cmd": 1,
            "megs": json.dumps(command, separators=(',', ':'))
        })

    # =========================================================================
    # System Commands
    # =========================================================================
    
    def reboot(self) -> None:
        """Reboot the ESP32."""
        self.send_command({"T": CommandType.REBOOT})
