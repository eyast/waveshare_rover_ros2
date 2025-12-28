"""
Comprehensive Test Suite for PyRover (Line-Based Protocol)
===========================================================

This test suite validates:
1. All output message parsing (I:, P:, Raw:, Ori:, S:, A:, E:)
2. All input command generation
3. Callback invocation
4. Edge cases and error handling
5. Integration scenarios

Run with:
    pytest tests/test_commands.py -v
    pytest tests/test_commands.py -v --tb=short  # Shorter output
    pytest tests/test_commands.py::TestMotorCommands -v  # Single class

Interactive hardware test:
    pyrover-test --port /dev/serial0
"""

import pytest
import time
import math
import threading
from unittest.mock import Mock, MagicMock, patch, call
from io import BytesIO
from dataclasses import dataclass

# Import pyrover classes
from pyrover import (
    PyRover, RoverCallbacks,
    IMUData, PowerData, RawSensorData, Orientation, SystemMessage,
    BatteryEstimator,
    OutputPrefix, StreamFormat, MAX_PWM, MIN_PWM,
)


# =============================================================================
# FIXTURES AND MOCKS
# =============================================================================

class MockSerial:
    """Mock serial port for testing without hardware."""
    
    def __init__(self):
        self.written = []
        self.read_buffer = BytesIO()
        self.is_open = True
        self.rts = False
        self.dtr = False
        self._in_waiting = 0
        self._lock = threading.Lock()
    
    def write(self, data: bytes) -> int:
        with self._lock:
            self.written.append(data.decode().strip())
        return len(data)
    
    def readline(self) -> bytes:
        with self._lock:
            line = self.read_buffer.readline()
            if line:
                remaining = self.read_buffer.read()
                self.read_buffer = BytesIO(remaining)
                self.read_buffer.seek(0, 2)
                self._in_waiting = len(remaining)
            return line
    
    def inject_response(self, line: str):
        """Inject a response to be read."""
        with self._lock:
            pos = self.read_buffer.tell()
            self.read_buffer.seek(0, 2)  # End
            self.read_buffer.write((line + '\n').encode())
            self.read_buffer.seek(pos)
            self._in_waiting = self.read_buffer.getbuffer().nbytes - pos
    
    @property
    def in_waiting(self) -> int:
        with self._lock:
            return self._in_waiting
    
    def close(self):
        self.is_open = False
    
    def get_last_command(self) -> str:
        """Get the last command sent."""
        with self._lock:
            return self.written[-1] if self.written else ""
    
    def get_all_commands(self) -> list:
        """Get all commands sent."""
        with self._lock:
            return list(self.written)
    
    def clear_written(self):
        """Clear command history."""
        with self._lock:
            self.written.clear()


@pytest.fixture
def mock_serial():
    """Create a mock serial port."""
    return MockSerial()


@pytest.fixture
def mock_rover(mock_serial):
    """Create rover with mock serial (no actual connection)."""
    with patch('serial.Serial') as mock_serial_class:
        mock_serial_class.return_value = mock_serial
        
        rover = PyRover('/dev/test', auto_connect=False)
        rover._ser = mock_serial
        rover._running = True
        
        yield rover, mock_serial
        
        rover._running = False


# =============================================================================
# OUTPUT PARSING TESTS - IMU (I:)
# =============================================================================

class TestIMUDataParsing:
    """Test IMUData parsing from I: lines."""
    
    def test_parse_valid_complete_line(self):
        """Parse a complete IMU line with all 13 values."""
        line = "I:45.5,-10.2,3.1,32.5,0.01,-0.02,1.00,0.5,-0.3,0.1,25.5,-15.3,40.2"
        data = IMUData.from_line(line)
        
        assert data is not None
        assert data.yaw == pytest.approx(45.5, rel=1e-5)
        assert data.pitch == pytest.approx(-10.2, rel=1e-5)
        assert data.roll == pytest.approx(3.1, rel=1e-5)
        assert data.temp == pytest.approx(32.5, rel=1e-5)
        assert data.ax == pytest.approx(0.01, rel=1e-5)
        assert data.ay == pytest.approx(-0.02, rel=1e-5)
        assert data.az == pytest.approx(1.00, rel=1e-5)
        assert data.gx == pytest.approx(0.5, rel=1e-5)
        assert data.gy == pytest.approx(-0.3, rel=1e-5)
        assert data.gz == pytest.approx(0.1, rel=1e-5)
        assert data.mx == pytest.approx(25.5, rel=1e-5)
        assert data.my == pytest.approx(-15.3, rel=1e-5)
        assert data.mz == pytest.approx(40.2, rel=1e-5)
    
    def test_parse_without_prefix(self):
        """Parse line without I: prefix."""
        line = "45.5,-10.2,3.1,32.5,0.01,-0.02,1.00,0.5,-0.3,0.1,25.5,-15.3,40.2"
        data = IMUData.from_line(line)
        assert data is not None
        assert data.yaw == pytest.approx(45.5)
    
    def test_parse_with_whitespace(self):
        """Parse line with trailing whitespace."""
        line = "I:45.5,-10.2,3.1,32.5,0.01,-0.02,1.00,0.5,-0.3,0.1,25.5,-15.3,40.2  \n"
        data = IMUData.from_line(line)
        assert data is not None
        assert data.yaw == pytest.approx(45.5)
    
    def test_parse_negative_values(self):
        """Parse line with all negative values."""
        line = "I:-180.0,-90.0,-45.0,-10.0,-1.0,-1.0,-1.0,-100.0,-100.0,-100.0,-50.0,-50.0,-50.0"
        data = IMUData.from_line(line)
        assert data is not None
        assert data.yaw == pytest.approx(-180.0)
        assert data.pitch == pytest.approx(-90.0)
        assert data.temp == pytest.approx(-10.0)
    
    def test_parse_zero_values(self):
        """Parse line with all zeros."""
        line = "I:0,0,0,0,0,0,0,0,0,0,0,0,0"
        data = IMUData.from_line(line)
        assert data is not None
        assert data.yaw == 0.0
        assert data.ax == 0.0
    
    def test_parse_scientific_notation(self):
        """Parse line with scientific notation (edge case)."""
        line = "I:1.5e1,-1.02e1,3.1,32.5,1e-2,-2e-2,1.00,0.5,-0.3,0.1,25.5,-15.3,40.2"
        data = IMUData.from_line(line)
        assert data is not None
        assert data.yaw == pytest.approx(15.0)  # 1.5e1
        assert data.ax == pytest.approx(0.01)   # 1e-2
    
    def test_parse_too_few_values(self):
        """Reject line with too few values."""
        line = "I:1,2,3,4,5"
        data = IMUData.from_line(line)
        assert data is None
    
    def test_parse_too_many_values(self):
        """Reject line with too many values."""
        line = "I:1,2,3,4,5,6,7,8,9,10,11,12,13,14,15"
        data = IMUData.from_line(line)
        assert data is None
    
    def test_parse_invalid_float(self):
        """Reject line with non-numeric values."""
        line = "I:not,valid,data,here,x,y,z,a,b,c,d,e,f"
        data = IMUData.from_line(line)
        assert data is None
    
    def test_parse_empty_line(self):
        """Reject empty line."""
        data = IMUData.from_line("")
        assert data is None
    
    def test_parse_only_prefix(self):
        """Reject line with only prefix."""
        data = IMUData.from_line("I:")
        assert data is None
    
    def test_is_valid_with_orientation(self):
        """is_valid() returns True when orientation is non-zero."""
        data = IMUData(yaw=45.0, pitch=10.0, roll=5.0)
        assert data.is_valid() is True
    
    def test_is_valid_with_zero_orientation(self):
        """is_valid() returns False when orientation is all zeros."""
        data = IMUData(yaw=0.0, pitch=0.0, roll=0.0, temp=25.0, ax=1.0)
        assert data.is_valid() is False
    
    def test_is_valid_partial_zero(self):
        """is_valid() returns True when only some orientation values are zero."""
        data = IMUData(yaw=45.0, pitch=0.0, roll=0.0)
        assert data.is_valid() is True


# =============================================================================
# OUTPUT PARSING TESTS - POWER (P:)
# =============================================================================

class TestPowerDataParsing:
    """Test PowerData parsing from P: lines."""
    
    def test_parse_valid_power_line(self):
        """Parse a complete power line."""
        line = "P:12.45,350.5,4300.25,3.5"
        data = PowerData.from_line(line)
        
        assert data is not None
        assert data.voltage == pytest.approx(12.45)
        assert data.current == pytest.approx(350.5)
        assert data.power == pytest.approx(4300.25)
        assert data.shunt == pytest.approx(3.5)
    
    def test_parse_without_prefix(self):
        """Parse line without P: prefix."""
        line = "12.45,350.5,4300.25,3.5"
        data = PowerData.from_line(line)
        assert data is not None
        assert data.voltage == pytest.approx(12.45)
    
    def test_parse_low_battery(self):
        """Parse low battery voltage."""
        line = "P:9.5,100.0,950.0,1.0"
        data = PowerData.from_line(line)
        assert data is not None
        assert data.voltage == pytest.approx(9.5)
    
    def test_parse_high_current(self):
        """Parse high current draw."""
        line = "P:11.1,2500.0,27750.0,25.0"
        data = PowerData.from_line(line)
        assert data is not None
        assert data.current == pytest.approx(2500.0)
    
    def test_parse_zero_current(self):
        """Parse zero current (idle)."""
        line = "P:12.6,0.0,0.0,0.0"
        data = PowerData.from_line(line)
        assert data is not None
        assert data.current == 0.0
        assert data.power == 0.0
    
    def test_parse_negative_shunt(self):
        """Parse negative shunt voltage (charging?)."""
        line = "P:12.45,-50.0,-600.0,-0.5"
        data = PowerData.from_line(line)
        assert data is not None
        assert data.current == pytest.approx(-50.0)
    
    def test_parse_too_few_values(self):
        """Reject line with too few values."""
        line = "P:12.45,350.5"
        data = PowerData.from_line(line)
        assert data is None
    
    def test_parse_invalid_values(self):
        """Reject line with non-numeric values."""
        line = "P:bad,data,here,now"
        data = PowerData.from_line(line)
        assert data is None


# =============================================================================
# OUTPUT PARSING TESTS - RAW SENSOR (Raw:)
# =============================================================================

class TestRawSensorDataParsing:
    """Test RawSensorData parsing from Raw: lines."""
    
    def test_parse_valid_raw_line(self):
        """Parse a complete Raw: line."""
        line = "Raw:100,200,8192,5,-3,1,250,-150,400"
        data = RawSensorData.from_line(line)
        
        assert data is not None
        assert data.ax == 100
        assert data.ay == 200
        assert data.az == 8192
        assert data.gx == 5
        assert data.gy == -3
        assert data.gz == 1
        assert data.mx == 250
        assert data.my == -150
        assert data.mz == 400
    
    def test_parse_without_prefix(self):
        """Parse without Raw: prefix."""
        line = "100,200,8192,5,-3,1,250,-150,400"
        data = RawSensorData.from_line(line)
        assert data is not None
    
    def test_parse_negative_values(self):
        """Parse all negative values."""
        line = "Raw:-1000,-2000,-8192,-50,-30,-10,-250,-150,-400"
        data = RawSensorData.from_line(line)
        assert data is not None
        assert data.ax == -1000
        assert data.mz == -400
    
    def test_to_physical_units_accel(self):
        """Convert accelerometer to g."""
        data = RawSensorData(ax=8192, ay=0, az=0, gx=0, gy=0, gz=0, mx=0, my=0, mz=0)
        accel, gyro, mag = data.to_physical_units()
        assert accel[0] == pytest.approx(1.0)  # 8192 / 8192 = 1g
    
    def test_to_physical_units_gyro(self):
        """Convert gyroscope to dps."""
        data = RawSensorData(ax=0, ay=0, az=0, gx=16, gy=32, gz=-16, mx=0, my=0, mz=0)
        accel, gyro, mag = data.to_physical_units()
        assert gyro[0] == pytest.approx(1.0)   # 16 / 16 = 1 dps
        assert gyro[1] == pytest.approx(2.0)
        assert gyro[2] == pytest.approx(-1.0)
    
    def test_to_physical_units_mag(self):
        """Convert magnetometer to uT."""
        data = RawSensorData(ax=0, ay=0, az=0, gx=0, gy=0, gz=0, mx=10, my=-20, mz=30)
        accel, gyro, mag = data.to_physical_units()
        assert mag[0] == pytest.approx(1.0)    # 10 / 10 = 1 uT
        assert mag[1] == pytest.approx(-2.0)
        assert mag[2] == pytest.approx(3.0)


# =============================================================================
# OUTPUT PARSING TESTS - ORIENTATION (Ori:)
# =============================================================================

class TestOrientationParsing:
    """Test Orientation parsing from Ori: lines."""
    
    def test_parse_with_space(self):
        """Parse Ori: line with space (MotionCal format)."""
        line = "Ori: 45.50,-10.25,3.10"
        data = Orientation.from_line(line)
        
        assert data is not None
        assert data.yaw == pytest.approx(45.50)
        assert data.pitch == pytest.approx(-10.25)
        assert data.roll == pytest.approx(3.10)
    
    def test_parse_without_space(self):
        """Parse Ori: line without space."""
        line = "Ori:45.50,-10.25,3.10"
        data = Orientation.from_line(line)
        assert data is not None
        assert data.yaw == pytest.approx(45.50)
    
    def test_parse_full_rotation(self):
        """Parse full rotation values."""
        line = "Ori: 180.0,-90.0,90.0"
        data = Orientation.from_line(line)
        assert data is not None
        assert data.yaw == pytest.approx(180.0)
        assert data.pitch == pytest.approx(-90.0)
        assert data.roll == pytest.approx(90.0)
    
    def test_parse_invalid(self):
        """Reject invalid Ori: line."""
        data = Orientation.from_line("Ori: bad,data")
        assert data is None


# =============================================================================
# OUTPUT PARSING TESTS - SYSTEM MESSAGE (S:)
# =============================================================================

class TestSystemMessageParsing:
    """Test SystemMessage parsing from S: lines."""
    
    def test_parse_with_message(self):
        """Parse S: line with module and message."""
        line = "S:BOOT,System initialized"
        data = SystemMessage.from_line(line)
        
        assert data is not None
        assert data.module == "BOOT"
        assert data.message == "System initialized"
    
    def test_parse_module_only(self):
        """Parse S: line with module only."""
        line = "S:MOTORS"
        data = SystemMessage.from_line(line)
        
        assert data is not None
        assert data.module == "MOTORS"
        assert data.message == ""
    
    def test_parse_message_with_commas(self):
        """Parse message containing commas."""
        line = "S:IMU,Calibration: x=0.1, y=0.2, z=0.3"
        data = SystemMessage.from_line(line)
        
        assert data is not None
        assert data.module == "IMU"
        assert "x=0.1" in data.message
    
    def test_parse_various_modules(self):
        """Parse various module names."""
        modules = ["BOOT", "MOTORS", "IMU", "MAG", "POWER", "WIFI", "WS"]
        for module in modules:
            line = f"S:{module},test message"
            data = SystemMessage.from_line(line)
            assert data is not None
            assert data.module == module


# =============================================================================
# INPUT COMMAND TESTS - MOTOR CONTROL
# =============================================================================

class TestMotorCommands:
    """Test motor control command generation."""
    
    def test_move_forward(self, mock_rover):
        """Move forward with equal speeds."""
        rover, serial = mock_rover
        rover.move(100, 100)
        assert "M:100,100" in serial.get_all_commands()
    
    def test_move_backward(self, mock_rover):
        """Move backward with negative speeds."""
        rover, serial = mock_rover
        rover.move(-100, -100)
        assert "M:-100,-100" in serial.get_all_commands()
    
    def test_turn_left(self, mock_rover):
        """Turn left (right faster than left)."""
        rover, serial = mock_rover
        rover.move(50, 100)
        assert "M:50,100" in serial.get_all_commands()
    
    def test_turn_right(self, mock_rover):
        """Turn right (left faster than right)."""
        rover, serial = mock_rover
        rover.move(100, 50)
        assert "M:100,50" in serial.get_all_commands()
    
    def test_spin_left(self, mock_rover):
        """Spin left in place."""
        rover, serial = mock_rover
        rover.move(-100, 100)
        assert "M:-100,100" in serial.get_all_commands()
    
    def test_spin_right(self, mock_rover):
        """Spin right in place."""
        rover, serial = mock_rover
        rover.move(100, -100)
        assert "M:100,-100" in serial.get_all_commands()
    
    def test_move_clamped_high(self, mock_rover):
        """Clamp values above 255."""
        rover, serial = mock_rover
        rover.move(300, 400)
        assert "M:255,255" in serial.get_all_commands()
    
    def test_move_clamped_low(self, mock_rover):
        """Clamp values below -255."""
        rover, serial = mock_rover
        rover.move(-300, -400)
        assert "M:-255,-255" in serial.get_all_commands()
    
    def test_move_zero(self, mock_rover):
        """Move with zero values."""
        rover, serial = mock_rover
        rover.move(0, 0)
        assert "M:0,0" in serial.get_all_commands()
    
    def test_move_normalized_full_forward(self, mock_rover):
        """Normalized move at full speed forward."""
        rover, serial = mock_rover
        rover.move_normalized(1.0, 1.0)
        assert "M:255,255" in serial.get_all_commands()
    
    def test_move_normalized_full_reverse(self, mock_rover):
        """Normalized move at full speed reverse."""
        rover, serial = mock_rover
        rover.move_normalized(-1.0, -1.0)
        assert "M:-255,-255" in serial.get_all_commands()
    
    def test_move_normalized_half(self, mock_rover):
        """Normalized move at half speed."""
        rover, serial = mock_rover
        rover.move_normalized(0.5, 0.5)
        assert "M:127,127" in serial.get_all_commands()
    
    def test_move_normalized_clamped(self, mock_rover):
        """Normalized move clamped to ±1.0."""
        rover, serial = mock_rover
        rover.move_normalized(2.0, -2.0)
        assert "M:255,-255" in serial.get_all_commands()
    
    def test_stop(self, mock_rover):
        """Stop motors."""
        rover, serial = mock_rover
        rover.stop()
        assert "STOP" in serial.get_all_commands()
    
    def test_emergency_stop(self, mock_rover):
        """Emergency stop."""
        rover, serial = mock_rover
        rover.emergency_stop()
        assert "ESTOP" in serial.get_all_commands()
    
    def test_enable_after_estop(self, mock_rover):
        """Enable motors after emergency stop."""
        rover, serial = mock_rover
        rover.enable()
        assert "ENABLE" in serial.get_all_commands()
    
    def test_heartbeat(self, mock_rover):
        """Send heartbeat."""
        rover, serial = mock_rover
        rover.heartbeat()
        assert "HB" in serial.get_all_commands()


# =============================================================================
# INPUT COMMAND TESTS - STREAMING CONTROL
# =============================================================================

class TestStreamingCommands:
    """Test streaming control command generation."""
    
    def test_stream_on(self, mock_rover):
        """Enable streaming."""
        rover, serial = mock_rover
        rover.stream_on()
        assert "STREAM:ON" in serial.get_all_commands()
    
    def test_stream_off(self, mock_rover):
        """Disable streaming."""
        rover, serial = mock_rover
        rover.stream_off()
        assert "STREAM:OFF" in serial.get_all_commands()
    
    def test_format_imu(self, mock_rover):
        """Set IMU telemetry format."""
        rover, serial = mock_rover
        rover.set_format_imu()
        assert "FMT:IMU" in serial.get_all_commands()
    
    def test_format_raw(self, mock_rover):
        """Set MotionCal raw format."""
        rover, serial = mock_rover
        rover.set_format_raw()
        assert "FMT:RAW" in serial.get_all_commands()


# =============================================================================
# INPUT COMMAND TESTS - CALIBRATION
# =============================================================================

class TestCalibrationCommands:
    """Test calibration command generation."""
    
    def test_calibrate_gyro(self, mock_rover):
        """Calibrate gyroscope."""
        rover, serial = mock_rover
        rover.calibrate_gyro()
        assert "CAL:G" in serial.get_all_commands()
    
    def test_calibrate_accel(self, mock_rover):
        """Calibrate accelerometer."""
        rover, serial = mock_rover
        rover.calibrate_accel()
        assert "CAL:A" in serial.get_all_commands()
    
    def test_calibrate_all(self, mock_rover):
        """Calibrate all sensors."""
        rover, serial = mock_rover
        rover.calibrate_all()
        assert "CAL:ALL" in serial.get_all_commands()


# =============================================================================
# INPUT COMMAND TESTS - FILTER CONTROL
# =============================================================================

class TestFilterCommands:
    """Test filter control command generation."""
    
    def test_set_beta_low(self, mock_rover):
        """Set low beta value."""
        rover, serial = mock_rover
        rover.set_filter_beta(0.05)
        assert "BETA:0.0500" in serial.get_all_commands()
    
    def test_set_beta_high(self, mock_rover):
        """Set high beta value."""
        rover, serial = mock_rover
        rover.set_filter_beta(2.5)
        assert "BETA:2.5000" in serial.get_all_commands()
    
    def test_set_beta_typical(self, mock_rover):
        """Set typical beta value."""
        rover, serial = mock_rover
        rover.set_filter_beta(0.1)
        assert "BETA:0.1000" in serial.get_all_commands()
    
    def test_fast_convergence_default(self, mock_rover):
        """Fast convergence with default duration."""
        rover, serial = mock_rover
        rover.start_fast_convergence()
        assert "FAST:2000" in serial.get_all_commands()
    
    def test_fast_convergence_custom(self, mock_rover):
        """Fast convergence with custom duration."""
        rover, serial = mock_rover
        rover.start_fast_convergence(5000)
        assert "FAST:5000" in serial.get_all_commands()
    
    def test_reinit_filter(self, mock_rover):
        """Re-initialize filter."""
        rover, serial = mock_rover
        rover.reinit_filter()
        assert "INIT" in serial.get_all_commands()


# =============================================================================
# INPUT COMMAND TESTS - WIFI / WEBSOCKET
# =============================================================================

class TestWiFiCommands:
    """Test WiFi/WebSocket command generation."""
    
    def test_connect_wifi_with_port(self, mock_rover):
        """Connect WiFi with custom port."""
        rover, serial = mock_rover
        rover.connect_wifi("MySSID", "MyPassword", "192.168.1.100", 8080)
        assert "WS:MySSID,MyPassword,192.168.1.100,8080" in serial.get_all_commands()
    
    def test_connect_wifi_default_port(self, mock_rover):
        """Connect WiFi with default port."""
        rover, serial = mock_rover
        rover.connect_wifi("Network", "Pass123", "10.0.0.1")
        assert "WS:Network,Pass123,10.0.0.1,8080" in serial.get_all_commands()
    
    def test_connect_wifi_special_chars(self, mock_rover):
        """Connect WiFi with special characters in password."""
        rover, serial = mock_rover
        rover.connect_wifi("WiFi-2.4G", "P@ss!123", "192.168.0.1", 9000)
        assert "WS:WiFi-2.4G,P@ss!123,192.168.0.1,9000" in serial.get_all_commands()
    
    def test_disconnect_wifi(self, mock_rover):
        """Disconnect WiFi/WebSocket."""
        rover, serial = mock_rover
        rover.disconnect_wifi()
        assert "WS:OFF" in serial.get_all_commands()


# =============================================================================
# INPUT COMMAND TESTS - SYSTEM
# =============================================================================

class TestSystemCommands:
    """Test system command generation."""
    
    def test_request_status(self, mock_rover):
        """Request system status."""
        rover, serial = mock_rover
        rover.request_status()
        assert "STATUS" in serial.get_all_commands()
    
    def test_reboot(self, mock_rover):
        """Reboot device."""
        rover, serial = mock_rover
        rover.reboot()
        assert "REBOOT" in serial.get_all_commands()


# =============================================================================
# CALLBACK TESTS
# =============================================================================

class TestCallbackInvocation:
    """Test that callbacks are properly invoked."""
    
    def test_imu_callback(self):
        """IMU callback invoked for I: lines."""
        received = []
        callbacks = RoverCallbacks(on_imu=lambda d: received.append(d))
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            line = "I:45.5,-10.2,3.1,32.5,0.01,-0.02,1.00,0.5,-0.3,0.1,25.5,-15.3,40.2"
            rover._parse_line(line)
            
            assert len(received) == 1
            assert received[0].yaw == pytest.approx(45.5)
    
    def test_power_callback(self):
        """Power callback invoked for P: lines."""
        received = []
        callbacks = RoverCallbacks(on_power=lambda d: received.append(d))
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            rover._parse_line("P:12.45,350.5,4300.25,3.5")
            
            assert len(received) == 1
            assert received[0].voltage == pytest.approx(12.45)
    
    def test_raw_callback(self):
        """Raw callback invoked for Raw: lines."""
        received = []
        callbacks = RoverCallbacks(on_raw=lambda d: received.append(d))
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            rover._parse_line("Raw:100,200,8192,5,-3,1,250,-150,400")
            
            assert len(received) == 1
            assert received[0].ax == 100
    
    def test_orientation_callback(self):
        """Orientation callback invoked for Ori: lines."""
        received = []
        callbacks = RoverCallbacks(on_orientation=lambda d: received.append(d))
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            rover._parse_line("Ori: 45.5,-10.2,3.1")
            
            assert len(received) == 1
            assert received[0].yaw == pytest.approx(45.5)
    
    def test_system_callback(self):
        """System callback invoked for S: lines."""
        received = []
        callbacks = RoverCallbacks(on_system=lambda d: received.append(d))
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            rover._parse_line("S:BOOT,ready")
            
            assert len(received) == 1
            assert received[0].module == "BOOT"
    
    def test_ack_callback(self):
        """ACK callback invoked for A: lines."""
        received = []
        callbacks = RoverCallbacks(on_ack=lambda d: received.append(d))
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            rover._parse_line("A:STOP")
            
            assert len(received) == 1
            assert received[0] == "STOP"
    
    def test_error_callback(self):
        """Error callback invoked for E: lines."""
        received = []
        callbacks = RoverCallbacks(on_error=lambda d: received.append(d))
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            rover._parse_line("E:MOTOR,overcurrent")
            
            assert len(received) == 1
            assert "overcurrent" in received[0]
    
    def test_line_callback_always(self):
        """Line callback invoked for all lines."""
        lines = []
        callbacks = RoverCallbacks(on_line=lambda l: lines.append(l))
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            rover._parse_line("I:1,2,3,4,5,6,7,8,9,10,11,12,13")
            rover._parse_line("P:1,2,3,4")
            rover._parse_line("S:TEST,message")
            rover._parse_line("Unknown line")
            
            assert len(lines) == 4
    
    def test_multiple_callbacks(self):
        """Multiple callbacks can be set and invoked."""
        imu_data = []
        power_data = []
        lines = []
        
        callbacks = RoverCallbacks(
            on_imu=lambda d: imu_data.append(d),
            on_power=lambda d: power_data.append(d),
            on_line=lambda l: lines.append(l),
        )
        
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', callbacks=callbacks, auto_connect=False)
            rover._ser = MockSerial()
            
            rover._parse_line("I:1,2,3,4,5,6,7,8,9,10,11,12,13")
            rover._parse_line("P:12,100,1200,1")
            
            assert len(imu_data) == 1
            assert len(power_data) == 1
            assert len(lines) == 2


# =============================================================================
# DATA CACHING TESTS
# =============================================================================

class TestDataCaching:
    """Test that latest data is cached and accessible."""
    
    def test_imu_cached(self):
        """Latest IMU data is cached."""
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', auto_connect=False)
            rover._ser = MockSerial()
            
            assert rover.imu is None  # Initially None
            
            rover._parse_line("I:45,10,5,25,0,0,1,0,0,0,20,10,30")
            
            assert rover.imu is not None
            assert rover.imu.yaw == pytest.approx(45)
    
    def test_power_cached(self):
        """Latest power data is cached."""
        with patch('serial.Serial'):
            rover = PyRover('/dev/test', auto_connect=False)
            rover._ser = MockSerial()
            
            assert rover.power is None  # Initially None
            
            rover._parse_line("P:12.0,300,3600,3")
            
            assert rover.power is not None
            assert rover.power.voltage == pytest.approx(12.0)


# =============================================================================
# BATTERY ESTIMATOR TESTS
# =============================================================================

class TestBatteryEstimator:
    """Test battery state of charge estimation."""
    
    def test_full_battery(self):
        """Full battery (12.6V for 3S)."""
        est = BatteryEstimator(num_cells=3)
        soc = est.voltage_to_soc(12.6)
        assert soc == pytest.approx(1.0)
    
    def test_empty_battery(self):
        """Empty battery (9.0V for 3S)."""
        est = BatteryEstimator(num_cells=3)
        soc = est.voltage_to_soc(9.0)
        assert soc == pytest.approx(0.0)
    
    def test_half_battery(self):
        """Half battery (~11.1V for 3S)."""
        est = BatteryEstimator(num_cells=3)
        soc = est.voltage_to_soc(11.1)  # 3.7V per cell
        assert 0.4 <= soc <= 0.6
    
    def test_over_voltage(self):
        """Over voltage clamped to 100%."""
        est = BatteryEstimator(num_cells=3)
        soc = est.voltage_to_soc(15.0)
        assert soc == 1.0
    
    def test_under_voltage(self):
        """Under voltage clamped to 0%."""
        est = BatteryEstimator(num_cells=3)
        soc = est.voltage_to_soc(6.0)
        assert soc == 0.0
    
    def test_health_status_good(self):
        """Good battery status."""
        est = BatteryEstimator(num_cells=3)
        status = est.get_health_status(11.5)
        assert status == 'GOOD'
    
    def test_health_status_low(self):
        """Low battery status."""
        est = BatteryEstimator(num_cells=3)
        status = est.get_health_status(10.0)
        assert status == 'LOW'
    
    def test_health_status_critical(self):
        """Critical battery status."""
        est = BatteryEstimator(num_cells=3)
        status = est.get_health_status(9.2)
        assert status == 'CRITICAL'
    
    def test_2s_battery(self):
        """2S battery configuration."""
        est = BatteryEstimator(num_cells=2)
        assert est.max_voltage == pytest.approx(8.4)
        assert est.min_voltage == pytest.approx(6.0)


# =============================================================================
# INTEGRATION TESTS
# =============================================================================

class TestIntegrationScenarios:
    """Test realistic usage scenarios."""
    
    def test_startup_sequence(self, mock_rover):
        """Test typical startup sequence."""
        rover, serial = mock_rover
        
        # Typical startup
        rover.stream_on()
        rover.set_format_imu()
        rover.reinit_filter()
        
        commands = serial.get_all_commands()
        assert "STREAM:ON" in commands
        assert "FMT:IMU" in commands
        assert "INIT" in commands
    
    def test_motion_sequence(self, mock_rover):
        """Test motion control sequence."""
        rover, serial = mock_rover
        
        # Move forward, turn, stop
        rover.move(100, 100)
        rover.move(50, 100)  # Turn
        rover.move(100, 100)
        rover.stop()
        
        commands = serial.get_all_commands()
        assert commands[-1] == "STOP"
        assert "M:100,100" in commands
        assert "M:50,100" in commands
    
    def test_calibration_sequence(self, mock_rover):
        """Test calibration sequence."""
        rover, serial = mock_rover
        
        rover.stream_off()
        rover.calibrate_all()
        rover.reinit_filter()
        rover.start_fast_convergence(3000)
        rover.stream_on()
        
        commands = serial.get_all_commands()
        assert "STREAM:OFF" in commands
        assert "CAL:ALL" in commands
        assert "INIT" in commands
        assert "FAST:3000" in commands
        assert "STREAM:ON" in commands
    
    def test_emergency_scenario(self, mock_rover):
        """Test emergency stop and recovery."""
        rover, serial = mock_rover
        
        rover.move(200, 200)
        rover.emergency_stop()
        # ... some time later ...
        rover.enable()
        rover.move(50, 50)
        
        commands = serial.get_all_commands()
        assert "ESTOP" in commands
        assert "ENABLE" in commands


# =============================================================================
# CLI TEST TOOL
# =============================================================================

def run_command_test_cli():
    """
    Interactive command test tool for real hardware.
    
    Entry point for `pyrover-test` command.
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="PyRover Command Test Tool")
    parser.add_argument('--port', '-p', default='/dev/serial0',
                        help='Serial port (default: /dev/serial0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='Baudrate (default: 115200)')
    parser.add_argument('--motor-test', action='store_true',
                        help='Include motor tests (robot will move!)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("PyRover Command Test Tool")
    print("=" * 60)
    print(f"\nConnecting to {args.port}...")
    
    # Callbacks to show responses
    def on_imu(data):
        print(f"  [IMU] yaw={data.yaw:.1f}° pitch={data.pitch:.1f}° roll={data.roll:.1f}° temp={data.temp:.1f}°C")
    
    def on_power(data):
        print(f"  [PWR] {data.voltage:.2f}V {data.current:.0f}mA {data.power:.0f}mW")
    
    def on_system(msg):
        print(f"  [SYS] [{msg.module}] {msg.message}")
    
    def on_ack(msg):
        print(f"  [ACK] {msg}")
    
    def on_error(msg):
        print(f"  [ERR] {msg}")
    
    callbacks = RoverCallbacks(
        on_imu=on_imu,
        on_power=on_power,
        on_system=on_system,
        on_ack=on_ack,
        on_error=on_error,
    )
    
    with PyRover(args.port, args.baudrate, callbacks=callbacks) as rover:
        print("✓ Connected!\n")
        
        # Test categories
        tests = []
        
        # Streaming tests
        tests.append(("--- STREAMING TESTS ---", None))
        tests.append(("STREAM:ON", rover.stream_on))
        tests.append(("FMT:IMU", rover.set_format_imu))
        tests.append(("(wait for data)", lambda: time.sleep(1)))
        tests.append(("FMT:RAW", rover.set_format_raw))
        tests.append(("(wait for data)", lambda: time.sleep(1)))
        tests.append(("FMT:IMU", rover.set_format_imu))
        tests.append(("STREAM:OFF", rover.stream_off))
        
        # Filter tests
        tests.append(("--- FILTER TESTS ---", None))
        tests.append(("BETA:0.15", lambda: rover.set_filter_beta(0.15)))
        tests.append(("INIT", rover.reinit_filter))
        tests.append(("FAST:1000", lambda: rover.start_fast_convergence(1000)))
        
        # Calibration tests (non-destructive)
        tests.append(("--- STATUS TESTS ---", None))
        tests.append(("STATUS", rover.request_status))
        tests.append(("HB (heartbeat)", rover.heartbeat))
        
        # Motor tests (optional)
        if args.motor_test:
            tests.append(("--- MOTOR TESTS (ROBOT WILL MOVE!) ---", None))
            tests.append(("M:50,50 (forward)", lambda: rover.move(50, 50)))
            tests.append(("(moving...)", lambda: time.sleep(0.5)))
            tests.append(("STOP", rover.stop))
            tests.append(("M:-50,50 (turn left)", lambda: rover.move(-50, 50)))
            tests.append(("(turning...)", lambda: time.sleep(0.5)))
            tests.append(("STOP", rover.stop))
            tests.append(("ESTOP", rover.emergency_stop))
            tests.append(("ENABLE", rover.enable))
        else:
            print("\n[Skipping motor tests - use --motor-test to enable]\n")
        
        # Run all tests
        for name, test_fn in tests:
            if test_fn is None:
                print(f"\n{name}")
            else:
                print(f"Test: {name}")
                try:
                    test_fn()
                    time.sleep(0.3)
                except Exception as e:
                    print(f"  ERROR: {e}")
        
        print("\n" + "=" * 60)
        print("All tests completed!")
        print("=" * 60)


if __name__ == '__main__':
    # Run pytest if called directly without args
    import sys
    if len(sys.argv) == 1:
        pytest.main([__file__, '-v'])
    else:
        run_command_test_cli()