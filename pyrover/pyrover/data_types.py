"""
Data Types for Waveshare WAVE ROVER (New Line-Based Driver)
============================================================

This module contains all data structures used by the WAVE ROVER library.
These are pure Python dataclasses with no ROS2 dependencies.

The new driver uses line-based output:
    I:yaw,pitch,roll,temp,ax,ay,az,gx,gy,gz,mx,my,mz
    P:voltage,current,power,shunt
"""

from dataclasses import dataclass, field
from typing import Optional, List


@dataclass
class IMUData:
    """
    IMU telemetry data parsed from I: messages.
    
    Format: I:yaw,pitch,roll,temp,ax,ay,az,gx,gy,gz,mx,my,mz
    
    Units:
        - yaw, pitch, roll: degrees
        - temp: Celsius (CPU temperature)
        - ax, ay, az: g (gravity units)
        - gx, gy, gz: degrees/second
        - mx, my, mz: microtesla (calibrated)
    """
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    temp: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0
    
    @classmethod
    def from_line(cls, line: str) -> Optional['IMUData']:
        """
        Parse IMU data from I: line.
        
        Args:
            line: Line starting with "I:" containing 13 comma-separated values
            
        Returns:
            IMUData instance or None if parsing fails
        """
        try:
            # Remove prefix and split
            if line.startswith("I:"):
                line = line[2:]
            
            parts = line.strip().split(',')
            if len(parts) != 13:
                return None
            
            values = [float(p) for p in parts]
            return cls(
                yaw=values[0],
                pitch=values[1],
                roll=values[2],
                temp=values[3],
                ax=values[4],
                ay=values[5],
                az=values[6],
                gx=values[7],
                gy=values[8],
                gz=values[9],
                mx=values[10],
                my=values[11],
                mz=values[12],
            )
        except (ValueError, IndexError):
            return None
    
    def is_valid(self) -> bool:
        """Check if data contains valid orientation (not all zeros)."""
        return not (self.yaw == 0.0 and self.pitch == 0.0 and self.roll == 0.0)


@dataclass
class PowerData:
    """
    Power telemetry data parsed from P: messages.
    
    Format: P:voltage,current,power,shunt
    
    Units:
        - voltage: Volts (battery voltage)
        - current: milliamps
        - power: milliwatts
        - shunt: millivolts (shunt voltage for debugging)
    """
    voltage: float = 0.0
    current: float = 0.0
    power: float = 0.0
    shunt: float = 0.0
    
    @classmethod
    def from_line(cls, line: str) -> Optional['PowerData']:
        """
        Parse power data from P: line.
        
        Args:
            line: Line starting with "P:" containing 4 comma-separated values
            
        Returns:
            PowerData instance or None if parsing fails
        """
        try:
            if line.startswith("P:"):
                line = line[2:]
            
            parts = line.strip().split(',')
            if len(parts) != 4:
                return None
            
            values = [float(p) for p in parts]
            return cls(
                voltage=values[0],
                current=values[1],
                power=values[2],
                shunt=values[3],
            )
        except (ValueError, IndexError):
            return None


@dataclass
class RawSensorData:
    """
    Raw sensor data parsed from Raw: messages (MotionCal format).
    
    Format: Raw:ax,ay,az,gx,gy,gz,mx,my,mz
    
    Note: Values are scaled for MotionCal compatibility:
        - Accel: 8192 LSB/g
        - Gyro: 16 LSB/dps  
        - Mag: 10 LSB/uT
    """
    ax: int = 0
    ay: int = 0
    az: int = 0
    gx: int = 0
    gy: int = 0
    gz: int = 0
    mx: int = 0
    my: int = 0
    mz: int = 0
    
    @classmethod
    def from_line(cls, line: str) -> Optional['RawSensorData']:
        """Parse from Raw: line."""
        try:
            if line.startswith("Raw:"):
                line = line[4:]
            
            parts = line.strip().split(',')
            if len(parts) != 9:
                return None
            
            values = [int(p) for p in parts]
            return cls(*values)
        except (ValueError, IndexError):
            return None
    
    def to_physical_units(self) -> tuple:
        """
        Convert to physical units.
        
        Returns:
            Tuple of (accel_g, gyro_dps, mag_ut) as 3-element lists
        """
        accel_g = [v / 8192.0 for v in [self.ax, self.ay, self.az]]
        gyro_dps = [v / 16.0 for v in [self.gx, self.gy, self.gz]]
        mag_ut = [v / 10.0 for v in [self.mx, self.my, self.mz]]
        return accel_g, gyro_dps, mag_ut


@dataclass
class Orientation:
    """
    Orientation data parsed from Ori: messages.
    
    Format: Ori: yaw,pitch,roll (note the space after Ori:)
    
    Units: degrees
    """
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    
    @classmethod
    def from_line(cls, line: str) -> Optional['Orientation']:
        """Parse from Ori: line."""
        try:
            # Handle "Ori: " with space
            if line.startswith("Ori:"):
                line = line[4:].strip()
            
            parts = line.split(',')
            if len(parts) != 3:
                return None
            
            return cls(
                yaw=float(parts[0]),
                pitch=float(parts[1]),
                roll=float(parts[2]),
            )
        except (ValueError, IndexError):
            return None


@dataclass
class SystemMessage:
    """
    System message parsed from S: messages.
    
    Format: S:module,message
    """
    module: str = ""
    message: str = ""
    
    @classmethod
    def from_line(cls, line: str) -> Optional['SystemMessage']:
        """Parse from S: line."""
        try:
            if line.startswith("S:"):
                line = line[2:]
            
            parts = line.strip().split(',', 1)
            if len(parts) >= 1:
                return cls(
                    module=parts[0],
                    message=parts[1] if len(parts) > 1 else ""
                )
            return None
        except (ValueError, IndexError):
            return None


class BatteryEstimator:
    """
    Estimates battery state of charge for Li-ion/LiPo cells.
    
    Uses a lookup table based on typical Li-ion discharge curves.
    
    Example:
        >>> estimator = BatteryEstimator(num_cells=3)
        >>> soc = estimator.voltage_to_soc(11.1)  # 3.7V per cell
        >>> print(f"Battery: {soc*100:.0f}%")
        Battery: 50%
    """
    
    # Voltage to SoC lookup table for single cell
    DISCHARGE_CURVE = [
        (4.20, 100), (4.15, 95), (4.10, 90), (4.05, 85), (4.00, 80),
        (3.95, 75), (3.90, 70), (3.85, 65), (3.80, 60), (3.75, 55),
        (3.70, 50), (3.65, 45), (3.60, 40), (3.55, 35), (3.50, 30),
        (3.45, 25), (3.40, 20), (3.35, 15), (3.30, 10), (3.20, 5),
        (3.00, 0),
    ]
    
    CELL_MAX_VOLTAGE = 4.2
    CELL_MIN_VOLTAGE = 3.0
    CELL_NOMINAL_VOLTAGE = 3.7
    
    def __init__(self, num_cells: int = 3, capacity_mah: int = 2600):
        """
        Initialize battery estimator.
        
        Args:
            num_cells: Number of cells in series (3S = 3 cells)
            capacity_mah: Capacity per cell in mAh
        """
        self.num_cells = num_cells
        self.capacity_mah = capacity_mah
        self.capacity_wh = (capacity_mah / 1000) * self.CELL_NOMINAL_VOLTAGE * num_cells
        
        self.pack_curve = [
            (v * num_cells, soc) for v, soc in self.DISCHARGE_CURVE
        ]
    
    @property
    def max_voltage(self) -> float:
        return self.CELL_MAX_VOLTAGE * self.num_cells
    
    @property
    def min_voltage(self) -> float:
        return self.CELL_MIN_VOLTAGE * self.num_cells
    
    @property
    def nominal_voltage(self) -> float:
        return self.CELL_NOMINAL_VOLTAGE * self.num_cells
    
    def voltage_to_soc(self, voltage: float) -> float:
        """
        Convert pack voltage to state of charge.
        
        Returns:
            State of charge as fraction (0.0 to 1.0)
        """
        if voltage >= self.max_voltage:
            return 1.0
        if voltage <= self.min_voltage:
            return 0.0
        
        for i in range(len(self.pack_curve) - 1):
            v_high, soc_high = self.pack_curve[i]
            v_low, soc_low = self.pack_curve[i + 1]
            
            if v_low <= voltage <= v_high:
                ratio = (voltage - v_low) / (v_high - v_low)
                soc = soc_low + ratio * (soc_high - soc_low)
                return soc / 100.0
        
        return 0.0
    
    def get_health_status(self, voltage: float) -> str:
        """Get battery health: 'GOOD', 'LOW', or 'CRITICAL'."""
        soc = self.voltage_to_soc(voltage)
        if soc > 0.2:
            return 'GOOD'
        elif soc > 0.1:
            return 'LOW'
        else:
            return 'CRITICAL'


# Backwards compatibility alias
IMUData_v2 = IMUData