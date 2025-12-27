"""
Data Types for Waveshare WAVE ROVER
===================================

This module contains all data structures used by the WAVE ROVER library.
These are pure Python dataclasses with no ROS2 dependencies.
"""

from dataclasses import dataclass
from typing import Dict, Any, List


@dataclass
class IMUData_v2:
    """
    New IMUData class.

    Based on the datatype sent by the new ESP32 driver.
    
    This provides complete 9-axis IMU data including accelerometer,
    gyroscope, and magnetometer readings, plus the computed Euler angles
    and temperature.
    
    Units (based on Waveshare firmware analysis):
        - roll, pitch, yaw: degrees
        - accel_x/y/z: milli-g (mg) - divide by 1000 and multiply by 9.81 for m/s²
        - gyro_x/y/z: degrees/second
        - mag_x/y/z: raw ADC or microtesla (direction matters, magnitude varies)
        - temperature: Celsius
    """
    r: float = 0.0
    p: float = 0.0
    y: float = 0.0
    t: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    mx: int = 0
    my: int = 0
    mz: int = 0

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'IMUData_v2':
        """
        Create IMUData from response dictionary.
        
        The rover returns JSON with abbreviated keys.
        """
        return cls(   
            r=data.get('r', 0.0),
            p=data.get('p', 0.0),
            y=data.get('y', 0.0),
            t=data.get('t', 0.0),
            ax=data["a"][0],
            ay=data["a"][1],
            az=data["a"][2],
            gx=data["g"][0],
            gy=data["g"][1],
            gz=data["g"][2],
            mx=data["m"][0],
            my=data["m"][1],
            mz=data["m"][2],
        )

    @classmethod
    def from_list(cls, data: List[float]) -> 'IMUData_v2':
        """
        Create IMUData from a list of 9 ordered values.
        
        Expected order: [ax, ay, az, gx, gy, gz, mx, my, mz]
        
        Note: roll, pitch, yaw will default to 0.0 since they're
        not included in the 9-element raw sensor data.
        """
        return cls(
            ax=data[0],
            ay=data[1],
            az=data[2],
            gx=data[3],
            gy=data[4],
            gz=data[5],
            mx=int(data[6]),
            my=int(data[7]),
            mz=int(data[8]),
        )

@dataclass 
class ChassisInfo:
    """
    Chassis feedback data from T=130 (BASE_FEEDBACK) command.
    
    Contains battery voltage, current draw, and wheel speeds.
    
    Attributes:
        voltage: Battery pack voltage in volts
        current: Current draw in amps (may be 0 if not measured)
        left_speed: Left wheel speed
        right_speed: Right wheel speed
    """
    voltage: float = 0.0
    current: float = 0.0
    left_speed: float = 0.0
    right_speed: float = 0.0
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ChassisInfo':
        """Create ChassisInfo from response dictionary."""
        return cls(
            voltage=data.get('v', data.get('voltage', 0.0)),
            current=data.get('i', data.get('current', 0.0)),
            left_speed=data.get('L', 0.0),
            right_speed=data.get('R', 0.0),
        )


class BatteryEstimator:
    """
    Estimates battery state of charge for 18650 Li-ion cells.
    
    Uses a lookup table based on typical Li-ion discharge curves.
    The discharge curve is non-linear, so linear interpolation between
    known points gives a reasonable estimate.
    
    Why This Matters
    ----------------
    Li-ion batteries have a very non-linear discharge curve:
    - Voltage drops quickly at the start (4.2V -> 4.0V)
    - Stays relatively flat in the middle (4.0V -> 3.5V)  
    - Drops quickly again at the end (3.5V -> 3.0V)
    
    A simple linear mapping (e.g., 50% at 3.6V) would be very inaccurate.
    This class uses measured discharge data for better estimates.
    
    Configuration
    -------------
    The WAVE ROVER typically uses a 3S (3 cells in series) 18650 battery pack.
    Common capacities are 2000-3500 mAh per cell.
    
    Example:
        >>> estimator = BatteryEstimator(num_cells=3, capacity_mah=2600)
        >>> soc = estimator.voltage_to_soc(11.1)  # 3.7V per cell
        >>> print(f"Battery: {soc*100:.0f}%")
        Battery: 50%
    """
    
    # Voltage to SoC lookup table for single 18650 cell
    # Based on typical Li-ion discharge curve at moderate load
    # Format: (voltage_per_cell, state_of_charge_percent)
    DISCHARGE_CURVE = [
        (4.20, 100),
        (4.15, 95),
        (4.10, 90),
        (4.05, 85),
        (4.00, 80),
        (3.95, 75),
        (3.90, 70),
        (3.85, 65),
        (3.80, 60),
        (3.75, 55),
        (3.70, 50),
        (3.65, 45),
        (3.60, 40),
        (3.55, 35),
        (3.50, 30),
        (3.45, 25),
        (3.40, 20),
        (3.35, 15),
        (3.30, 10),
        (3.20, 5),
        (3.00, 0),
    ]
    
    # Voltage limits
    CELL_MAX_VOLTAGE = 4.2
    CELL_MIN_VOLTAGE = 3.0
    CELL_NOMINAL_VOLTAGE = 3.7
    
    def __init__(self, num_cells: int = 3, capacity_mah: int = 2600):
        """
        Initialize battery estimator.
        
        Args:
            num_cells: Number of cells in series (3S = 3 cells)
            capacity_mah: Capacity per cell in mAh (typical 18650: 2000-3500)
        """
        self.num_cells = num_cells
        self.capacity_mah = capacity_mah
        self.capacity_wh = (capacity_mah / 1000) * self.CELL_NOMINAL_VOLTAGE * num_cells
        
        # Scale discharge curve for pack voltage
        self.pack_curve = [
            (v * num_cells, soc) for v, soc in self.DISCHARGE_CURVE
        ]
    
    @property
    def max_voltage(self) -> float:
        """Maximum pack voltage (fully charged)."""
        return self.CELL_MAX_VOLTAGE * self.num_cells
    
    @property
    def min_voltage(self) -> float:
        """Minimum pack voltage (empty)."""
        return self.CELL_MIN_VOLTAGE * self.num_cells
    
    @property
    def nominal_voltage(self) -> float:
        """Nominal pack voltage."""
        return self.CELL_NOMINAL_VOLTAGE * self.num_cells
    
    def voltage_to_soc(self, voltage: float) -> float:
        """
        Convert pack voltage to state of charge percentage.
        
        Args:
            voltage: Pack voltage in volts
            
        Returns:
            State of charge as fraction (0.0 to 1.0)
        """
        if voltage >= self.max_voltage:
            return 1.0
        if voltage <= self.min_voltage:
            return 0.0
        
        # Find surrounding points in curve and interpolate
        for i in range(len(self.pack_curve) - 1):
            v_high, soc_high = self.pack_curve[i]
            v_low, soc_low = self.pack_curve[i + 1]
            
            if v_low <= voltage <= v_high:
                # Linear interpolation
                ratio = (voltage - v_low) / (v_high - v_low)
                soc = soc_low + ratio * (soc_high - soc_low)
                return soc / 100.0
        
        return 0.0
    
    def get_charge_wh(self, voltage: float) -> float:
        """
        Estimate remaining charge in Watt-hours.
        
        Args:
            voltage: Pack voltage in volts
            
        Returns:
            Estimated remaining charge in Wh
        """
        soc = self.voltage_to_soc(voltage)
        return self.capacity_wh * soc
    
    def get_voltage_per_cell(self, pack_voltage: float) -> float:
        """Get voltage per cell from pack voltage."""
        return pack_voltage / self.num_cells
    
    def get_health_status(self, voltage: float) -> str:
        """
        Get battery health status string.
        
        Args:
            voltage: Pack voltage
            
        Returns:
            Status string: 'GOOD', 'LOW', 'CRITICAL'
        """
        soc = self.voltage_to_soc(voltage)
        if soc > 0.2:
            return 'GOOD'
        elif soc > 0.1:
            return 'LOW'
        else:
            return 'CRITICAL'
