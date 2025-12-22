"""
Motor Gain Calibration
======================

The PyRover uses differential drive without wheel encoders. This means
we can't measure actual wheel speeds - we only know what PWM values we send.
To convert velocity commands (m/s, rad/s) to PWM, we need calibration gains.

The Problem
-----------
When you command "move at 0.3 m/s", how fast does the robot actually move?
Without encoders, we don't know. Factors that affect this:

1. **Motor characteristics** - Voltage, load, temperature
2. **Battery level** - Lower voltage = slower speeds  
3. **Surface friction** - Carpet vs tile vs concrete
4. **Weight distribution** - Payload affects acceleration

The Solution: Empirical Calibration
-----------------------------------
We run the robot at known command values, measure actual motion, and compute
correction gains:

    actual_speed = commanded_speed * linear_gain
    actual_angular = commanded_angular * angular_gain

L-Shaped Calibration Pattern
----------------------------
The calibration runs an "L" pattern:

    1. Drive forward for T seconds at speed V
       → Measure actual distance traveled (D_actual)
       → linear_gain = D_actual / (V * T)
       
    2. Rotate in place for T seconds at angular speed W
       → Measure actual rotation (θ_actual in degrees)
       → angular_gain = θ_actual / (W * T * 180/π)

Why L-Shape?
    - Tests linear and angular motion independently
    - Easy to mark start/end positions on floor
    - Rotation test shows any differential imbalance

Usage Example
-------------
>>> from pyrover import PyRover
>>> from pyrover.calibration import MotorCalibrator
>>>
>>> with PyRover('/dev/serial0') as rover:
...     calibrator = MotorCalibrator(rover)
...     
...     # Run linear test
...     print("Robot will drive forward...")
...     result = calibrator.run_linear_test(speed=0.3, duration=2.0)
...     
...     # User measures and inputs actual distance
...     measured = float(input("Measured distance (m): "))
...     linear_gain = result.compute_gain(measured)
...     
...     # Run angular test
...     print("Robot will rotate in place...")
...     result = calibrator.run_angular_test(speed=0.5, duration=2.0)
...     
...     # User measures rotation
...     degrees = float(input("Measured rotation (degrees): "))
...     angular_gain = result.compute_gain(degrees)
...     
...     print(f"Linear gain: {linear_gain:.3f}")
...     print(f"Angular gain: {angular_gain:.3f}")
"""

import time
import math
from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from ..rover import PyRover


@dataclass
class LinearCalibrationResult:
    """
    Result of a linear motion calibration test.
    
    Attributes
    ----------
    commanded_speed : float
        The speed that was commanded (m/s equivalent)
    duration : float
        How long the test ran (seconds)
    expected_distance : float
        Distance we'd expect if gain were 1.0
    measured_distance : float or None
        User-measured actual distance (set via compute_gain)
    computed_gain : float or None
        Calculated gain correction
        
    Example
    -------
    >>> result = calibrator.run_linear_test(0.3, 2.0)
    >>> print(f"Expected {result.expected_distance:.2f} m at gain=1.0")
    >>> gain = result.compute_gain(measured_distance=0.48)
    >>> print(f"Actual gain: {gain:.3f}")
    """
    commanded_speed: float
    duration: float
    expected_distance: float = field(init=False)
    measured_distance: Optional[float] = None
    computed_gain: Optional[float] = None
    
    def __post_init__(self):
        self.expected_distance = self.commanded_speed * self.duration
        
    def compute_gain(self, measured_distance: float) -> float:
        """
        Compute the linear gain from measured distance.
        
        Parameters
        ----------
        measured_distance : float
            Actual distance traveled in meters
            
        Returns
        -------
        float
            Gain correction factor. Multiply commanded speed by this
            to get actual speed.
            
        Notes
        -----
        gain = measured / expected = measured / (speed * time)
        
        If gain > 1.0: Robot moves faster than commanded
        If gain < 1.0: Robot moves slower than commanded
        """
        self.measured_distance = measured_distance
        if self.expected_distance > 0:
            self.computed_gain = measured_distance / self.expected_distance
        else:
            self.computed_gain = 1.0
        return self.computed_gain


@dataclass 
class AngularCalibrationResult:
    """
    Result of an angular motion calibration test.
    
    Attributes
    ----------
    commanded_angular_speed : float
        The angular speed that was commanded (rad/s equivalent)
    duration : float
        How long the test ran (seconds)
    expected_rotation_deg : float
        Degrees we'd expect if gain were 1.0
    measured_rotation_deg : float or None
        User-measured actual rotation in degrees
    computed_gain : float or None
        Calculated gain correction
    """
    commanded_angular_speed: float
    duration: float
    expected_rotation_deg: float = field(init=False)
    measured_rotation_deg: Optional[float] = None
    computed_gain: Optional[float] = None
    
    def __post_init__(self):
        # Convert rad/s * s -> rad -> degrees
        expected_rad = self.commanded_angular_speed * self.duration
        self.expected_rotation_deg = math.degrees(expected_rad)
        
    def compute_gain(self, measured_rotation_deg: float) -> float:
        """
        Compute the angular gain from measured rotation.
        
        Parameters
        ----------
        measured_rotation_deg : float
            Actual rotation in degrees (positive = counterclockwise)
            
        Returns
        -------
        float
            Gain correction factor.
        """
        self.measured_rotation_deg = measured_rotation_deg
        if abs(self.expected_rotation_deg) > 0.1:
            self.computed_gain = measured_rotation_deg / self.expected_rotation_deg
        else:
            self.computed_gain = 1.0
        return self.computed_gain


@dataclass
class FullCalibrationResult:
    """
    Complete calibration results.
    
    Attributes
    ----------
    linear_gain : float
        Multiply linear commands by this
    angular_gain : float
        Multiply angular commands by this
    linear_result : LinearCalibrationResult
        Details from linear test
    angular_result : AngularCalibrationResult
        Details from angular test
    """
    linear_gain: float
    angular_gain: float
    linear_result: LinearCalibrationResult
    angular_result: AngularCalibrationResult
    
    def to_yaml(self) -> str:
        """Generate YAML config snippet."""
        return f"""# Motor calibration results
# Generated by waveshare-calibrate-motors
rover_node:
  ros__parameters:
    linear_gain: {self.linear_gain:.4f}
    angular_gain: {self.angular_gain:.4f}
"""

    def __str__(self) -> str:
        return (
            f"Calibration Results:\n"
            f"  Linear gain:  {self.linear_gain:.4f}\n"
            f"  Angular gain: {self.angular_gain:.4f}\n"
            f"\n"
            f"Linear test:\n"
            f"  Commanded: {self.linear_result.commanded_speed:.2f} m/s for "
            f"{self.linear_result.duration:.1f}s\n"
            f"  Expected:  {self.linear_result.expected_distance:.3f} m\n"
            f"  Measured:  {self.linear_result.measured_distance:.3f} m\n"
            f"\n"
            f"Angular test:\n"
            f"  Commanded: {self.angular_result.commanded_angular_speed:.2f} rad/s for "
            f"{self.angular_result.duration:.1f}s\n"
            f"  Expected:  {self.angular_result.expected_rotation_deg:.1f}°\n"
            f"  Measured:  {self.angular_result.measured_rotation_deg:.1f}°\n"
        )


class MotorCalibrator:
    """
    Motor gain calibration tool for PyRover.
    
    Runs controlled movements and helps calculate gain corrections
    based on measured actual motion.
    
    Parameters
    ----------
    rover : PyRover
        Connected rover instance
    countdown_seconds : int
        Countdown before each test (default 3)
        
    Example
    -------
    >>> with PyRover('/dev/serial0') as rover:
    ...     cal = MotorCalibrator(rover)
    ...     
    ...     # Linear test
    ...     result = cal.run_linear_test(0.3, 2.0)
    ...     linear_gain = result.compute_gain(0.52)  # Measured 0.52m
    ...     
    ...     # Angular test  
    ...     result = cal.run_angular_test(0.5, 2.0)
    ...     angular_gain = result.compute_gain(95.0)  # Measured 95 degrees
    """
    
    def __init__(self, rover: 'PyRover', countdown_seconds: int = 3):
        """
        Initialize the motor calibrator.
        
        Parameters
        ----------
        rover : PyRover
            Connected rover instance
        countdown_seconds : int
            Seconds to count down before each test
        """
        self.rover = rover
        self.countdown_seconds = countdown_seconds
        
    def run_linear_test(
        self, 
        speed: float = 0.3, 
        duration: float = 2.0,
        verbose: bool = True
    ) -> LinearCalibrationResult:
        """
        Run a straight-line driving test.
        
        The robot will drive forward at the specified speed for the
        specified duration, then stop.
        
        Parameters
        ----------
        speed : float
            Commanded speed (0 to 0.5, where 0.5 is full speed)
        duration : float
            How long to drive in seconds
        verbose : bool
            Print progress messages
            
        Returns
        -------
        LinearCalibrationResult
            Contains expected distance. Call compute_gain() with
            measured distance to get the gain.
            
        Notes
        -----
        Mark the robot's starting position before calling this method.
        After the test, measure the distance from start to finish.
        """
        if verbose:
            print(f"\n{'='*50}")
            print("LINEAR CALIBRATION TEST")
            print(f"{'='*50}")
            print(f"\nWill drive at speed {speed:.2f} for {duration:.1f} seconds")
            print("Mark the robot's starting position!")
            
        # Countdown
        if verbose:
            for i in range(self.countdown_seconds, 0, -1):
                print(f"  Starting in {i}...")
                time.sleep(1)
            print("  GO!")
            
        # Execute movement
        self.rover.move(speed, speed)  # Both motors same = straight
        time.sleep(duration)
        self.rover.stop()
        
        if verbose:
            print("  STOP!")
            print(f"\nMark the robot's ending position.")
        
        result = LinearCalibrationResult(
            commanded_speed=speed,
            duration=duration
        )
        
        if verbose:
            print(f"Expected distance (at gain=1.0): {result.expected_distance:.3f} m")
            
        return result
        
    def run_angular_test(
        self,
        angular_speed: float = 0.5,
        duration: float = 2.0,
        verbose: bool = True
    ) -> AngularCalibrationResult:
        """
        Run a rotation-in-place test.
        
        The robot will rotate counterclockwise at the specified angular
        speed for the specified duration, then stop.
        
        Parameters
        ----------
        angular_speed : float
            Commanded angular speed (higher = faster rotation)
        duration : float
            How long to rotate in seconds
        verbose : bool
            Print progress messages
            
        Returns
        -------
        AngularCalibrationResult
            Contains expected rotation. Call compute_gain() with
            measured rotation in degrees to get the gain.
            
        Notes
        -----
        Use a reference line or tape to measure the rotation angle.
        Positive rotation is counterclockwise when viewed from above.
        
        For differential drive:
            - Left motor backward, right motor forward = CCW rotation
        """
        if verbose:
            print(f"\n{'='*50}")
            print("ANGULAR CALIBRATION TEST")
            print(f"{'='*50}")
            print(f"\nWill rotate at angular speed {angular_speed:.2f} for {duration:.1f}s")
            print("Mark a reference line on the robot (e.g., tape on front)!")
            
        # Countdown
        if verbose:
            for i in range(self.countdown_seconds, 0, -1):
                print(f"  Starting in {i}...")
                time.sleep(1)
            print("  GO!")
            
        # Execute rotation (left backward, right forward = CCW)
        self.rover.move(-angular_speed, angular_speed)
        time.sleep(duration)
        self.rover.stop()
        
        if verbose:
            print("  STOP!")
            print(f"\nMeasure how many degrees the robot rotated.")
        
        result = AngularCalibrationResult(
            commanded_angular_speed=angular_speed,
            duration=duration
        )
        
        if verbose:
            print(f"Expected rotation (at gain=1.0): {result.expected_rotation_deg:.1f}°")
            
        return result
        
    def run_full_calibration(
        self,
        linear_speed: float = 0.3,
        linear_duration: float = 2.0,
        angular_speed: float = 0.5,
        angular_duration: float = 2.0,
        verbose: bool = True
    ) -> FullCalibrationResult:
        """
        Run complete L-shaped calibration.
        
        Runs both linear and angular tests interactively, prompting
        for measurements after each test.
        
        Parameters
        ----------
        linear_speed : float
            Speed for linear test
        linear_duration : float
            Duration of linear test
        angular_speed : float
            Speed for angular test
        angular_duration : float
            Duration of angular test
        verbose : bool
            Print progress (should be True for interactive use)
            
        Returns
        -------
        FullCalibrationResult
            Complete calibration with both gains
        """
        if verbose:
            print("\n" + "=" * 60)
            print("       PyRover MOTOR CALIBRATION")
            print("=" * 60)
            print("\nThis will run two tests:")
            print("  1. Drive forward in a straight line")
            print("  2. Rotate in place")
            print("\nYou will measure the actual motion after each test.")
            input("\nPress ENTER when ready to begin...")
            
        # Linear test
        linear_result = self.run_linear_test(
            speed=linear_speed,
            duration=linear_duration,
            verbose=verbose
        )
        
        if verbose:
            measured_distance = float(input("\nMeasured distance (meters): "))
            linear_gain = linear_result.compute_gain(measured_distance)
            print(f"  → Linear gain: {linear_gain:.4f}")
        else:
            raise ValueError("Non-verbose mode requires manual gain computation")
            
        # Angular test
        if verbose:
            input("\nPress ENTER to continue to angular test...")
            
        angular_result = self.run_angular_test(
            angular_speed=angular_speed,
            duration=angular_duration,
            verbose=verbose
        )
        
        if verbose:
            measured_rotation = float(input("\nMeasured rotation (degrees): "))
            angular_gain = angular_result.compute_gain(measured_rotation)
            print(f"  → Angular gain: {angular_gain:.4f}")
            
        return FullCalibrationResult(
            linear_gain=linear_gain,
            angular_gain=angular_gain,
            linear_result=linear_result,
            angular_result=angular_result
        )


def run_motor_calibration_cli():
    """
    Command-line interface for motor calibration.
    
    Entry point for `waveshare-calibrate-motors` command.
    """
    import argparse
    import sys
    
    parser = argparse.ArgumentParser(
        description="PyRover Motor Calibration Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    waveshare-calibrate-motors --port /dev/serial0
    
The robot will run an L-shaped pattern:
    1. Drive forward - measure distance traveled
    2. Rotate in place - measure rotation angle
    
Output: linear_gain and angular_gain for your config file.
        """
    )
    parser.add_argument('--port', '-p', default='/dev/serial0',
                        help='Serial port (default: /dev/serial0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='Baudrate (default: 115200)')
    parser.add_argument('--linear-speed', type=float, default=0.3,
                        help='Linear test speed (default: 0.3)')
    parser.add_argument('--linear-duration', type=float, default=2.0,
                        help='Linear test duration in seconds (default: 2.0)')
    parser.add_argument('--angular-speed', type=float, default=0.5,
                        help='Angular test speed (default: 0.5)')
    parser.add_argument('--angular-duration', type=float, default=2.0,
                        help='Angular test duration in seconds (default: 2.0)')
    parser.add_argument('--test', action='store_true',
                        help='Test mode with simulated rover')
    
    args = parser.parse_args()
    
    # Connect to rover
    if args.test:
        print("TEST MODE - Using simulated rover")
        
        class MockRover:
            def move(self, left, right):
                print(f"  [MOCK] move({left:.2f}, {right:.2f})")
            def stop(self):
                print("  [MOCK] stop()")
                
        rover = MockRover()
    else:
        try:
            from pyrover import PyRover
            rover = PyRover(port=args.port, baudrate=args.baudrate)
            rover.connect()
            print(f"✓ Connected to {args.port}")
        except Exception as e:
            print(f"ERROR: Could not connect to rover: {e}")
            sys.exit(1)
            
    # Run calibration
    try:
        calibrator = MotorCalibrator(rover)
        result = calibrator.run_full_calibration(
            linear_speed=args.linear_speed,
            linear_duration=args.linear_duration,
            angular_speed=args.angular_speed,
            angular_duration=args.angular_duration
        )
        
        print("\n" + "=" * 60)
        print("CALIBRATION COMPLETE")
        print("=" * 60)
        print(result)
        
        print("-" * 40)
        print("Add to your rover_params.yaml:")
        print("-" * 40)
        print(result.to_yaml())
        
    except KeyboardInterrupt:
        print("\n\nCalibration cancelled.")
        if hasattr(rover, 'stop'):
            rover.stop()
        sys.exit(1)
    finally:
        if hasattr(rover, 'disconnect'):
            rover.disconnect()


if __name__ == '__main__':
    run_motor_calibration_cli()
