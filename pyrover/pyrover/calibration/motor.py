"""
Motor Gain Calibration (New Line-Based Protocol)
=================================================

The PyRover uses differential drive without wheel encoders. This calibration
tool helps determine the relationship between PWM values and actual motion.

The Problem
-----------
When you send `M:100,100`, how fast does the robot actually move?
Factors that affect this:
- Motor characteristics, battery level, surface friction, weight

The Solution: Empirical Calibration
-----------------------------------
Run the robot at known PWM values, measure actual motion, and compute
conversion factors:

    actual_speed_mps = pwm_value * linear_factor
    actual_angular_rps = pwm_value * angular_factor

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
...     result = calibrator.run_linear_test(pwm=100, duration=2.0)
...     
...     # User measures and inputs actual distance
...     measured = float(input("Measured distance (m): "))
...     linear_factor = result.compute_factor(measured)
...     
...     print(f"Linear factor: {linear_factor:.6f} m/s per PWM unit")
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
    pwm_value : int
        The PWM value that was commanded (-255 to 255)
    duration : float
        How long the test ran (seconds)
    measured_distance : float or None
        User-measured actual distance (set via compute_factor)
    computed_factor : float or None
        Calculated factor: m/s per PWM unit
    """
    pwm_value: int
    duration: float
    measured_distance: Optional[float] = None
    computed_factor: Optional[float] = None
    
    def compute_factor(self, measured_distance: float) -> float:
        """
        Compute the linear factor from measured distance.
        
        Parameters
        ----------
        measured_distance : float
            Actual distance traveled in meters
            
        Returns
        -------
        float
            Factor in m/s per PWM unit.
            
        Notes
        -----
        factor = (distance / time) / pwm = velocity / pwm
        
        Usage: actual_velocity = pwm * factor
        """
        self.measured_distance = measured_distance
        if self.pwm_value != 0 and self.duration > 0:
            velocity = measured_distance / self.duration
            self.computed_factor = velocity / abs(self.pwm_value)
        else:
            self.computed_factor = 0.0
        return self.computed_factor


@dataclass 
class AngularCalibrationResult:
    """
    Result of an angular motion calibration test.
    
    For differential drive rotation:
    - Left motor: -pwm, Right motor: +pwm
    - This produces counterclockwise rotation
    """
    pwm_value: int
    duration: float
    measured_rotation_deg: Optional[float] = None
    computed_factor: Optional[float] = None
    
    def compute_factor(self, measured_rotation_deg: float) -> float:
        """
        Compute the angular factor from measured rotation.
        
        Parameters
        ----------
        measured_rotation_deg : float
            Actual rotation in degrees (positive = counterclockwise)
            
        Returns
        -------
        float
            Factor in rad/s per PWM unit.
        """
        self.measured_rotation_deg = measured_rotation_deg
        if self.pwm_value != 0 and self.duration > 0:
            # Convert degrees to radians
            rotation_rad = math.radians(measured_rotation_deg)
            angular_velocity = rotation_rad / self.duration
            self.computed_factor = angular_velocity / abs(self.pwm_value)
        else:
            self.computed_factor = 0.0
        return self.computed_factor


@dataclass
class FullCalibrationResult:
    """
    Complete calibration results.
    
    Attributes
    ----------
    linear_factor : float
        m/s per PWM unit
    angular_factor : float
        rad/s per PWM unit
    linear_gain : float
        For ROS2: converts m/s command to PWM (1/linear_factor * 255)
    angular_gain : float
        For ROS2: converts rad/s command to PWM (1/angular_factor * 255)
    """
    linear_factor: float
    angular_factor: float
    linear_result: LinearCalibrationResult
    angular_result: AngularCalibrationResult
    
    @property
    def linear_gain(self) -> float:
        """ROS2 linear_gain parameter value."""
        if self.linear_factor > 0:
            # At PWM=255, what velocity do we get?
            max_velocity = 255 * self.linear_factor
            # So to get 1 m/s, we need PWM = 1/linear_factor
            return 1.0 / self.linear_factor
        return 200.0  # Default
    
    @property
    def angular_gain(self) -> float:
        """ROS2 angular_gain parameter value."""
        if self.angular_factor > 0:
            return 1.0 / self.angular_factor
        return 100.0  # Default
    
    def to_yaml(self) -> str:
        """Generate YAML config snippet for ROS2."""
        return f"""# Motor calibration results
# Generated by pyrover-calibrate-motors
#
# Measured factors:
#   Linear:  {self.linear_factor:.6f} m/s per PWM unit
#   Angular: {self.angular_factor:.6f} rad/s per PWM unit
#
rover_node:
  ros__parameters:
    linear_gain: {self.linear_gain:.1f}
    angular_gain: {self.angular_gain:.1f}
    max_speed: 255
"""

    def __str__(self) -> str:
        return (
            f"Calibration Results:\n"
            f"  Linear factor:  {self.linear_factor:.6f} m/s per PWM\n"
            f"  Angular factor: {self.angular_factor:.6f} rad/s per PWM\n"
            f"\n"
            f"  ROS2 linear_gain:  {self.linear_gain:.1f}\n"
            f"  ROS2 angular_gain: {self.angular_gain:.1f}\n"
            f"\n"
            f"Linear test (PWM={self.linear_result.pwm_value}):\n"
            f"  Duration:  {self.linear_result.duration:.1f}s\n"
            f"  Measured:  {self.linear_result.measured_distance:.3f} m\n"
            f"  Velocity:  {self.linear_result.measured_distance/self.linear_result.duration:.3f} m/s\n"
            f"\n"
            f"Angular test (PWM=±{self.angular_result.pwm_value}):\n"
            f"  Duration:  {self.angular_result.duration:.1f}s\n"
            f"  Measured:  {self.angular_result.measured_rotation_deg:.1f}°\n"
            f"  Angular:   {math.radians(self.angular_result.measured_rotation_deg)/self.angular_result.duration:.3f} rad/s\n"
        )


class MotorCalibrator:
    """
    Motor calibration tool for PyRover (PWM-based protocol).
    
    Parameters
    ----------
    rover : PyRover
        Connected rover instance
    countdown_seconds : int
        Countdown before each test (default 3)
    """
    
    def __init__(self, rover: 'PyRover', countdown_seconds: int = 3):
        self.rover = rover
        self.countdown_seconds = countdown_seconds
        
    def run_linear_test(
        self, 
        pwm: int = 100, 
        duration: float = 2.0,
        verbose: bool = True
    ) -> LinearCalibrationResult:
        """
        Run a straight-line driving test.
        
        Parameters
        ----------
        pwm : int
            PWM value (0 to 255, where 255 is full speed)
        duration : float
            How long to drive in seconds
        verbose : bool
            Print progress messages
            
        Returns
        -------
        LinearCalibrationResult
            Call compute_factor() with measured distance to get the factor.
        """
        pwm = abs(pwm)  # Ensure positive
        
        if verbose:
            print(f"\n{'='*50}")
            print("LINEAR CALIBRATION TEST")
            print(f"{'='*50}")
            print(f"\nWill drive at PWM={pwm} for {duration:.1f} seconds")
            print("Mark the robot's starting position!")
            
        # Countdown
        if verbose:
            for i in range(self.countdown_seconds, 0, -1):
                print(f"  Starting in {i}...")
                time.sleep(1)
            print("  GO!")
            
        # Execute movement (both motors same = straight)
        self.rover.move(pwm, pwm)
        time.sleep(duration)
        self.rover.stop()
        
        if verbose:
            print("  STOP!")
            print(f"\nMark the robot's ending position.")
        
        return LinearCalibrationResult(pwm_value=pwm, duration=duration)
        
    def run_angular_test(
        self,
        pwm: int = 100,
        duration: float = 2.0,
        verbose: bool = True
    ) -> AngularCalibrationResult:
        """
        Run a rotation-in-place test.
        
        Parameters
        ----------
        pwm : int
            PWM value (motors will be ±pwm)
        duration : float
            How long to rotate in seconds
        verbose : bool
            Print progress messages
            
        Returns
        -------
        AngularCalibrationResult
            Call compute_factor() with measured rotation in degrees.
        """
        pwm = abs(pwm)
        
        if verbose:
            print(f"\n{'='*50}")
            print("ANGULAR CALIBRATION TEST")
            print(f"{'='*50}")
            print(f"\nWill rotate at PWM=±{pwm} for {duration:.1f}s")
            print("Mark a reference line on the robot!")
            
        if verbose:
            for i in range(self.countdown_seconds, 0, -1):
                print(f"  Starting in {i}...")
                time.sleep(1)
            print("  GO!")
            
        # Execute rotation (left backward, right forward = CCW)
        self.rover.move(-pwm, pwm)
        time.sleep(duration)
        self.rover.stop()
        
        if verbose:
            print("  STOP!")
            print(f"\nMeasure how many degrees the robot rotated.")
        
        return AngularCalibrationResult(pwm_value=pwm, duration=duration)
        
    def run_full_calibration(
        self,
        linear_pwm: int = 100,
        linear_duration: float = 2.0,
        angular_pwm: int = 100,
        angular_duration: float = 2.0,
        verbose: bool = True
    ) -> FullCalibrationResult:
        """
        Run complete L-shaped calibration interactively.
        
        Returns
        -------
        FullCalibrationResult
            Complete calibration with factors and ROS2 gains
        """
        if verbose:
            print("\n" + "=" * 60)
            print("       PyRover MOTOR CALIBRATION (PWM Protocol)")
            print("=" * 60)
            print("\nThis will run two tests:")
            print("  1. Drive forward in a straight line")
            print("  2. Rotate in place")
            print("\nYou will measure the actual motion after each test.")
            input("\nPress ENTER when ready to begin...")
            
        # Linear test
        linear_result = self.run_linear_test(
            pwm=linear_pwm,
            duration=linear_duration,
            verbose=verbose
        )
        
        measured_distance = float(input("\nMeasured distance (meters): "))
        linear_factor = linear_result.compute_factor(measured_distance)
        print(f"  → Linear factor: {linear_factor:.6f} m/s per PWM")
            
        # Angular test
        input("\nPress ENTER to continue to angular test...")
            
        angular_result = self.run_angular_test(
            pwm=angular_pwm,
            duration=angular_duration,
            verbose=verbose
        )
        
        measured_rotation = float(input("\nMeasured rotation (degrees): "))
        angular_factor = angular_result.compute_factor(measured_rotation)
        print(f"  → Angular factor: {angular_factor:.6f} rad/s per PWM")
            
        return FullCalibrationResult(
            linear_factor=linear_factor,
            angular_factor=angular_factor,
            linear_result=linear_result,
            angular_result=angular_result
        )


def run_motor_calibration_cli():
    """
    Command-line interface for motor calibration.
    
    Entry point for `pyrover-calibrate-motors` command.
    """
    import argparse
    import sys
    
    parser = argparse.ArgumentParser(
        description="PyRover Motor Calibration Tool (PWM Protocol)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    pyrover-calibrate-motors --port /dev/serial0
    
The robot will run an L-shaped pattern:
    1. Drive forward - measure distance traveled
    2. Rotate in place - measure rotation angle
    
Output: Factors and ROS2 gains for your config file.
        """
    )
    parser.add_argument('--port', '-p', default='/dev/serial0',
                        help='Serial port (default: /dev/serial0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='Baudrate (default: 115200)')
    parser.add_argument('--linear-pwm', type=int, default=100,
                        help='Linear test PWM value (default: 100)')
    parser.add_argument('--linear-duration', type=float, default=2.0,
                        help='Linear test duration in seconds (default: 2.0)')
    parser.add_argument('--angular-pwm', type=int, default=100,
                        help='Angular test PWM value (default: 100)')
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
                print(f"  [MOCK] M:{left},{right}")
            def stop(self):
                print("  [MOCK] STOP")
                
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
            linear_pwm=args.linear_pwm,
            linear_duration=args.linear_duration,
            angular_pwm=args.angular_pwm,
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