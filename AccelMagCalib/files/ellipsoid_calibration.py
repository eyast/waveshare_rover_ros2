#!/usr/bin/env python3
"""
Ellipsoid Fitting for Accelerometer/Magnetometer Calibration

This implements the constrained least-squares ellipsoid fitting algorithm
for sensor calibration. It computes:
  1. Bias vector (B) - the offset/center of the ellipsoid
  2. Calibration matrix (A_inv) - transforms ellipsoid to sphere

Based on the paper: "Ellipsoid Fit" with algebraic constraint methods.

Mathematical Background:
-----------------------
General ellipsoid equation:
    ax² + by² + cz² + 2fyz + 2gxz + 2hxy + 2px + 2qy + 2rz + d = 0

In quadratic form:
    X^T Q X + 2U^T X + J = 0
Where:
    Q = [a  h  g]
        [h  b  f]
        [g  f  c]
    
    U = [p]
        [q]
        [r]
    
    X = [x]
        [y]
        [z]

The algorithm solves for these parameters with the constraint that
forces the solution to be an ellipsoid (not hyperboloid or paraboloid).

Author: Based on C implementation using mymathlib.com routines
"""

import numpy as np
from typing import Tuple, Optional
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fn = 'accelerator_samples.txt'

class EllipsoidCalibration:
    """
    Accelerometer/Magnetometer calibration using ellipsoid fitting.
    
    This class fits an ellipsoid to 3D sensor data and extracts calibration
    parameters to correct sensor readings.
    """
    
    def __init__(self, reference_field_strength: Optional[float] = None):
        """
        Initialize calibration.
        
        Parameters
        ----------
        reference_field_strength : float, optional
            Expected field magnitude after calibration.
            - For accelerometer: 1.0 (representing 1g)
            - For magnetometer: local field strength in your units (e.g., 0.569 Gauss)
            If None, will be calculated from data
        """
        self.reference_field_strength = reference_field_strength
        self.bias = None
        self.calibration_matrix = None
        self.raw_data = None
        
    def fit(self, data: np.ndarray, verbose: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Fit ellipsoid to data and compute calibration parameters.
        
        Parameters
        ----------
        data : np.ndarray
            N x 3 array of sensor measurements [x, y, z]
            Minimum 10 samples required, 100+ recommended
            
        verbose : bool
            Print detailed information about the calibration
            
        Returns
        -------
        bias : np.ndarray
            3x1 bias vector [bx, by, bz] to subtract from raw readings
        calibration_matrix : np.ndarray
            3x3 calibration matrix to multiply (reading - bias) by
            
        Algorithm Steps:
        ---------------
        1. Build design matrix D from sensor measurements
        2. Compute scatter matrix S = D^T @ D
        3. Partition S into blocks [S11, S12; S12^T, S22]
        4. Apply constraint matrix C (forces ellipsoid constraint)
        5. Solve generalized eigenvalue problem
        6. Extract largest eigenvector
        7. Reconstruct full parameter vector v
        8. Extract Q matrix and U vector from v
        9. Compute bias: B = -Q^(-1) @ U
        10. Compute calibration matrix: A_inv = k * sqrt(Q) where k is scaling factor
        """
        if data.shape[1] != 3:
            raise ValueError("Data must have 3 columns [x, y, z]")
        if data.shape[0] < 10:
            raise ValueError("Need at least 10 samples (100+ recommended)")
            
        self.raw_data = data.copy()
        n_samples = data.shape[0]
        
        if verbose:
            print(f"Calibrating with {n_samples} samples...")
            print(f"Data range: X[{data[:,0].min():.2f}, {data[:,0].max():.2f}], "
                  f"Y[{data[:,1].min():.2f}, {data[:,1].max():.2f}], "
                  f"Z[{data[:,2].min():.2f}, {data[:,2].max():.2f}]")
        
        # Step 1: Build design matrix D (10 x n_samples)
        # Each column is: [x², y², z², 2yz, 2xz, 2xy, 2x, 2y, 2z, 1]
        D = self._build_design_matrix(data)
        
        # Step 2: Compute scatter matrix S = D @ D^T (10 x 10)
        S = D @ D.T
        
        # Step 3: Partition S into blocks
        S11 = S[0:6, 0:6]  # 6x6 upper-left (quadratic terms)
        S12 = S[0:6, 6:10]  # 6x4 upper-right
        S22 = S[6:10, 6:10]  # 4x4 lower-right (linear + constant)
        
        # Step 4: Pre-inverted constraint matrix C (6x6)
        # This constraint forces the solution to be an ellipsoid
        # Constraint: 4abc + fgh - af² - bg² - ch² > 0 (ellipsoid condition)
        C = np.array([
            [0.0,  0.5,  0.5,  0.0,   0.0,   0.0],
            [0.5,  0.0,  0.5,  0.0,   0.0,   0.0],
            [0.5,  0.5,  0.0,  0.0,   0.0,   0.0],
            [0.0,  0.0,  0.0, -0.25,  0.0,   0.0],
            [0.0,  0.0,  0.0,  0.0,  -0.25,  0.0],
            [0.0,  0.0,  0.0,  0.0,   0.0,  -0.25]
        ])
        
        # Step 5: Solve reduced eigenvalue problem
        # We need: S11*v1 - S12*S22^(-1)*S12^T*v1 = lambda*C^(-1)*v1
        # Simplified: (S11 - S12*S22^(-1)*S12^T) @ v1 = lambda * C^(-1) @ v1
        
        S22_inv = np.linalg.inv(S22)
        SS = S11 - S12 @ S22_inv @ S12.T  # Schur complement
        
        # Solve generalized eigenvalue problem: E @ v1 = lambda @ v1
        # where E = C @ SS
        E = C @ SS
        
        # Step 6: Find eigenvectors and eigenvalues
        eigenvalues, eigenvectors = np.linalg.eig(E)
        
        # Step 7: Select eigenvector with largest eigenvalue
        max_idx = np.argmax(eigenvalues)
        v1 = eigenvectors[:, max_idx].real
        
        # Normalize v1
        v1 = v1 / np.linalg.norm(v1)
        
        # Ensure first element is positive (convention)
        if v1[0] < 0:
            v1 = -v1
            
        if verbose:
            print(f"Largest eigenvalue: {eigenvalues[max_idx].real:.6f}")
        
        # Step 8: Compute v2 from v1
        # v2 = -S22^(-1) @ S12^T @ v1
        v2 = -S22_inv @ S12.T @ v1
        
        # Step 9: Combine to get full 10-parameter vector
        v = np.concatenate([v1, v2])
        
        # Step 10: Extract ellipsoid parameters
        # v = [a, b, c, f, g, h, p, q, r, d]
        # Quadratic form matrix Q:
        Q = np.array([
            [v[0], v[5], v[4]],  # [a, h, g]
            [v[5], v[1], v[3]],  # [h, b, f]
            [v[4], v[3], v[2]]   # [g, f, c]
        ])
        
        # Linear term vector U:
        U = np.array([v[6], v[7], v[8]])
        
        # Constant term:
        J = v[9]
        
        # Step 11: Compute bias (center of ellipsoid)
        # B = -Q^(-1) @ U
        Q_inv = np.linalg.inv(Q)
        self.bias = -Q_inv @ U
        
        if verbose:
            print(f"Bias (offset): [{self.bias[0]:.4f}, {self.bias[1]:.4f}, {self.bias[2]:.4f}]")
        
        # Step 12: Compute expected field magnitude
        # hmb = sqrt(B^T @ Q @ B - J)
        hmb = np.sqrt(self.bias.T @ Q @ self.bias - J)
        
        if verbose:
            print(f"Measured field magnitude: {hmb:.4f}")
        
        # Step 13: Compute calibration matrix
        # We need: A_inv = (hm / hmb) * sqrt(Q)
        # where sqrt(Q) is the matrix square root
        
        # Compute matrix square root via eigendecomposition
        eigenvalues_Q, eigenvectors_Q = np.linalg.eig(Q)
        
        # Normalize eigenvectors
        for i in range(3):
            eigenvectors_Q[:, i] = eigenvectors_Q[:, i] / np.linalg.norm(eigenvectors_Q[:, i])
        
        # Build diagonal matrix of square roots of eigenvalues
        D_sqrt = np.diag(np.sqrt(eigenvalues_Q.real))
        
        # Reconstruct: sqrt(Q) = V @ D_sqrt @ V^T
        sqrt_Q = eigenvectors_Q.real @ D_sqrt @ eigenvectors_Q.real.T
        
        # Apply field strength scaling
        if self.reference_field_strength is None:
            # Use measured field magnitude
            self.reference_field_strength = hmb
            scaling = 1.0
        else:
            scaling = self.reference_field_strength / hmb
            
        self.calibration_matrix = scaling * sqrt_Q
        
        if verbose:
            print(f"Reference field strength: {self.reference_field_strength:.4f}")
            print(f"Scaling factor: {scaling:.4f}")
            print("\nCalibration Matrix:")
            print(self.calibration_matrix)
            
            # Compute calibration quality metrics
            self._compute_quality_metrics(data, verbose=True)
        
        return self.bias, self.calibration_matrix
    
    def _build_design_matrix(self, data: np.ndarray) -> np.ndarray:
        """
        Build design matrix D from sensor measurements.
        
        Each column represents one measurement and contains:
        [x², y², z², 2yz, 2xz, 2xy, 2x, 2y, 2z, 1]
        
        Parameters
        ----------
        data : np.ndarray
            N x 3 array of measurements
            
        Returns
        -------
        D : np.ndarray
            10 x N design matrix
        """
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]
        
        D = np.vstack([
            x * x,          # x²
            y * y,          # y²
            z * z,          # z²
            2.0 * y * z,    # 2yz
            2.0 * x * z,    # 2xz
            2.0 * x * y,    # 2xy
            2.0 * x,        # 2x
            2.0 * y,        # 2y
            2.0 * z,        # 2z
            np.ones_like(x) # 1
        ])
        
        return D
    
    def apply_calibration(self, data: np.ndarray) -> np.ndarray:
        """
        Apply calibration to raw sensor data.
        
        Corrected = A_inv @ (Raw - Bias)
        
        Parameters
        ----------
        data : np.ndarray
            N x 3 array of raw sensor measurements
            
        Returns
        -------
        calibrated : np.ndarray
            N x 3 array of calibrated measurements
        """
        if self.bias is None or self.calibration_matrix is None:
            raise ValueError("Must call fit() before applying calibration")
        
        # Subtract bias
        centered = data - self.bias
        
        # Apply calibration matrix
        calibrated = (self.calibration_matrix @ centered.T).T
        
        return calibrated
    
    def _compute_quality_metrics(self, data: np.ndarray, verbose: bool = True):
        """
        Compute quality metrics for the calibration.
        
        Metrics:
        - Original data: mean magnitude, std deviation
        - Calibrated data: mean magnitude, std deviation, improvement
        """
        # Apply calibration to original data
        calibrated = self.apply_calibration(data)
        
        # Compute magnitudes
        raw_magnitudes = np.linalg.norm(data, axis=1)
        cal_magnitudes = np.linalg.norm(calibrated, axis=1)
        
        # Statistics
        raw_mean = np.mean(raw_magnitudes)
        raw_std = np.std(raw_magnitudes)
        cal_mean = np.mean(cal_magnitudes)
        cal_std = np.std(cal_magnitudes)
        
        improvement = ((raw_std - cal_std) / raw_std) * 100
        
        if verbose:
            print("\n" + "="*60)
            print("CALIBRATION QUALITY METRICS")
            print("="*60)
            print(f"Raw data magnitude:        {raw_mean:.4f} ± {raw_std:.4f}")
            print(f"Calibrated data magnitude: {cal_mean:.4f} ± {cal_std:.4f}")
            print(f"Improvement in std dev:    {improvement:.1f}%")
            print("="*60)
            
            if cal_std / cal_mean > 0.05:  # >5% variation
                print("⚠️  WARNING: High variation in calibrated data!")
                print("   This could indicate:")
                print("   - Insufficient data coverage (rotate sensor in ALL directions)")
                print("   - Non-ellipsoidal errors (temperature drift, nearby metal)")
                print("   - Poor quality sensor data")
    
    def plot_calibration(self, show_plot: bool = True, save_path: Optional[str] = None):
        """
        Visualize raw vs calibrated data in 3D.
        
        Parameters
        ----------
        show_plot : bool
            Whether to display the plot
        save_path : str, optional
            If provided, save plot to this path
        """
        if self.raw_data is None:
            raise ValueError("No data to plot. Call fit() first.")
        
        calibrated = self.apply_calibration(self.raw_data)
        
        fig = plt.figure(figsize=(16, 7))
        
        # Plot 1: Raw data
        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(self.raw_data[:, 0], self.raw_data[:, 1], self.raw_data[:, 2],
                   c='red', marker='o', alpha=0.6, s=20)
        ax1.scatter([self.bias[0]], [self.bias[1]], [self.bias[2]],
                   c='blue', marker='x', s=200, linewidths=3, label='Bias')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        ax1.set_title('Raw Data (Ellipsoid)')
        ax1.legend()
        self._set_equal_aspect_3d(ax1, self.raw_data)
        
        # Plot 2: Calibrated data
        ax2 = fig.add_subplot(122, projection='3d')
        ax2.scatter(calibrated[:, 0], calibrated[:, 1], calibrated[:, 2],
                   c='green', marker='o', alpha=0.6, s=20)
        
        # Draw reference sphere
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 50)
        r = self.reference_field_strength if self.reference_field_strength else 1.0
        x_sphere = r * np.outer(np.cos(u), np.sin(v))
        y_sphere = r * np.outer(np.sin(u), np.sin(v))
        z_sphere = r * np.outer(np.ones(np.size(u)), np.cos(v))
        ax2.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.2, color='cyan')
        
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')
        ax2.set_title('Calibrated Data (Sphere)')
        self._set_equal_aspect_3d(ax2, calibrated)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Plot saved to: {save_path}")
        
        if show_plot:
            plt.show()
    
    def _set_equal_aspect_3d(self, ax, data):
        """Set equal aspect ratio for 3D plot."""
        max_range = np.array([
            data[:, 0].max() - data[:, 0].min(),
            data[:, 1].max() - data[:, 1].min(),
            data[:, 2].max() - data[:, 2].min()
        ]).max() / 2.0
        
        mid_x = (data[:, 0].max() + data[:, 0].min()) * 0.5
        mid_y = (data[:, 1].max() + data[:, 1].min()) * 0.5
        mid_z = (data[:, 2].max() + data[:, 2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    def save_calibration(self, filename: str):
        """
        Save calibration parameters to file.
        
        Parameters
        ----------
        filename : str
            Path to save calibration (numpy .npz format)
        """
        if self.bias is None or self.calibration_matrix is None:
            raise ValueError("No calibration to save. Call fit() first.")
        
        np.savez(filename,
                 bias=self.bias,
                 calibration_matrix=self.calibration_matrix,
                 reference_field_strength=self.reference_field_strength)
        print(f"Calibration saved to: {filename}")
    
    def load_calibration(self, filename: str):
        """
        Load calibration parameters from file.
        
        Parameters
        ----------
        filename : str
            Path to calibration file (.npz format)
        """
        data = np.load(filename)
        self.bias = data['bias']
        self.calibration_matrix = data['calibration_matrix']
        self.reference_field_strength = float(data['reference_field_strength'])
        print(f"Calibration loaded from: {filename}")
        print(f"Bias: {self.bias}")
        print(f"Reference field: {self.reference_field_strength}")


def load_data_from_file(filename: str) -> np.ndarray:
    """
    Load sensor data from text file.
    
    Expected format: tab or space separated x, y, z values
    Example:
        -23.4  45.1  12.7
        -21.2  43.9  15.3
        ...
    
    Parameters
    ----------
    filename : str
        Path to data file
        
    Returns
    -------
    data : np.ndarray
        N x 3 array of measurements
    """
    try:
        # Try to load as space/tab delimited
        data = np.loadtxt(filename)
        
        if data.ndim == 1:
            data = data.reshape(-1, 3)
        
        if data.shape[1] != 3:
            raise ValueError("Data must have 3 columns [x, y, z]")
        
        print(f"Loaded {data.shape[0]} samples from {filename}")
        return data
        
    except Exception as e:
        print(f"Error loading data: {e}")
        print("\nExpected format: text file with 3 columns (x, y, z)")
        print("Example:")
        print("  -23.4  45.1  12.7")
        print("  -21.2  43.9  15.3")
        raise


# Example usage
if __name__ == "__main__":
    import sys
    
    print("="*70)
    print("ELLIPSOID CALIBRATION FOR ACCELEROMETER/MAGNETOMETER")
    print("="*70)
    
    # Example with synthetic data (you'll replace this with your actual data file)
    if True:
    #if len(sys.argv) > 1:
        # Load from file
        #data_file = sys.argv[1]
        data = load_data_from_file(fn)
        
        # Determine sensor type
        sensor_type = input("Sensor type (accelerometer/magnetometer) [a/m]: ").lower()
        
        if sensor_type.startswith('a'):
            # Accelerometer: expect 1g
            reference = 1.0
            print("Calibrating accelerometer (reference = 1g)")
        else:
            # Magnetometer: use measured field or specify
            use_default = input("Use local magnetic field strength? [y/n]: ").lower()
            if use_default.startswith('y'):
                reference = None  # Will use measured value
            else:
                reference = float(input("Enter reference field strength: "))
            print(f"Calibrating magnetometer (reference = {reference})")
    else:
        # Generate synthetic ellipsoid data for demonstration
        print("\nNo data file provided. Generating synthetic data for demonstration...")
        print("Run with: python ellipsoid_calibration.py <your_data_file.txt>\n")
        
        # Generate data
        np.random.seed(42)
        n_samples = 500
        
        # Create sphere
        theta = np.random.uniform(0, 2*np.pi, n_samples)
        phi = np.random.uniform(0, np.pi, n_samples)
        r = 1.0
        
        x = r * np.sin(phi) * np.cos(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(phi)
        
        sphere = np.column_stack([x, y, z])
        
        # Transform to ellipsoid with bias
        # Scaling matrix (soft-iron / sensitivity)
        A = np.array([
            [1.2, 0.1, 0.05],
            [0.1, 0.9, 0.08],
            [0.05, 0.08, 1.1]
        ])
        
        # Bias (hard-iron / zero-g offset)
        bias_true = np.array([0.3, -0.2, 0.15])
        
        # Apply transformation
        ellipsoid = (A @ sphere.T).T + bias_true
        
        # Add noise
        ellipsoid += np.random.normal(0, 0.02, ellipsoid.shape)
        
        data = ellipsoid
        reference = 1.0
        
        print(f"Generated {n_samples} samples")
        print(f"True bias: {bias_true}")
        print(f"True transformation matrix:\n{A}\n")
    
    # Perform calibration
    print("\nStarting calibration...")
    print("-" * 70)
    
    calib = EllipsoidCalibration(reference_field_strength=reference)
    bias, cal_matrix = calib.fit(data, verbose=True)
    
    # Save calibration
    calib.save_calibration('calibration.npz')
    
    # Visualize
    print("\nGenerating visualization...")
    calib.plot_calibration(show_plot=True, save_path='calibration_plot.png')
    
    # Show how to use calibration
    print("\n" + "="*70)
    print("HOW TO USE THE CALIBRATION")
    print("="*70)
    print("\nIn your embedded code or application:")
    print("```")
    print("// For each new reading:")
    print("float raw_x, raw_y, raw_z;  // Read from sensor")
    print("")
    print("// 1. Subtract bias")
    print(f"float centered_x = raw_x - ({bias[0]:.6f});")
    print(f"float centered_y = raw_y - ({bias[1]:.6f});")
    print(f"float centered_z = raw_z - ({bias[2]:.6f});")
    print("")
    print("// 2. Apply calibration matrix")
    print("float calibrated_x = ", end="")
    print(f"{cal_matrix[0,0]:7.4f}*centered_x + {cal_matrix[0,1]:7.4f}*centered_y + {cal_matrix[0,2]:7.4f}*centered_z;")
    print("float calibrated_y = ", end="")
    print(f"{cal_matrix[1,0]:7.4f}*centered_x + {cal_matrix[1,1]:7.4f}*centered_y + {cal_matrix[1,2]:7.4f}*centered_z;")
    print("float calibrated_z = ", end="")
    print(f"{cal_matrix[2,0]:7.4f}*centered_x + {cal_matrix[2,1]:7.4f}*centered_y + {cal_matrix[2,2]:7.4f}*centered_z;")
    print("```")
    print("="*70)
