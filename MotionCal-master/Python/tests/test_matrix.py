"""
Unit tests for matrix operations

Tests matrix.py functions against known values
"""

import unittest
import numpy as np
from motioncal.calibration.matrix import (
    f3x3matrixDetA as matrix_determinant_3x3,
    f3x3matrixAeqInvSymB as matrix_inverse_symmetric_3x3,
    eigencompute
)


class TestMatrixOperations(unittest.TestCase):
    """Test matrix operation functions"""

    def test_determinant_identity(self):
        """Test determinant of identity matrix"""
        A = np.eye(3, dtype=np.float32)
        det = matrix_determinant_3x3(A)
        self.assertAlmostEqual(det, 1.0, places=5)

    def test_determinant_simple(self):
        """Test determinant of simple matrix"""
        A = np.array([
            [2.0, 0.0, 0.0],
            [0.0, 3.0, 0.0],
            [0.0, 0.0, 4.0]
        ], dtype=np.float32)
        det = matrix_determinant_3x3(A)
        self.assertAlmostEqual(det, 24.0, places=5)

    def test_inverse_identity(self):
        """Test inverse of identity matrix"""
        A = np.zeros((3, 3), dtype=np.float32)
        B = np.eye(3, dtype=np.float32)
        matrix_inverse_symmetric_3x3(A, B)
        
        # Should be identity
        np.testing.assert_array_almost_equal(A, np.eye(3), decimal=5)

    def test_inverse_diagonal(self):
        """Test inverse of diagonal matrix"""
        A = np.zeros((3, 3), dtype=np.float32)
        B = np.array([
            [2.0, 0.0, 0.0],
            [0.0, 4.0, 0.0],
            [0.0, 0.0, 8.0]
        ], dtype=np.float32)
        matrix_inverse_symmetric_3x3(A, B)
        
        expected = np.array([
            [0.5, 0.0, 0.0],
            [0.0, 0.25, 0.0],
            [0.0, 0.0, 0.125]
        ], dtype=np.float32)
        np.testing.assert_array_almost_equal(A, expected, decimal=5)

    def test_eigencompute_identity(self):
        """Test eigenvalue computation for identity matrix"""
        A = np.eye(3, dtype=np.float32)
        eigval = np.zeros(3, dtype=np.float32)
        eigvec = np.zeros((3, 3), dtype=np.float32)
        
        eigencompute(A, eigval, eigvec, 3)
        
        # All eigenvalues should be 1.0
        expected_eigval = np.ones(3, dtype=np.float32)
        np.testing.assert_array_almost_equal(eigval, expected_eigval, decimal=4)

    def test_eigencompute_diagonal(self):
        """Test eigenvalue computation for diagonal matrix"""
        A = np.array([
            [4.0, 0.0, 0.0],
            [0.0, 9.0, 0.0],
            [0.0, 0.0, 16.0]
        ], dtype=np.float32)
        eigval = np.zeros(3, dtype=np.float32)
        eigvec = np.zeros((3, 3), dtype=np.float32)
        
        eigencompute(A, eigval, eigvec, 3)
        
        # Eigenvalues should be [4, 9, 16] (possibly reordered)
        expected_eigval = np.array([4.0, 9.0, 16.0], dtype=np.float32)
        np.testing.assert_array_almost_equal(
            np.sort(eigval), np.sort(expected_eigval), decimal=4
        )


if __name__ == '__main__':
    unittest.main()
