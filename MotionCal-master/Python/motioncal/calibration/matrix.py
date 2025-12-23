"""
Matrix manipulation functions

Ports matrix.c - All matrix operations used by the calibration algorithms.
These functions are critical - they must preserve exact C float behavior.
"""

import numpy as np


# Constants
CORRUPTMATRIX = np.float32(0.001)  # column vector modulus limit for rotation matrix
NITERATIONS = 15  # maximum iterations for eigencompute convergence


def f3x3matrixAeqI(A):
    """
    Set 3x3 matrix A to identity matrix

    Ports: f3x3matrixAeqI from matrix.c

    Args:
        A: 3x3 numpy array (modified in place)
    """
    A.fill(0.0)
    A[0, 0] = np.float32(1.0)
    A[1, 1] = np.float32(1.0)
    A[2, 2] = np.float32(1.0)


def fmatrixAeqI(A, rc):
    """
    Set rc x rc matrix A to identity matrix

    Ports: fmatrixAeqI from matrix.c

    Args:
        A: rc x rc numpy array (modified in place)
        rc: size of square matrix
    """
    A.fill(0.0)
    for i in range(rc):
        A[i, i] = np.float32(1.0)


def f3x3matrixAeqScalar(A, scalar):
    """
    Set every entry in 3x3 matrix A to a constant scalar

    Ports: f3x3matrixAeqScalar from matrix.c

    Args:
        A: 3x3 numpy array (modified in place)
        scalar: value to fill matrix with
    """
    A.fill(np.float32(scalar))


def f3x3matrixAeqAxScalar(A, scalar):
    """
    Multiply all elements of 3x3 matrix A by the specified scalar

    Ports: f3x3matrixAeqAxScalar from matrix.c

    Args:
        A: 3x3 numpy array (modified in place)
        scalar: multiplier
    """
    A *= np.float32(scalar)


def f3x3matrixAeqMinusA(A):
    """
    Negate all elements of 3x3 matrix A

    Ports: f3x3matrixAeqMinusA from matrix.c

    Args:
        A: 3x3 numpy array (modified in place)
    """
    A *= np.float32(-1.0)


def f3x3matrixAeqInvSymB(A, B):
    """
    Calculate the symmetric inverse of a symmetric 3x3 matrix B
    Only the on and above diagonal terms in B are used

    Ports: f3x3matrixAeqInvSymB from matrix.c

    Args:
        A: 3x3 numpy array to store inverse (modified in place)
        B: 3x3 symmetric numpy array to invert
    """
    # Calculate useful products
    fB11B22mB12B12 = B[1, 1] * B[2, 2] - B[1, 2] * B[1, 2]
    fB12B02mB01B22 = B[1, 2] * B[0, 2] - B[0, 1] * B[2, 2]
    fB01B12mB11B02 = B[0, 1] * B[1, 2] - B[1, 1] * B[0, 2]

    # Calculate determinant of B
    ftmp = B[0, 0] * fB11B22mB12B12 + B[0, 1] * fB12B02mB01B22 + B[0, 2] * fB01B12mB11B02

    # Set A to inverse of B if determinant is non-zero
    if ftmp != np.float32(0.0):
        ftmp = np.float32(1.0) / ftmp
        A[0, 0] = fB11B22mB12B12 * ftmp
        A[1, 0] = A[0, 1] = fB12B02mB01B22 * ftmp
        A[2, 0] = A[0, 2] = fB01B12mB11B02 * ftmp
        A[1, 1] = (B[0, 0] * B[2, 2] - B[0, 2] * B[0, 2]) * ftmp
        A[2, 1] = A[1, 2] = (B[0, 2] * B[0, 1] - B[0, 0] * B[1, 2]) * ftmp
        A[2, 2] = (B[0, 0] * B[1, 1] - B[0, 1] * B[0, 1]) * ftmp
    else:
        # Provide identity matrix if determinant is zero
        f3x3matrixAeqI(A)


def f3x3matrixDetA(A):
    """
    Calculate the determinant of a 3x3 matrix

    Ports: f3x3matrixDetA from matrix.c

    Args:
        A: 3x3 numpy array

    Returns:
        float32: determinant of A
    """
    X, Y, Z = 0, 1, 2
    return np.float32(
        A[X, X] * (A[Y, Y] * A[Z, Z] - A[Y, Z] * A[Z, Y]) +
        A[X, Y] * (A[Y, Z] * A[Z, X] - A[Y, X] * A[Z, Z]) +
        A[X, Z] * (A[Y, X] * A[Z, Y] - A[Y, Y] * A[Z, X])
    )


def eigencompute(A, eigval, eigvec, n):
    """
    Compute all eigenvalues and eigenvectors of a real symmetric matrix
    using Jacobi rotation method

    Ports: eigencompute from matrix.c

    This is a CRITICAL function for calibration algorithms. It must preserve
    exact C float behavior.

    Args:
        A: 10x10 numpy array containing nxn symmetric matrix in top-left
           (modified on output)
        eigval: array of length n to store eigenvalues (output)
        eigvec: 10x10 numpy array to store normalized eigenvectors (output)
        n: size of matrix (typically 4, 7, or 10)
    """
    # Initialize eigenvectors matrix to identity
    eigvec.fill(0.0)
    for ir in range(n):
        eigvec[ir, ir] = np.float32(1.0)
        # Initialize eigenvalues to diagonal elements of A
        eigval[ir] = A[ir, ir]

    # Jacobi rotation iterations
    ctr = 0
    while ctr < NITERATIONS:
        # Compute residue (sum of absolute values of above-diagonal elements)
        residue = np.float32(0.0)
        for ir in range(n - 1):
            for ic in range(ir + 1, n):
                residue += abs(A[ir, ic])

        # Exit if converged
        if residue <= np.float32(0.0):
            break

        # Perform Jacobi rotations
        for ir in range(n - 1):
            for ic in range(ir + 1, n):
                # Only process non-zero elements
                if abs(A[ir, ic]) > np.float32(0.0):
                    # Calculate cot(2*phi) where phi is Jacobi rotation angle
                    cot2phi = np.float32(0.5) * (eigval[ic] - eigval[ir]) / A[ir, ic]

                    # Calculate tan(phi) with sign correction for smaller solution
                    tanphi = np.float32(1.0) / (abs(cot2phi) + np.sqrt(np.float32(1.0) + cot2phi * cot2phi))
                    if cot2phi < np.float32(0.0):
                        tanphi = -tanphi

                    # Calculate sin(phi) and cos(phi)
                    cosphi = np.float32(1.0) / np.sqrt(np.float32(1.0) + tanphi * tanphi)
                    sinphi = tanphi * cosphi

                    # Calculate tan(phi/2)
                    tanhalfphi = sinphi / (np.float32(1.0) + cosphi)

                    # Update eigenvalues
                    ftmp = tanphi * A[ir, ic]
                    eigval[ir] -= ftmp
                    eigval[ic] += ftmp

                    # Zero out the rotated element
                    A[ir, ic] = np.float32(0.0)

                    # Apply Jacobi rotation to eigenvector matrix
                    for j in range(n):
                        ftmp = eigvec[j, ir]
                        eigvec[j, ir] = ftmp - sinphi * (eigvec[j, ic] + tanhalfphi * ftmp)
                        eigvec[j, ic] = eigvec[j, ic] + sinphi * (ftmp - tanhalfphi * eigvec[j, ic])

                    # Apply Jacobi rotation to matrix A
                    # Elements in rows 0 to ir-1
                    for j in range(ir):
                        ftmp = A[j, ir]
                        A[j, ir] = ftmp - sinphi * (A[j, ic] + tanhalfphi * ftmp)
                        A[j, ic] = A[j, ic] + sinphi * (ftmp - tanhalfphi * A[j, ic])

                    # Elements in rows ir+1 to ic-1
                    for j in range(ir + 1, ic):
                        ftmp = A[ir, j]
                        A[ir, j] = ftmp - sinphi * (A[j, ic] + tanhalfphi * ftmp)
                        A[j, ic] = A[j, ic] + sinphi * (ftmp - tanhalfphi * A[j, ic])

                    # Elements in rows ic+1 to n-1
                    for j in range(ic + 1, n):
                        ftmp = A[ir, j]
                        A[ir, j] = ftmp - sinphi * (A[ic, j] + tanhalfphi * ftmp)
                        A[ic, j] = A[ic, j] + sinphi * (ftmp - tanhalfphi * A[ic, j])

        ctr += 1


def fmatrixAeqInvA(A, iColInd, iRowInd, iPivot, isize):
    """
    Compute inverse of matrix A using Gauss-Jordan elimination
    A is replaced with its inverse on exit

    Ports: fmatrixAeqInvA from matrix.c

    Args:
        A: isize x isize matrix (modified in place to become inverse)
        iColInd: work array of length isize
        iRowInd: work array of length isize
        iPivot: work array of length isize
        isize: size of square matrix
    """
    # Initialize pivot array to 0
    iPivot.fill(0)

    # Main loop over matrix dimensions
    for i in range(isize):
        # Find largest element for pivoting
        largest = np.float32(0.0)
        iPivotRow = iPivotCol = 0

        for j in range(isize):
            if iPivot[j] != 1:
                for k in range(isize):
                    if iPivot[k] == 0:
                        if abs(A[j, k]) >= largest:
                            iPivotRow = j
                            iPivotCol = k
                            largest = np.float32(abs(A[iPivotRow, iPivotCol]))
                    elif iPivot[k] > 1:
                        # Singular matrix: return identity
                        fmatrixAeqI(A, isize)
                        return

        # Mark column as pivoted
        iPivot[iPivotCol] += 1

        # Swap rows if needed
        if iPivotRow != iPivotCol:
            for l in range(isize):
                ftmp = A[iPivotRow, l]
                A[iPivotRow, l] = A[iPivotCol, l]
                A[iPivotCol, l] = ftmp

        # Record row swap
        iRowInd[i] = iPivotRow
        iColInd[i] = iPivotCol

        # Check for singular matrix
        if A[iPivotCol, iPivotCol] == np.float32(0.0):
            fmatrixAeqI(A, isize)
            return

        # Normalize pivot row
        recippiv = np.float32(1.0) / A[iPivotCol, iPivotCol]
        A[iPivotCol, iPivotCol] = np.float32(1.0)
        for l in range(isize):
            A[iPivotCol, l] *= recippiv

        # Eliminate column in other rows
        for m in range(isize):
            if m != iPivotCol:
                scaling = A[m, iPivotCol]
                A[m, iPivotCol] = np.float32(0.0)
                for l in range(isize):
                    A[m, l] -= A[iPivotCol, l] * scaling

    # Apply column swaps in reverse order
    for l in range(isize - 1, -1, -1):
        i = iRowInd[l]
        j = iColInd[l]

        if i != j:
            for k in range(isize):
                ftmp = A[k, i]
                A[k, i] = A[k, j]
                A[k, j] = ftmp


def fmatrixAeqRenormRotA(A):
    """
    Re-orthonormalize a 3x3 rotation matrix

    Ports: fmatrixAeqRenormRotA from matrix.c

    Args:
        A: 3x3 rotation matrix (modified in place)
    """
    X, Y, Z = 0, 1, 2

    # Normalize X column
    ftmp = np.sqrt(A[X, X] * A[X, X] + A[Y, X] * A[Y, X] + A[Z, X] * A[Z, X])
    if ftmp > CORRUPTMATRIX:
        ftmp = np.float32(1.0) / ftmp
        A[X, X] *= ftmp
        A[Y, X] *= ftmp
        A[Z, X] *= ftmp
    else:
        # Set X column to {1, 0, 0}
        A[X, X] = np.float32(1.0)
        A[Y, X] = A[Z, X] = np.float32(0.0)

    # Force Y column orthogonal to X using y = y - (x·y)x
    ftmp = A[X, X] * A[X, Y] + A[Y, X] * A[Y, Y] + A[Z, X] * A[Z, Y]
    A[X, Y] -= ftmp * A[X, X]
    A[Y, Y] -= ftmp * A[Y, X]
    A[Z, Y] -= ftmp * A[Z, X]

    # Normalize Y column
    ftmp = np.sqrt(A[X, Y] * A[X, Y] + A[Y, Y] * A[Y, Y] + A[Z, Y] * A[Z, Y])
    if ftmp > CORRUPTMATRIX:
        ftmp = np.float32(1.0) / ftmp
        A[X, Y] *= ftmp
        A[Y, Y] *= ftmp
        A[Z, Y] *= ftmp
    else:
        # Set Y column to {0, 1, 0}
        A[X, Y] = np.float32(0.0)
        A[Y, Y] = np.float32(1.0)
        A[Z, Y] = np.float32(0.0)

    # Calculate Z column as cross product of X and Y
    A[X, Z] = A[Y, X] * A[Z, Y] - A[Z, X] * A[Y, Y]
    A[Y, Z] = A[Z, X] * A[X, Y] - A[X, X] * A[Z, Y]
    A[Z, Z] = A[X, X] * A[Y, Y] - A[Y, X] * A[X, Y]
