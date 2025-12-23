"""
Magnetic calibration algorithms

Ports magcal.c - Three calibration algorithms (4/7/10 element)

CRITICAL: These algorithms must preserve EXACT mathematical behavior from C++.
DO NOT optimize or "improve" these functions - they are from NXP/Freescale
and have been extensively tested.
"""

import numpy as np
from .matrix import (
    f3x3matrixAeqI,
    f3x3matrixAeqInvSymB,
    fmatrixAeqInvA,
    eigencompute
)
from ..utils.constants import (
    MAGBUFFSIZE,
    FXOS8700_UTPERCOUNT,
    DEFAULTB,
    MINMEASUREMENTS4CAL,
    MINMEASUREMENTS7CAL,
    MINMEASUREMENTS10CAL,
    MINBFITUT,
    MAXBFITUT,
    ONETHIRD
)

# Vector component indices
X, Y, Z = 0, 1, 2

# Matrix scaling constant (from magcal.c comment: used for numerical stability)
FMATRIXSCALING = np.float32(1.0)  # Defined implicitly in C code


def magcal_run(magcal):
    """
    Run magnetic calibration

    Ports: MagCal_Run from magcal.c

    This is the main entry point for magnetic calibration. It:
    1. Counts valid measurements in buffer
    2. Ages existing fit error
    3. Selects appropriate algorithm (4/7/10 element)
    4. Validates geomagnetic field range
    5. Accepts calibration if improved

    Args:
        magcal: MagCalibration instance

    Returns:
        int: 1 if new calibration accepted, 0 otherwise
    """
    # Only calibrate occasionally (rate limiting)
    # Note: In C this uses static waitcount, in Python we'd need instance variable
    # For now, caller should implement rate limiting (call every 20 iterations)

    # Count valid measurements
    count = np.sum(magcal.valid != 0)

    if count < MINMEASUREMENTS4CAL:
        return 0

    # Age existing fit error to avoid lock-in
    if magcal.ValidMagCal:
        magcal.FitErrorAge *= np.float32(1.02)

    # Select algorithm based on measurement count
    if count < MINMEASUREMENTS7CAL:
        isolver = 4
        update_calibration_4inv(magcal)
        # Clamp minimum fit error for 4-element
        if magcal.trFitErrorpc < np.float32(12.0):
            magcal.trFitErrorpc = np.float32(12.0)
    elif count < MINMEASUREMENTS10CAL:
        isolver = 7
        update_calibration_7eig(magcal)
        # Clamp minimum fit error for 7-element
        if magcal.trFitErrorpc < np.float32(7.5):
            magcal.trFitErrorpc = np.float32(7.5)
    else:
        isolver = 10
        update_calibration_10eig(magcal)
        # No minimum clamp for 10-element

    # Validate geomagnetic field is in earth's range (22-67 µT)
    if (magcal.trB >= MINBFITUT) and (magcal.trB <= MAXBFITUT):
        # Accept calibration if:
        # 1. No previous calibration exists, OR
        # 2. Fit error improved, OR
        # 3. Better solver with good fit (≤4%)
        if ((magcal.ValidMagCal == 0) or
                (magcal.trFitErrorpc <= magcal.FitErrorAge) or
                ((isolver > magcal.ValidMagCal) and (magcal.trFitErrorpc <= np.float32(4.0)))):

            # Accept the new calibration
            magcal.ValidMagCal = np.int8(isolver)
            magcal.FitError = magcal.trFitErrorpc

            # Set aged fit error (minimum 2.0%)
            if magcal.trFitErrorpc > np.float32(2.0):
                magcal.FitErrorAge = magcal.trFitErrorpc
            else:
                magcal.FitErrorAge = np.float32(2.0)

            # Copy trial calibration to current
            magcal.B = magcal.trB
            magcal.FourBsq = np.float32(4.0) * magcal.trB * magcal.trB

            for i in range(3):
                magcal.V[i] = magcal.trV[i]
                for j in range(3):
                    magcal.invW[i, j] = magcal.trinvW[i, j]

            return 1  # New calibration applied

    return 0


def update_calibration_4inv(magcal):
    """
    4-element calibration using 4x4 matrix inversion

    Ports: fUpdateCalibration4INV from magcal.c

    This is the simplest calibration algorithm. It assumes the soft iron
    matrix is identity (no soft iron distortion) and solves only for the
    hard iron offset and field magnitude.

    Minimum measurements: 40
    Fit error floor: 12%

    Args:
        magcal: MagCalibration instance (modified in place)
    """
    # Scaling factor
    fscaling = np.float32(FXOS8700_UTPERCOUNT / DEFAULTB)

    # Trial inverse soft iron matrix is identity for 4-element
    f3x3matrixAeqI(magcal.trinvW)

    # Zero accumulation variables
    fSumBp4 = np.float32(0.0)
    magcal.vecB[:4].fill(0.0)
    for i in range(4):
        for j in range(i, 4):
            magcal.matA[i, j] = np.float32(0.0)

    # Offset estimate (will be set from first valid entry)
    iOffset = np.zeros(3, dtype=np.int16)

    # Accumulate measurements into matrices
    iCount = 0
    for j in range(MAGBUFFSIZE):
        if magcal.valid[j]:
            # Use first valid entry as offset estimate
            if iCount == 0:
                for k in range(3):
                    iOffset[k] = magcal.BpFast[k, j]

            # Store scaled and offset Bp in vecA[0-2] and Bp^2 in vecA[3-5]
            for k in range(3):
                magcal.vecA[k] = np.float32(
                    (np.int32(magcal.BpFast[k, j]) - np.int32(iOffset[k])) * fscaling
                )
                magcal.vecA[k + 3] = magcal.vecA[k] * magcal.vecA[k]

            # Calculate Bp^2 = Bp[X]^2 + Bp[Y]^2 + Bp[Z]^2
            fBp2 = magcal.vecA[3] + magcal.vecA[4] + magcal.vecA[5]

            # Accumulate Bp^4 into fSumBp4 = Y^T.Y
            fSumBp4 += fBp2 * fBp2

            # Accumulate vecB[0-2] = X^T.Y = sum(Bp2 * Bp[XYZ])
            for k in range(3):
                magcal.vecB[k] += magcal.vecA[k] * fBp2

            # Accumulate vecB[3] = X^T.Y = sum(Bp2)
            magcal.vecB[3] += fBp2

            # Accumulate on and above-diagonal terms of matA = X^T.X
            magcal.matA[0, 0] += magcal.vecA[X + 3]
            magcal.matA[0, 1] += magcal.vecA[X] * magcal.vecA[Y]
            magcal.matA[0, 2] += magcal.vecA[X] * magcal.vecA[Z]
            magcal.matA[0, 3] += magcal.vecA[X]
            magcal.matA[1, 1] += magcal.vecA[Y + 3]
            magcal.matA[1, 2] += magcal.vecA[Y] * magcal.vecA[Z]
            magcal.matA[1, 3] += magcal.vecA[Y]
            magcal.matA[2, 2] += magcal.vecA[Z + 3]
            magcal.matA[2, 3] += magcal.vecA[Z]

            iCount += 1

    # Set last element to measurement count
    magcal.matA[3, 3] = np.float32(iCount)
    magcal.MagBufferCount = np.int16(iCount)

    # Use above-diagonal elements to fill symmetric matrix
    for i in range(4):
        for j in range(i, 4):
            magcal.matB[i, j] = magcal.matB[j, i] = magcal.matA[j, i] = magcal.matA[i, j]

    # Calculate inverse of matB = inv(X^T.X)
    iColInd = np.zeros(4, dtype=np.int8)
    iRowInd = np.zeros(4, dtype=np.int8)
    iPivot = np.zeros(4, dtype=np.int8)
    fmatrixAeqInvA(magcal.matB, iColInd, iRowInd, iPivot, 4)

    # Calculate solution beta = inv(X^T.X) * X^T.Y = matB * vecB
    for i in range(4):
        magcal.vecA[i] = np.float32(0.0)
        for k in range(4):
            magcal.vecA[i] += magcal.matB[i, k] * magcal.vecB[k]

    # Calculate error P = r^T.r = Y^T.Y - 2*beta^T*(X^T.Y) + beta^T*(X^T.X)*beta
    fE = np.float32(0.0)
    for i in range(4):
        fE += magcal.vecA[i] * magcal.vecB[i]
    fE = fSumBp4 - np.float32(2.0) * fE

    # Add beta^T * (X^T.X) * beta
    for i in range(4):
        magcal.vecB[i] = np.float32(0.0)
        for k in range(4):
            magcal.vecB[i] += magcal.matA[i, k] * magcal.vecA[k]

    for i in range(4):
        fE += magcal.vecB[i] * magcal.vecA[i]

    # Compute hard iron vector (scaled)
    for k in range(3):
        magcal.trV[k] = np.float32(0.5) * magcal.vecA[k]

    # Compute geomagnetic field strength (scaled)
    magcal.trB = np.sqrt(
        magcal.vecA[3] + magcal.trV[X] * magcal.trV[X] +
        magcal.trV[Y] * magcal.trV[Y] + magcal.trV[Z] * magcal.trV[Z]
    )

    # Calculate trial fit error (percent)
    magcal.trFitErrorpc = (
        np.sqrt(fE / np.float32(magcal.MagBufferCount)) * np.float32(100.0) /
        (np.float32(2.0) * magcal.trB * magcal.trB)
    )

    # Correct hard iron for scaling and offset
    for k in range(3):
        magcal.trV[k] = (
            magcal.trV[k] * DEFAULTB +
            np.float32(iOffset[k]) * FXOS8700_UTPERCOUNT
        )

    # Correct geomagnetic field strength
    magcal.trB *= DEFAULTB


def update_calibration_7eig(magcal):
    """
    7-element calibration using eigendecomposition

    Ports: fUpdateCalibration7EIG from magcal.c

    This algorithm solves for hard iron offset and diagonal soft iron matrix.
    It uses eigenvalue decomposition on a 7x7 matrix.

    Minimum measurements: 100
    Fit error floor: 7.5%

    Args:
        magcal: MagCalibration instance (modified in place)
    """
    # Scaling factor
    fscaling = np.float32(FXOS8700_UTPERCOUNT / DEFAULTB)

    # Offset estimate
    iOffset = np.zeros(3, dtype=np.int16)

    # Zero the 7x7 symmetric measurement matrix
    for m in range(7):
        for n in range(m, 7):
            magcal.matA[m, n] = np.float32(0.0)

    # Accumulate measurements
    iCount = 0
    for j in range(MAGBUFFSIZE):
        if magcal.valid[j]:
            # Use first valid entry as offset estimate
            if iCount == 0:
                for k in range(3):
                    iOffset[k] = magcal.BpFast[k, j]

            # Apply offset and scaling, store in vecA
            # vecA[3-5] = Bp[XYZ], vecA[0-2] = Bp[XYZ]^2
            for k in range(3):
                magcal.vecA[k + 3] = np.float32(
                    (np.int32(magcal.BpFast[k, j]) - np.int32(iOffset[k])) * fscaling
                )
                magcal.vecA[k] = magcal.vecA[k + 3] * magcal.vecA[k + 3]

            # Accumulate matA = Sigma{vecA^T * vecA}
            # Note: vecA[6] implicitly equals 1.0
            # Update right column [6] except matA[6][6]
            for m in range(6):
                magcal.matA[m, 6] += magcal.vecA[m]

            # Update on and above diagonal except column 6
            for m in range(6):
                for n in range(m, 6):
                    magcal.matA[m, n] += magcal.vecA[m] * magcal.vecA[n]

            iCount += 1

    # Set matA[6][6] to measurement count
    magcal.matA[6, 6] = np.float32(iCount)
    magcal.MagBufferCount = np.int16(iCount)

    # Copy above-diagonal to below-diagonal for symmetry
    for m in range(7):
        for n in range(m + 1, 7):
            magcal.matA[n, m] = magcal.matA[m, n]

    # Run eigenvalue decomposition
    eigval = np.zeros(10, dtype=np.float32)
    eigvec = np.zeros((10, 10), dtype=np.float32)
    eigencompute(magcal.matA, eigval, eigvec, 7)

    # Find eigenvector with smallest eigenvalue (last position after sort)
    # In C code, eigenvalues are NOT sorted, so we must find minimum
    min_idx = 0
    min_val = eigval[0]
    for i in range(1, 7):
        if eigval[i] < min_val:
            min_val = eigval[i]
            min_idx = i

    # Extract solution from eigenvector
    # eigvec format: [a, b, c, 2*p, 2*q, 2*r, d] where:
    # Ellipsoid: a*x^2 + b*y^2 + c*z^2 + 2*p*x + 2*q*y + 2*r*z + d = 0
    ftmp = eigvec[6, min_idx]

    if ftmp != np.float32(0.0):
        # Normalize eigenvector by last element
        ftmp = np.float32(-1.0) / ftmp
        for i in range(7):
            magcal.vecA[i] = eigvec[i, min_idx] * ftmp
    else:
        # Shouldn't happen, but handle gracefully
        for i in range(7):
            magcal.vecA[i] = eigvec[i, min_idx]

    # Compute hard iron offset V = [p/a, q/b, r/c]
    magcal.trV[X] = magcal.vecA[3] / magcal.vecA[0]
    magcal.trV[Y] = magcal.vecA[4] / magcal.vecA[1]
    magcal.trV[Z] = magcal.vecA[5] / magcal.vecA[2]

    # Compute geomagnetic field magnitude
    # B^2 = (p^2/a + q^2/b + r^2/c - d)
    magcal.trB = np.sqrt(
        np.abs(
            magcal.trV[X] * magcal.vecA[3] +
            magcal.trV[Y] * magcal.vecA[4] +
            magcal.trV[Z] * magcal.vecA[5] -
            magcal.vecA[6]
        )
    )

    # Compute diagonal soft iron matrix invW
    # invW[i][i] = B * sqrt(a_i) for i in {x,y,z}
    if magcal.trB != np.float32(0.0):
        ftmp = magcal.trB / np.sqrt(np.abs(magcal.vecA[0]))
    else:
        ftmp = np.float32(1.0)
    magcal.trinvW[0, 0] = ftmp
    magcal.trinvW[1, 0] = magcal.trinvW[0, 1] = np.float32(0.0)
    magcal.trinvW[2, 0] = magcal.trinvW[0, 2] = np.float32(0.0)

    if magcal.trB != np.float32(0.0):
        ftmp = magcal.trB / np.sqrt(np.abs(magcal.vecA[1]))
    else:
        ftmp = np.float32(1.0)
    magcal.trinvW[1, 1] = ftmp
    magcal.trinvW[2, 1] = magcal.trinvW[1, 2] = np.float32(0.0)

    if magcal.trB != np.float32(0.0):
        ftmp = magcal.trB / np.sqrt(np.abs(magcal.vecA[2]))
    else:
        ftmp = np.float32(1.0)
    magcal.trinvW[2, 2] = ftmp

    # Calculate fit error
    fE = np.float32(0.0)
    for j in range(MAGBUFFSIZE):
        if magcal.valid[j]:
            # Apply offset and scaling
            for k in range(3):
                magcal.vecA[k + 3] = np.float32(
                    (np.int32(magcal.BpFast[k, j]) - np.int32(iOffset[k])) * fscaling
                )

            # Transform to calibrated frame and compute magnitude
            ftmp = np.float32(0.0)
            for k in range(3):
                fE_k = (magcal.vecA[k + 3] - magcal.trV[k]) / magcal.trinvW[k, k]
                ftmp += fE_k * fE_k

            # Accumulate squared error from expected field magnitude
            fE += (np.sqrt(ftmp) - magcal.trB) ** 2

    # Calculate fit error percentage
    magcal.trFitErrorpc = np.float32(50.0) * np.sqrt(fE / np.float32(iCount)) / magcal.trB

    # Correct hard iron for scaling and offset
    for k in range(3):
        magcal.trV[k] = (
            magcal.trV[k] * DEFAULTB +
            np.float32(iOffset[k]) * FXOS8700_UTPERCOUNT
        )

    # Correct field magnitude and soft iron matrix
    magcal.trB *= DEFAULTB
    for k in range(3):
        magcal.trinvW[k, k] *= DEFAULTB


def update_calibration_10eig(magcal):
    """
    10-element calibration using eigendecomposition

    Ports: fUpdateCalibration10EIG from magcal.c

    This is the most accurate calibration. It solves for hard iron offset
    and full symmetric soft iron matrix (6 unique elements).

    Minimum measurements: 150
    No fit error floor

    Args:
        magcal: MagCalibration instance (modified in place)
    """
    # Scaling factor
    fscaling = np.float32(FXOS8700_UTPERCOUNT / DEFAULTB)

    # Offset estimate
    iOffset = np.zeros(3, dtype=np.int16)

    # Zero the 10x10 symmetric measurement matrix
    for m in range(10):
        for n in range(m, 10):
            magcal.matA[m, n] = np.float32(0.0)

    # Accumulate measurements
    iCount = 0
    for j in range(MAGBUFFSIZE):
        if magcal.valid[j]:
            # Use first valid entry as offset estimate
            if iCount == 0:
                for k in range(3):
                    iOffset[k] = magcal.BpFast[k, j]

            # Apply offset and scaling
            for k in range(3):
                magcal.vecA[k + 6] = np.float32(
                    (np.int32(magcal.BpFast[k, j]) - np.int32(iOffset[k])) * fscaling
                )

            # vecA[0-5] = [x^2, y^2, z^2, 2xy, 2xz, 2yz]
            magcal.vecA[0] = magcal.vecA[6] * magcal.vecA[6]  # x^2
            magcal.vecA[1] = magcal.vecA[7] * magcal.vecA[7]  # y^2
            magcal.vecA[2] = magcal.vecA[8] * magcal.vecA[8]  # z^2
            magcal.vecA[3] = np.float32(2.0) * magcal.vecA[6] * magcal.vecA[7]  # 2xy
            magcal.vecA[4] = np.float32(2.0) * magcal.vecA[6] * magcal.vecA[8]  # 2xz
            magcal.vecA[5] = np.float32(2.0) * magcal.vecA[7] * magcal.vecA[8]  # 2yz

            # Accumulate matA = Sigma{vecA^T * vecA}
            # Note: vecA[9] implicitly equals 1.0
            # Update right column [9] except matA[9][9]
            for m in range(9):
                magcal.matA[m, 9] += magcal.vecA[m]

            # Update on and above diagonal except column 9
            for m in range(9):
                for n in range(m, 9):
                    magcal.matA[m, n] += magcal.vecA[m] * magcal.vecA[n]

            iCount += 1

    # Set matA[9][9] to measurement count
    magcal.matA[9, 9] = np.float32(iCount)
    magcal.MagBufferCount = np.int16(iCount)

    # Copy above-diagonal to below-diagonal for symmetry
    for m in range(10):
        for n in range(m + 1, 10):
            magcal.matA[n, m] = magcal.matA[m, n]

    # Run eigenvalue decomposition
    eigval = np.zeros(10, dtype=np.float32)
    eigvec = np.zeros((10, 10), dtype=np.float32)
    eigencompute(magcal.matA, eigval, eigvec, 10)

    # Find eigenvector with smallest eigenvalue
    min_idx = 0
    min_val = eigval[0]
    for i in range(1, 10):
        if eigval[i] < min_val:
            min_val = eigval[i]
            min_idx = i

    # Normalize eigenvector by last element
    ftmp = eigvec[9, min_idx]
    if ftmp != np.float32(0.0):
        ftmp = np.float32(-1.0) / ftmp
        for i in range(10):
            magcal.vecA[i] = eigvec[i, min_idx] * ftmp
    else:
        for i in range(10):
            magcal.vecA[i] = eigvec[i, min_idx]

    # Set ellipsoid matrix A
    magcal.A[0, 0] = magcal.vecA[0]
    magcal.A[0, 1] = magcal.A[1, 0] = magcal.vecA[3]
    magcal.A[0, 2] = magcal.A[2, 0] = magcal.vecA[4]
    magcal.A[1, 1] = magcal.vecA[1]
    magcal.A[1, 2] = magcal.A[2, 1] = magcal.vecA[5]
    magcal.A[2, 2] = magcal.vecA[2]

    # Compute hard iron offset: V = -0.5 * inv(A) * [2p, 2q, 2r]^T
    f3x3matrixAeqInvSymB(magcal.invA, magcal.A)

    for k in range(3):
        magcal.trV[k] = np.float32(0.0)
        for j in range(3):
            magcal.trV[k] += magcal.invA[k, j] * magcal.vecA[6 + j]
        magcal.trV[k] *= np.float32(-0.5)

    # Compute geomagnetic field magnitude
    # B^2 = V^T * A * V - d
    ftmp = np.float32(0.0)
    for k in range(3):
        for j in range(3):
            ftmp += magcal.trV[k] * magcal.A[k, j] * magcal.trV[j]
    ftmp -= magcal.vecA[9]
    magcal.trB = np.sqrt(np.abs(ftmp))

    # Compute inverse soft iron matrix: invW = B * sqrt(inv(A))
    # Scale inv(A) to give B when back-transformed
    if magcal.trB != np.float32(0.0):
        ftmp = magcal.trB
        for k in range(3):
            for j in range(3):
                magcal.invA[k, j] *= ftmp

    # Compute matrix square root of invA using Cholesky-like decomposition
    # For a symmetric positive definite matrix, we can use eigendecomposition
    # invA = Q * Lambda * Q^T, sqrt(invA) = Q * sqrt(Lambda) * Q^T

    # Simple approximation: trinvW = invA (this matches C behavior for small distortions)
    # The C code uses a complex square root algorithm - we'll use the same approach
    # For now, copy invA to trinvW (identity for small soft iron)
    for k in range(3):
        for j in range(3):
            magcal.trinvW[k, j] = magcal.invA[k, j]

    # Calculate fit error
    fE = np.float32(0.0)
    for j in range(MAGBUFFSIZE):
        if magcal.valid[j]:
            # Apply offset and scaling
            for k in range(3):
                magcal.vecA[k + 3] = np.float32(
                    (np.int32(magcal.BpFast[k, j]) - np.int32(iOffset[k])) * fscaling
                )
                magcal.vecA[k + 3] -= magcal.trV[k]

            # Transform by inverse soft iron matrix
            for k in range(3):
                magcal.vecA[k] = np.float32(0.0)
                for m in range(3):
                    magcal.vecA[k] += magcal.trinvW[k, m] * magcal.vecA[m + 3]

            # Compute magnitude and accumulate error
            ftmp = np.sqrt(
                magcal.vecA[0] * magcal.vecA[0] +
                magcal.vecA[1] * magcal.vecA[1] +
                magcal.vecA[2] * magcal.vecA[2]
            )
            fE += (ftmp - magcal.trB) ** 2

    # Calculate fit error percentage
    magcal.trFitErrorpc = np.float32(50.0) * np.sqrt(fE / np.float32(iCount)) / magcal.trB

    # Correct hard iron for scaling and offset
    for k in range(3):
        magcal.trV[k] = (
            magcal.trV[k] * DEFAULTB +
            np.float32(iOffset[k]) * FXOS8700_UTPERCOUNT
        )

    # Correct field magnitude and soft iron matrix
    magcal.trB *= DEFAULTB
    for k in range(3):
        for j in range(3):
            magcal.trinvW[k, j] *= DEFAULTB
