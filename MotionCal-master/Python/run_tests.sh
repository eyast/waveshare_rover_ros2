#!/bin/bash
# Test runner for MotionCal Python

set -e

echo "=========================================="
echo "MotionCal Python - Test Suite"
echo "=========================================="
echo ""

# Activate conda environment
echo "Activating conda environment: motioncal"
eval "$(conda shell.bash hook)"
conda activate motioncal

echo "Python version:"
python --version
echo ""

echo "Running unit tests..."
echo "=========================================="

# Run all tests with verbose output
python -m unittest discover -s tests -p "test_*.py" -v

echo ""
echo "=========================================="
echo "All tests completed!"
echo "=========================================="
