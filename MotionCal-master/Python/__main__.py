"""
MotionCal entry point

Launches the PyQt6 GUI application
"""

import sys


def main():
    """Main entry point for MotionCal application"""
    from PyQt6.QtWidgets import QApplication
    from motioncal.gui.main_window import MainWindow

    print("MotionCal Python v1.0.0")
    print("Motion Sensor Calibration Tool")
    print()

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
