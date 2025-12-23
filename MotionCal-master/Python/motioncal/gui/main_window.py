"""
Main window for MotionCal GUI

Ports gui.cpp - Main application window with all panels and OpenGL visualization
"""

import sys
import numpy as np
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGroupBox, QLabel, QComboBox, QPushButton,
    QTextEdit, QGridLayout, QSizePolicy
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap, QIcon

from ..utils.constants import MAGBUFFSIZE, OVERSAMPLE_RATIO
from ..calibration.data_structures import MagCalibration
from ..calibration.quality import (
    quality_reset, quality_surface_gap_error,
    quality_magnitude_variance_error, quality_wobble_error,
    quality_spherical_fit_error
)
from ..data.raw_data import raw_data, raw_data_reset, cal1_data, cal2_data
from ..fusion.mahony import fusion_init, fusion_update, fusion_read
from ..serial.protocol import ProtocolParser, LineEnding
from ..serial.port_manager import PortManager, enumerate_ports
from ..serial.calibration_sender import send_calibration
from ..visualization.transforms import quaternion_to_euler
from .gl_canvas import GLCanvas


class MainWindow(QMainWindow):
    """
    Main application window

    Ports: MyFrame from gui.cpp
    """

    # Standard baud rates
    BAUD_RATES = [300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400]

    def __init__(self):
        """Initialize main window"""
        super().__init__()

        # Window properties
        self.setWindowTitle("MotionCal - Motion Sensor Calibration")
        self.setMinimumSize(1000, 700)

        # Application state
        self.magcal = MagCalibration()
        self.current_orientation = None
        self.paused = False
        self.can_save = False
        self.calibration_confirmed = False
        
        # For deduplication
        self.last_offsets = None
        self.last_soft_iron = None

        # Serial communication
        self.port_manager = PortManager()
        self.protocol_parser = ProtocolParser()
        self.protocol_parser.on_raw_data = self._on_raw_data
        self.protocol_parser.on_cal1_data = self._on_cal1_data
        self.protocol_parser.on_cal2_data = self._on_cal2_data

        # Calibration confirmation tracking
        self.cal_confirm_needed = [0]  # Bit 0: Cal1, Bit 1: Cal2
        self.cal_data_sent = np.zeros(19, dtype=np.float32)

        # Initialize calibration
        raw_data_reset(self.magcal, fusion_init)

        # Build UI
        self._build_ui()

        # Timer for serial reading (14ms like C++ version)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(14)

        self._log_message("MotionCal Python started")

    def _build_ui(self):
        """Build the user interface"""
        # Central widget with horizontal layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)

        # Left panel (Processing)
        left_panel = self._build_left_panel()
        layout.addWidget(left_panel)

        # Right panel (Calibration)
        right_panel = self._build_right_panel()
        layout.addWidget(right_panel)

    def _build_left_panel(self) -> QGroupBox:
        """Build left panel with OpenGL, connection, actions, data, and messages"""
        panel = QGroupBox("Processing")
        layout = QVBoxLayout()

        # OpenGL canvas for magnetometer visualization
        info_label = QLabel("Ideal calibration is a perfectly centered sphere")
        info_label.setStyleSheet("font-size: 9px; font-style: italic;")
        info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(info_label)

        self.gl_canvas = GLCanvas()
        self.gl_canvas.set_calibration(self.magcal)
        layout.addWidget(self.gl_canvas)

        # Quality metrics
        metrics_layout = QGridLayout()
        layout.addLayout(metrics_layout)

        # Gaps
        metrics_layout.addWidget(QLabel("Gaps"), 0, 0, Qt.AlignmentFlag.AlignCenter)
        self.label_gaps = QLabel("100.0%")
        self.label_gaps.setAlignment(Qt.AlignmentFlag.AlignCenter)
        metrics_layout.addWidget(self.label_gaps, 1, 0)

        # Variance
        metrics_layout.addWidget(QLabel("Variance"), 0, 1, Qt.AlignmentFlag.AlignCenter)
        self.label_variance = QLabel("100.0%")
        self.label_variance.setAlignment(Qt.AlignmentFlag.AlignCenter)
        metrics_layout.addWidget(self.label_variance, 1, 1)

        # Wobble
        metrics_layout.addWidget(QLabel("Wobble"), 0, 2, Qt.AlignmentFlag.AlignCenter)
        self.label_wobble = QLabel("100.0%")
        self.label_wobble.setAlignment(Qt.AlignmentFlag.AlignCenter)
        metrics_layout.addWidget(self.label_wobble, 1, 2)

        # Fit Error
        metrics_layout.addWidget(QLabel("Fit Error"), 0, 3, Qt.AlignmentFlag.AlignCenter)
        self.label_fit = QLabel("100.0%")
        self.label_fit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        metrics_layout.addWidget(self.label_fit, 1, 3)

        # Connection panel
        connection_panel = self._build_connection_panel()
        layout.addWidget(connection_panel)

        # Actions panel
        actions_panel = self._build_actions_panel()
        layout.addWidget(actions_panel)

        # Data panel
        data_panel = self._build_data_panel()
        layout.addWidget(data_panel)

        # Messages panel
        messages_panel = self._build_messages_panel()
        layout.addWidget(messages_panel)

        panel.setLayout(layout)
        return panel

    def _build_connection_panel(self) -> QGroupBox:
        """Build connection panel with port, baud, and line ending selection"""
        panel = QGroupBox("Connection")
        layout = QVBoxLayout()

        # Port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Port"))
        self.combo_port = QComboBox()
        self.combo_port.addItem("(none)")
        self.combo_port.currentTextChanged.connect(self._on_port_changed)
        port_layout.addWidget(self.combo_port)
        layout.addLayout(port_layout)

        # Baud rate selection
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate"))
        self.combo_baud = QComboBox()
        for baud in self.BAUD_RATES:
            self.combo_baud.addItem(str(baud))
        self.combo_baud.setCurrentText("115200")
        self.combo_baud.currentTextChanged.connect(self._on_baud_changed)
        baud_layout.addWidget(self.combo_baud)
        layout.addLayout(baud_layout)

        # Line ending selection
        ending_layout = QHBoxLayout()
        ending_layout.addWidget(QLabel("Line Endings"))
        self.combo_ending = QComboBox()
        self.combo_ending.addItem("(none)")
        self.combo_ending.addItem("LF")
        self.combo_ending.addItem("CR")
        self.combo_ending.addItem("CRLF")
        self.combo_ending.setCurrentText("LF")
        self.combo_ending.currentTextChanged.connect(self._on_ending_changed)
        ending_layout.addWidget(self.combo_ending)
        layout.addLayout(ending_layout)

        panel.setLayout(layout)
        return panel

    def _build_actions_panel(self) -> QGroupBox:
        """Build actions panel with pause, clear, and send buttons"""
        panel = QGroupBox("Actions")
        layout = QHBoxLayout()

        # Status icon placeholder (TODO: use actual images)
        self.label_status = QLabel("●")
        self.label_status.setStyleSheet("color: gray; font-size: 48px;")
        self.label_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.label_status)

        # Buttons column
        buttons_layout = QVBoxLayout()

        # Pause button
        self.button_pause = QPushButton("Pause")
        self.button_pause.setEnabled(False)
        self.button_pause.clicked.connect(self._on_pause_clicked)
        buttons_layout.addWidget(self.button_pause)

        # Clear button
        self.button_clear = QPushButton("Clear")
        self.button_clear.setEnabled(False)
        self.button_clear.clicked.connect(self._on_clear_clicked)
        buttons_layout.addWidget(self.button_clear)

        # Send Calibration button
        self.button_send = QPushButton("Send Calibration")
        self.button_send.setEnabled(False)
        self.button_send.clicked.connect(self._on_send_clicked)
        buttons_layout.addWidget(self.button_send)

        layout.addLayout(buttons_layout)
        panel.setLayout(layout)
        return panel

    def _build_data_panel(self) -> QGroupBox:
        """Build data panel with raw and orientation grids"""
        panel = QGroupBox("Received Data")
        layout = QVBoxLayout()

        # Raw data
        layout.addWidget(QLabel("Raw:"))
        raw_grid = QGridLayout()
        layout.addLayout(raw_grid)

        # Headers
        raw_grid.addWidget(QLabel(""), 0, 0)
        raw_grid.addWidget(QLabel("X"), 0, 1, Qt.AlignmentFlag.AlignRight)
        raw_grid.addWidget(QLabel("Y"), 0, 2, Qt.AlignmentFlag.AlignRight)
        raw_grid.addWidget(QLabel("Z"), 0, 3, Qt.AlignmentFlag.AlignRight)

        # Accelerometer
        raw_grid.addWidget(QLabel("Accel"), 1, 0)
        self.label_accel = [QLabel("0"), QLabel("0"), QLabel("0")]
        for i, label in enumerate(self.label_accel):
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            raw_grid.addWidget(label, 1, i + 1)

        # Gyroscope
        raw_grid.addWidget(QLabel("Gyro"), 2, 0)
        self.label_gyro = [QLabel("0"), QLabel("0"), QLabel("0")]
        for i, label in enumerate(self.label_gyro):
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            raw_grid.addWidget(label, 2, i + 1)

        # Magnetometer
        raw_grid.addWidget(QLabel("Mag"), 3, 0)
        self.label_mag = [QLabel("0"), QLabel("0"), QLabel("0")]
        for i, label in enumerate(self.label_mag):
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            raw_grid.addWidget(label, 3, i + 1)

        # Orientation
        layout.addWidget(QLabel("Orientation:"))
        orient_grid = QGridLayout()
        layout.addLayout(orient_grid)

        orient_grid.addWidget(QLabel("Yaw"), 0, 0, Qt.AlignmentFlag.AlignRight)
        orient_grid.addWidget(QLabel("Pitch"), 0, 1, Qt.AlignmentFlag.AlignRight)
        orient_grid.addWidget(QLabel("Roll"), 0, 2, Qt.AlignmentFlag.AlignRight)

        self.label_orientation = [QLabel("0.0"), QLabel("0.0"), QLabel("0.0")]
        for i, label in enumerate(self.label_orientation):
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            orient_grid.addWidget(label, 1, i)

        panel.setLayout(layout)
        return panel

    def _build_messages_panel(self) -> QGroupBox:
        """Build messages panel for log output"""
        panel = QGroupBox("Messages")
        layout = QVBoxLayout()

        self.text_messages = QTextEdit()
        self.text_messages.setReadOnly(True)
        self.text_messages.setMaximumHeight(100)
        layout.addWidget(self.text_messages)

        panel.setLayout(layout)
        return panel

    def _build_right_panel(self) -> QGroupBox:
        """Build right panel with calibration results"""
        panel = QGroupBox("Calibration")
        layout = QVBoxLayout()

        # Magnetic Offset
        layout.addWidget(QLabel("Magnetic Offset"))
        offset_grid = QGridLayout()
        self.label_mag_offset = []
        for i in range(3):
            label = QLabel("0.00")
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            self.label_mag_offset.append(label)
            offset_grid.addWidget(label, 0, i)
        layout.addLayout(offset_grid)

        # Magnetic Mapping
        layout.addWidget(QLabel("Magnetic Mapping"))
        mapping_grid = QGridLayout()
        self.label_mag_mapping = []
        for i in range(3):
            row = []
            for j in range(3):
                val = "+1.000" if i == j else "+0.000"
                label = QLabel(val)
                label.setAlignment(Qt.AlignmentFlag.AlignRight)
                row.append(label)
                mapping_grid.addWidget(label, i, j)
            self.label_mag_mapping.append(row)
        layout.addLayout(mapping_grid)

        # Magnetic Field
        layout.addWidget(QLabel("Magnetic Field"))
        self.label_mag_field = QLabel("0.00")
        self.label_mag_field.setAlignment(Qt.AlignmentFlag.AlignRight)
        layout.addWidget(self.label_mag_field)

        # Accelerometer (always 0.000 in ASCII mode)
        layout.addWidget(QLabel("Accelerometer"))
        accel_grid = QGridLayout()
        for i in range(3):
            label = QLabel("0.000")
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            accel_grid.addWidget(label, 0, i)
        layout.addLayout(accel_grid)

        # Gyroscope (always 0.000 in ASCII mode)
        layout.addWidget(QLabel("Gyroscope"))
        gyro_grid = QGridLayout()
        for i in range(3):
            label = QLabel("0.000")
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            gyro_grid.addWidget(label, 0, i)
        layout.addLayout(gyro_grid)

        # Warning text
        warning = QLabel(
            "Calibration should be performed after final installation.\n"
            "Presence of magnets and ferrous metals can alter magnetic calibration.\n"
            "Mechanical stress during assembly can alter accelerometer\n"
            "and gyroscope calibration."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("font-size: 9px; font-style: italic;")
        layout.addWidget(warning)

        layout.addStretch()
        panel.setLayout(layout)
        return panel

    def _on_timer(self):
        """Timer callback for reading serial data"""
        if self.port_manager.is_open() and not self.paused:
            # Read available data
            data = self.port_manager.read_available()
            if data:
                self.protocol_parser.parse_bytes(data)

        # Update UI
        self._update_ui()

    def _on_raw_data(self, data: np.ndarray):
        """Callback for Raw: message"""
        # Update raw data display
        for i in range(3):
            self.label_accel[i].setText(str(data[i]))
            self.label_gyro[i].setText(str(data[i + 3]))
            self.label_mag[i].setText(str(data[i + 6]))

        # Process data through calibration
        self.current_orientation = raw_data(
            data, self.magcal,
            fusion_init, fusion_update, fusion_read,
            quality_surface_gap_error
        )

        # Update orientation display
        if self.current_orientation:
            yaw, pitch, roll = quaternion_to_euler(self.current_orientation)
            self.label_orientation[0].setText(f"{yaw:.1f}")
            self.label_orientation[1].setText(f"{pitch:.1f}")
            self.label_orientation[2].setText(f"{roll:.1f}")

    def _on_cal1_data(self, data: np.ndarray):
        """Callback for Cal1: message"""
        cal1_data(data, self.cal_data_sent, self.cal_confirm_needed, self._calibration_confirmed)

    def _on_cal2_data(self, data: np.ndarray):
        """Callback for Cal2: message"""
        cal2_data(data, self.cal_data_sent, self.cal_confirm_needed, self._calibration_confirmed)

    def _calibration_confirmed(self):
        """Callback when calibration is confirmed by device"""
        self.calibration_confirmed = True
        self.label_status.setStyleSheet("color: green; font-size: 48px;")
        self._log_message("Calibration confirmed by device")

    def _update_ui(self):
        """Update UI elements based on current state"""
        # Update quality metrics
        gaps = quality_surface_gap_error()
        variance = quality_magnitude_variance_error()
        wobble = quality_wobble_error()
        fit = quality_spherical_fit_error(self.magcal)

        self.label_gaps.setText(f"{gaps:.1f}%")
        self.label_variance.setText(f"{variance:.1f}%")
        self.label_wobble.setText(f"{wobble:.1f}%")
        self.label_fit.setText(f"{fit:.1f}%")

        # Determine if can save (all metrics good)
        self.can_save = (gaps < 15.0 and variance < 4.5 and wobble < 4.0 and fit < 5.0)

        # Update status icon
        if self.calibration_confirmed:
            self.label_status.setStyleSheet("color: green; font-size: 48px;")
        elif self.can_save:
            self.label_status.setStyleSheet("color: white; font-size: 48px;")
        else:
            self.label_status.setStyleSheet("color: gray; font-size: 48px;")

        # Update button states
        is_connected = self.port_manager.is_open()
        self.button_pause.setEnabled(is_connected)
        self.button_clear.setEnabled(is_connected)
        self.button_send.setEnabled(is_connected and self.can_save)

        # Update calibration display (only if changed)
        new_offsets = (self.magcal.V[0], self.magcal.V[1], self.magcal.V[2])
        if new_offsets != self.last_offsets:
            self.last_offsets = new_offsets
            for i in range(3):
                self.label_mag_offset[i].setText(f"{self.magcal.V[i]:.2f}")
            self.label_mag_field.setText(f"{self.magcal.B:.2f}")

        # Update soft iron matrix (only if changed)
        new_soft_iron = tuple(self.magcal.invW.flatten())
        if new_soft_iron != self.last_soft_iron:
            self.last_soft_iron = new_soft_iron
            for i in range(3):
                for j in range(3):
                    val = self.magcal.invW[i, j]
                    self.label_mag_mapping[i][j].setText(f"{val:+.3f}")

        # Update OpenGL canvas
        if self.current_orientation:
            self.gl_canvas.set_orientation(self.current_orientation)
        self.gl_canvas.set_can_save(self.can_save)
        self.gl_canvas.update()

    def _on_port_changed(self, port_name: str):
        """Handle port selection change"""
        if port_name == "(none)":
            self.port_manager.close()
            self._log_message("Port closed")
            return
        port_name = "/dev/cu.usbserial-110"
        # Get current settings
        baud = int(self.combo_baud.currentText())
        ending_text = self.combo_ending.currentText()
        ending_map = {
            "(none)": LineEnding.NONE,
            "LF": LineEnding.LF,
            "CR": LineEnding.CR,
            "CRLF": LineEnding.CRLF,
        }
        ending = ending_map.get(ending_text, LineEnding.LF)

        # Open port
        if self.port_manager.open(port_name, baud, ending):
            self.protocol_parser.set_line_ending(ending)
            self._log_message(f"Opened {port_name} at {baud} baud")
        else:
            self._log_message(f"Failed to open {port_name}")
            self.combo_port.setCurrentText("(none)")

    def _on_baud_changed(self, baud_text: str):
        """Handle baud rate change"""
        if self.port_manager.is_open():
            # Reopen with new baud rate
            port_name = self.port_manager.port_name
            self._on_port_changed(port_name)

    def _on_ending_changed(self, ending_text: str):
        """Handle line ending change"""
        ending_map = {
            "(none)": LineEnding.NONE,
            "LF": LineEnding.LF,
            "CR": LineEnding.CR,
            "CRLF": LineEnding.CRLF,
        }
        ending = ending_map.get(ending_text, LineEnding.LF)
        self.protocol_parser.set_line_ending(ending)
        
        if self.port_manager.is_open():
            # Reopen with new line ending
            port_name = self.port_manager.port_name
            self._on_port_changed(port_name)

    def _on_pause_clicked(self):
        """Handle pause button click"""
        self.paused = not self.paused
        if self.paused:
            self.button_pause.setText("Capture")
            self._log_message("Data capture paused")
        else:
            self.button_pause.setText("Pause")
            self._log_message("Data capture resumed")

    def _on_clear_clicked(self):
        """Handle clear button click"""
        raw_data_reset(self.magcal, fusion_init)
        self.calibration_confirmed = False
        self._log_message("Calibration data cleared")

    def _on_send_clicked(self):
        """Handle send calibration button click"""
        if not self.can_save:
            return

        # Build calibration packet
        packet = send_calibration(self.magcal, self.cal_data_sent)

        # Send to device
        if self.port_manager.write(packet):
            self.cal_confirm_needed[0] = 3  # Expecting Cal1 and Cal2
            self._log_message("Calibration sent to device")
        else:
            self._log_message("Failed to send calibration")

    def _log_message(self, message: str):
        """Add message to log"""
        self.text_messages.append(message)

    def showEvent(self, event):
        """Handle window show event"""
        super().showEvent(event)
        # Populate port list
        self._refresh_ports()

    def _refresh_ports(self):
        """Refresh available ports list"""
        self.combo_port.clear()
        self.combo_port.addItem("(none)")
        
        for port_name, port_desc in enumerate_ports():
            self.combo_port.addItem(f"{port_name} - {port_desc}", port_name)

    def closeEvent(self, event):
        """Handle window close event"""
        self.timer.stop()
        self.port_manager.close()
        super().closeEvent(event)
