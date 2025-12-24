#!/usr/bin/env python3
"""IMU Visualizer v4 - MotionCal Format
Parses Raw:, Uni:, and Ori: messages from CustomIMU ESP32 firmware.
Shows 3D orientation and plots sensor data with magnetometer sphere visualization.
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import threading
import queue
import time
import math
import numpy as np
import pandas as pd
from datetime import datetime
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class Config:
    serial_port: str = "/dev/cu.usbserial-110"
    baud_rate: int = 115200
    window_width: int = 1600
    window_height: int = 900
    fps: int = 60
    plot_history: int = 200
    plot_height: int = 45
    plot_margin: int = 3
    bg_color: tuple = (30, 30, 35)
    plot_bg_color: tuple = (20, 20, 25)
    grid_color: tuple = (50, 50, 55)
    text_color: tuple = (200, 200, 200)
    mag_sphere_points: int = 500  # Number of points to keep for mag sphere

# =============================================================================
# MOTIONCAL SCALE FACTORS (from motioncal_output.h)
# =============================================================================

MOTIONCAL_ACCEL_SCALE = 8192.0  # LSB per g
MOTIONCAL_GYRO_SCALE = 16.0     # LSB per dps
MOTIONCAL_MAG_SCALE = 10.0      # LSB per uT

# =============================================================================
# DATA STRUCTURE
# =============================================================================

@dataclass
class IMUData:
    timestamp: float = 0.0
    # Orientation (from Ori: message or computed)
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    # Raw data (from Raw: message) - uncalibrated, scaled integers
    raw_ax: float = 0.0
    raw_ay: float = 0.0
    raw_az: float = 0.0
    raw_gx: float = 0.0
    raw_gy: float = 0.0
    raw_gz: float = 0.0
    raw_mx: float = 0.0
    raw_my: float = 0.0
    raw_mz: float = 0.0
    # Unified data (from Uni: message) - calibrated, physical units
    uni_ax: float = 0.0  # m/s²
    uni_ay: float = 0.0
    uni_az: float = 0.0
    uni_gx: float = 0.0  # rad/s
    uni_gy: float = 0.0
    uni_gz: float = 0.0
    uni_mx: float = 0.0  # uT (calibrated)
    uni_my: float = 0.0
    uni_mz: float = 0.0
    # Derived values
    mag_raw_magnitude: float = 0.0
    mag_cal_magnitude: float = 0.0

# =============================================================================
# SERIAL HANDLER
# =============================================================================

class SerialHandler:
    def __init__(self, config: Config, data_queue: queue.Queue, mag_points_queue: queue.Queue):
        self.config = config
        self.data_queue = data_queue
        self.mag_points_queue = mag_points_queue
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.connected = False
        self.current_data = IMUData()
        self.line_count = 0

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def _connect(self):
        try:
            self.serial_port = serial.Serial(
                port=self.config.serial_port,
                baudrate=self.config.baud_rate,
                timeout=0.1,
                write_timeout=1.0
            )
            self.connected = True
            print(f"Connected to {self.config.serial_port}")
            return True
        except Exception as e:
            self.connected = False
            print(f"Serial connection failed: {e}")
            return False

    def _parse_raw(self, parts: List[str]) -> bool:
        """Parse Raw:ax,ay,az,gx,gy,gz,mx,my,mz"""
        try:
            if len(parts) != 9:
                return False
            # Convert from MotionCal scaled values back to physical units
            self.current_data.raw_ax = float(parts[0]) / MOTIONCAL_ACCEL_SCALE  # g
            self.current_data.raw_ay = float(parts[1]) / MOTIONCAL_ACCEL_SCALE
            self.current_data.raw_az = float(parts[2]) / MOTIONCAL_ACCEL_SCALE
            self.current_data.raw_gx = float(parts[3]) / MOTIONCAL_GYRO_SCALE   # dps
            self.current_data.raw_gy = float(parts[4]) / MOTIONCAL_GYRO_SCALE
            self.current_data.raw_gz = float(parts[5]) / MOTIONCAL_GYRO_SCALE
            self.current_data.raw_mx = float(parts[6]) / MOTIONCAL_MAG_SCALE    # uT
            self.current_data.raw_my = float(parts[7]) / MOTIONCAL_MAG_SCALE
            self.current_data.raw_mz = float(parts[8]) / MOTIONCAL_MAG_SCALE

            # Calculate raw magnetometer magnitude
            self.current_data.mag_raw_magnitude = math.sqrt(
                self.current_data.raw_mx**2 +
                self.current_data.raw_my**2 +
                self.current_data.raw_mz**2
            )

            # Add point to magnetometer sphere visualization
            self.mag_points_queue.put((
                self.current_data.raw_mx,
                self.current_data.raw_my,
                self.current_data.raw_mz
            ))

            return True
        except (ValueError, IndexError) as e:
            return False

    def _parse_uni(self, parts: List[str]) -> bool:
        """Parse Uni:ax,ay,az,gx,gy,gz,mx,my,mz"""
        try:
            if len(parts) != 9:
                return False
            self.current_data.uni_ax = float(parts[0])  # m/s²
            self.current_data.uni_ay = float(parts[1])
            self.current_data.uni_az = float(parts[2])
            self.current_data.uni_gx = float(parts[3])  # rad/s
            self.current_data.uni_gy = float(parts[4])
            self.current_data.uni_gz = float(parts[5])
            self.current_data.uni_mx = float(parts[6])  # uT (calibrated)
            self.current_data.uni_my = float(parts[7])
            self.current_data.uni_mz = float(parts[8])

            # Calculate calibrated magnetometer magnitude
            self.current_data.mag_cal_magnitude = math.sqrt(
                self.current_data.uni_mx**2 +
                self.current_data.uni_my**2 +
                self.current_data.uni_mz**2
            )

            return True
        except (ValueError, IndexError) as e:
            return False

    def _parse_ori(self, parts: List[str]) -> bool:
        """Parse Ori: yaw,pitch,roll"""
        try:
            if len(parts) != 3:
                return False
            self.current_data.yaw = float(parts[0])
            self.current_data.pitch = float(parts[1])
            self.current_data.roll = float(parts[2])
            return True
        except (ValueError, IndexError) as e:
            return False

    def _parse_line(self, line: str) -> bool:
        """Parse a line of data from the ESP32"""
        line = line.strip()
        if not line:
            return False

        self.current_data.timestamp = time.time()
        parsed = False

        if line.startswith("Raw:"):
            parts = line[4:].split(",")
            parsed = self._parse_raw(parts)
        elif line.startswith("Uni:"):
            parts = line[5:].split(",")
            parsed = self._parse_uni(parts)
        elif line.startswith("Ori:") or line.startswith("Ori "):
            # Handle both "Ori:" and "Ori " formats
            if line.startswith("Ori:"):
                parts = line[4:].strip().split(",")
            else:
                parts = line[4:].strip().split(",")
            parsed = self._parse_ori(parts)

        if parsed:
            self.line_count += 1
            # Only queue data periodically to avoid flooding
            if self.line_count % 2 == 0:  # Every other successful parse
                try:
                    # Create a copy of current data
                    data_copy = IMUData(
                        timestamp=self.current_data.timestamp,
                        yaw=self.current_data.yaw,
                        pitch=self.current_data.pitch,
                        roll=self.current_data.roll,
                        raw_ax=self.current_data.raw_ax,
                        raw_ay=self.current_data.raw_ay,
                        raw_az=self.current_data.raw_az,
                        raw_gx=self.current_data.raw_gx,
                        raw_gy=self.current_data.raw_gy,
                        raw_gz=self.current_data.raw_gz,
                        raw_mx=self.current_data.raw_mx,
                        raw_my=self.current_data.raw_my,
                        raw_mz=self.current_data.raw_mz,
                        uni_ax=self.current_data.uni_ax,
                        uni_ay=self.current_data.uni_ay,
                        uni_az=self.current_data.uni_az,
                        uni_gx=self.current_data.uni_gx,
                        uni_gy=self.current_data.uni_gy,
                        uni_gz=self.current_data.uni_gz,
                        uni_mx=self.current_data.uni_mx,
                        uni_my=self.current_data.uni_my,
                        uni_mz=self.current_data.uni_mz,
                        mag_raw_magnitude=self.current_data.mag_raw_magnitude,
                        mag_cal_magnitude=self.current_data.mag_cal_magnitude,
                    )
                    self.data_queue.put_nowait(data_copy)
                except queue.Full:
                    pass

        return parsed

    def _run(self):
        buffer = ""
        while self.running:
            if not self.connected:
                if not self._connect():
                    time.sleep(2.0)
                    continue

            try:
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += chunk.decode('utf-8', errors='ignore')

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self._parse_line(line)
            except Exception as e:
                print(f"Read error: {e}")
                self.connected = False

            time.sleep(0.001)

# =============================================================================
# 3D RENDERER WITH MAGNETOMETER SPHERE
# =============================================================================

class Renderer3D:
    def __init__(self, x: int, y: int, width: int, height: int, config: Config):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.config = config
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.mag_points: deque = deque(maxlen=config.mag_sphere_points)
        self.view_mode = 0  # 0 = orientation, 1 = mag sphere

    def update_orientation(self, roll: float, pitch: float, yaw: float):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def add_mag_point(self, x: float, y: float, z: float):
        self.mag_points.append((x, y, z))

    def toggle_view(self):
        self.view_mode = (self.view_mode + 1) % 2

    def render(self, window_height: int):
        glViewport(self.x, window_height - self.y - self.height, self.width, self.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.width / self.height, 0.1, 500.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        if self.view_mode == 0:
            self._render_orientation()
        else:
            self._render_mag_sphere()

    def _render_orientation(self):
        gluLookAt(4, 3, 4, 0, 0, 0, 0, 1, 0)
        glEnable(GL_DEPTH_TEST)
        glClear(GL_DEPTH_BUFFER_BIT)

        self._draw_reference_frame()

        glRotatef(self.yaw, 0, 1, 0)
        glRotatef(self.pitch, 1, 0, 0)
        glRotatef(self.roll, 0, 0, 1)

        self._draw_arrow()
        glDisable(GL_DEPTH_TEST)

    def _render_mag_sphere(self):
        # Position camera to see magnetometer data
        gluLookAt(150, 100, 150, 0, 0, 0, 0, 1, 0)
        glEnable(GL_DEPTH_TEST)
        glClear(GL_DEPTH_BUFFER_BIT)

        # Draw reference axes
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.3, 0.3); glVertex3f(-100,0,0); glVertex3f(100,0,0)
        glColor3f(0.3, 1.0, 0.3); glVertex3f(0,-100,0); glVertex3f(0,100,0)
        glColor3f(0.3, 0.3, 1.0); glVertex3f(0,0,-100); glVertex3f(0,0,100)
        glEnd()

        # Draw magnetometer points
        glPointSize(3.0)
        glBegin(GL_POINTS)
        for i, (mx, my, mz) in enumerate(self.mag_points):
            # Color from old (dark) to new (bright green)
            age = i / max(len(self.mag_points), 1)
            glColor3f(0.2 + 0.3 * age, 0.5 + 0.5 * age, 0.2 + 0.3 * age)
            glVertex3f(mx, my, mz)
        glEnd()

        # Draw ideal sphere outline (approximate Earth's field ~50uT)
        if len(self.mag_points) > 10:
            # Calculate center and radius from points
            points = np.array(list(self.mag_points))
            center = np.mean(points, axis=0)
            radii = np.linalg.norm(points - center, axis=1)
            avg_radius = np.mean(radii)

            # Draw wireframe sphere at estimated center
            glColor3f(0.3, 0.3, 0.5)
            glLineWidth(1.0)
            self._draw_wireframe_sphere(center[0], center[1], center[2], avg_radius, 16)

        glDisable(GL_DEPTH_TEST)

    def _draw_wireframe_sphere(self, cx, cy, cz, radius, segments):
        # Draw latitude lines
        for i in range(segments // 2 + 1):
            theta = math.pi * i / (segments // 2)
            r = radius * math.sin(theta)
            y = cy + radius * math.cos(theta)
            glBegin(GL_LINE_LOOP)
            for j in range(segments):
                phi = 2 * math.pi * j / segments
                x = cx + r * math.cos(phi)
                z = cz + r * math.sin(phi)
                glVertex3f(x, y, z)
            glEnd()

        # Draw longitude lines
        for j in range(segments):
            phi = 2 * math.pi * j / segments
            glBegin(GL_LINE_STRIP)
            for i in range(segments // 2 + 1):
                theta = math.pi * i / (segments // 2)
                x = cx + radius * math.sin(theta) * math.cos(phi)
                y = cy + radius * math.cos(theta)
                z = cz + radius * math.sin(theta) * math.sin(phi)
                glVertex3f(x, y, z)
            glEnd()

    def _draw_reference_frame(self):
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.3, 0.3); glVertex3f(0,0,0); glVertex3f(2.5,0,0)
        glColor3f(0.3, 1.0, 0.3); glVertex3f(0,0,0); glVertex3f(0,2.5,0)
        glColor3f(0.3, 0.3, 1.0); glVertex3f(0,0,0); glVertex3f(0,0,2.5)
        glEnd()

    def _draw_arrow(self):
        glLineWidth(3.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.8, 0.0)
        glVertex3f(0, 0, -1.0)
        glVertex3f(0, 0, 1.5)
        glEnd()

        glBegin(GL_TRIANGLES)
        glColor3f(1.0, 0.6, 0.0)
        glVertex3f(0, 0, 2.0); glVertex3f(0.3, 0.3, 1.5); glVertex3f(-0.3, 0.3, 1.5)
        glVertex3f(0, 0, 2.0); glVertex3f(-0.3, -0.3, 1.5); glVertex3f(0.3, -0.3, 1.5)
        glVertex3f(0, 0, 2.0); glVertex3f(-0.3, 0.3, 1.5); glVertex3f(-0.3, -0.3, 1.5)
        glVertex3f(0, 0, 2.0); glVertex3f(0.3, -0.3, 1.5); glVertex3f(0.3, 0.3, 1.5)
        glEnd()

        glBegin(GL_QUADS)
        glColor3f(0.2, 0.6, 0.9)
        glVertex3f(0.1, 0, 0); glVertex3f(1.2, 0, 0.3); glVertex3f(1.2, 0, -0.3); glVertex3f(0.1, 0, -0.5)
        glVertex3f(-0.1, 0, 0); glVertex3f(-1.2, 0, 0.3); glVertex3f(-1.2, 0, -0.3); glVertex3f(-0.1, 0, -0.5)
        glEnd()

        glBegin(GL_TRIANGLES)
        glColor3f(0.9, 0.2, 0.2)
        glVertex3f(0, 0, -1.0); glVertex3f(0, 0.8, -0.5); glVertex3f(0, 0, 0)
        glEnd()

# =============================================================================
# PLOT RENDERER
# =============================================================================

@dataclass
class PlotData:
    name: str
    data: deque
    color: tuple
    min_val: float = -1.0
    max_val: float = 1.0
    unit: str = ""
    auto_scale: bool = True

class PlotRenderer:
    def __init__(self, config: Config):
        self.config = config
        self.left_plots: list[PlotData] = []
        self.right_plots: list[PlotData] = []
        self.font: Optional[pygame.font.Font] = None

        # Left column: Raw data (uncalibrated)
        left_configs = [
            ("Raw Acc X", (255, 150, 150), "g", -2, 2),
            ("Raw Acc Y", (150, 255, 150), "g", -2, 2),
            ("Raw Acc Z", (150, 150, 255), "g", -2, 2),
            ("Raw Gyro X", (255, 200, 150), "dps", -250, 250),
            ("Raw Gyro Y", (200, 255, 150), "dps", -250, 250),
            ("Raw Gyro Z", (150, 255, 200), "dps", -250, 250),
            ("Raw Mag X", (255, 100, 100), "uT", -100, 100),
            ("Raw Mag Y", (100, 255, 100), "uT", -100, 100),
            ("Raw Mag Z", (100, 100, 255), "uT", -100, 100),
            ("|Mag| Raw", (255, 200, 50), "uT", 0, 100),
        ]

        # Right column: Unified/calibrated data
        right_configs = [
            ("Cal Acc X", (255, 150, 150), "m/s2", -20, 20),
            ("Cal Acc Y", (150, 255, 150), "m/s2", -20, 20),
            ("Cal Acc Z", (150, 150, 255), "m/s2", -20, 20),
            ("Cal Gyro X", (255, 200, 150), "rad/s", -5, 5),
            ("Cal Gyro Y", (200, 255, 150), "rad/s", -5, 5),
            ("Cal Gyro Z", (150, 255, 200), "rad/s", -5, 5),
            ("Cal Mag X", (255, 100, 100), "uT", -100, 100),
            ("Cal Mag Y", (100, 255, 100), "uT", -100, 100),
            ("Cal Mag Z", (100, 100, 255), "uT", -100, 100),
            ("|Mag| Cal", (50, 200, 255), "uT", 0, 100),
        ]

        for name, color, unit, min_val, max_val in left_configs:
            self.left_plots.append(PlotData(
                name=name,
                data=deque(maxlen=config.plot_history),
                color=color,
                min_val=min_val,
                max_val=max_val,
                unit=unit,
                auto_scale=True
            ))

        for name, color, unit, min_val, max_val in right_configs:
            self.right_plots.append(PlotData(
                name=name,
                data=deque(maxlen=config.plot_history),
                color=color,
                min_val=min_val,
                max_val=max_val,
                unit=unit,
                auto_scale=True
            ))

    def init_font(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('monospace', 11)
        self.font_large = pygame.font.SysFont('monospace', 14, bold=True)

    def update(self, imu_data: IMUData):
        # Left column: Raw values
        left_values = [
            imu_data.raw_ax, imu_data.raw_ay, imu_data.raw_az,
            imu_data.raw_gx, imu_data.raw_gy, imu_data.raw_gz,
            imu_data.raw_mx, imu_data.raw_my, imu_data.raw_mz,
            imu_data.mag_raw_magnitude,
        ]

        # Right column: Unified/calibrated values
        right_values = [
            imu_data.uni_ax, imu_data.uni_ay, imu_data.uni_az,
            imu_data.uni_gx, imu_data.uni_gy, imu_data.uni_gz,
            imu_data.uni_mx, imu_data.uni_my, imu_data.uni_mz,
            imu_data.mag_cal_magnitude,
        ]

        for plot, value in zip(self.left_plots, left_values):
            plot.data.append(value)
            if plot.auto_scale and len(plot.data) > 10:
                data_min = min(plot.data)
                data_max = max(plot.data)
                margin = (data_max - data_min) * 0.1 + 0.1
                plot.min_val = data_min - margin
                plot.max_val = data_max + margin

        for plot, value in zip(self.right_plots, right_values):
            plot.data.append(value)
            if plot.auto_scale and len(plot.data) > 10:
                data_min = min(plot.data)
                data_max = max(plot.data)
                margin = (data_max - data_min) * 0.1 + 0.1
                plot.min_val = data_min - margin
                plot.max_val = data_max + margin

    def _render_plot_column(self, surface: pygame.Surface, plots: list, x: int, y: int, width: int):
        plot_height = self.config.plot_height
        margin = self.config.plot_margin
        label_width = 80

        for i, plot in enumerate(plots):
            plot_y = y + i * (plot_height + margin)
            plot_x = x + label_width
            plot_width = width - label_width - 5

            pygame.draw.rect(surface, self.config.plot_bg_color,
                           (plot_x, plot_y, plot_width, plot_height))
            pygame.draw.rect(surface, self.config.grid_color,
                           (plot_x, plot_y, plot_width, plot_height), 1)

            if plot.min_val < 0 < plot.max_val:
                zero_y = plot_y + plot_height - int(
                    (0 - plot.min_val) / (plot.max_val - plot.min_val) * plot_height
                )
                pygame.draw.line(surface, (60, 60, 65),
                               (plot_x, zero_y), (plot_x + plot_width, zero_y), 1)

            if len(plot.data) > 1:
                points = []
                data_list = list(plot.data)
                for j, value in enumerate(data_list):
                    px = plot_x + int(j / (self.config.plot_history - 1) * plot_width)
                    clamped = max(plot.min_val, min(plot.max_val, value))
                    if plot.max_val != plot.min_val:
                        normalized = (clamped - plot.min_val) / (plot.max_val - plot.min_val)
                    else:
                        normalized = 0.5
                    py = plot_y + plot_height - int(normalized * plot_height)
                    points.append((px, py))
                if len(points) > 1:
                    pygame.draw.lines(surface, plot.color, False, points, 2)

            label = self.font.render(plot.name, True, self.config.text_color)
            surface.blit(label, (x + 3, plot_y + 2))

            if len(plot.data) > 0:
                current = plot.data[-1]
                val_text = f"{current:+.2f}"
                val_surface = self.font.render(val_text, True, plot.color)
                surface.blit(val_surface, (x + 3, plot_y + plot_height - 14))

    def render(self, surface: pygame.Surface, x: int, y: int, width: int, height: int, imu_data: IMUData):
        if not self.font:
            self.init_font()

        col_width = width // 2 - 5

        # Column headers
        header_raw = self.font_large.render("RAW (Uncalibrated)", True, (255, 200, 100))
        header_cal = self.font_large.render("UNIFIED (Calibrated)", True, (100, 200, 255))
        surface.blit(header_raw, (x + 10, y))
        surface.blit(header_cal, (x + col_width + 20, y))

        # Left column
        self._render_plot_column(surface, self.left_plots, x, y + 20, col_width)

        # Right column
        self._render_plot_column(surface, self.right_plots, x + col_width + 10, y + 20, col_width)

        # Orientation status panel at bottom
        status_y = y + 20 + len(self.left_plots) * (self.config.plot_height + self.config.plot_margin) + 10
        self._render_status(surface, x, status_y, width, imu_data)

    def _render_status(self, surface: pygame.Surface, x: int, y: int, width: int, imu_data: IMUData):
        # Background
        pygame.draw.rect(surface, (40, 40, 45), (x, y, width, 60))
        pygame.draw.rect(surface, self.config.grid_color, (x, y, width, 60), 1)

        # Orientation
        ori_text = f"YAW: {imu_data.yaw:+7.2f}   PITCH: {imu_data.pitch:+7.2f}   ROLL: {imu_data.roll:+7.2f}"
        ori_surface = self.font_large.render(ori_text, True, (100, 200, 255))
        surface.blit(ori_surface, (x + 15, y + 10))

        # Magnetometer comparison
        mag_diff = abs(imu_data.mag_cal_magnitude - imu_data.mag_raw_magnitude)
        mag_text = f"|Mag| Raw: {imu_data.mag_raw_magnitude:.1f} uT   Cal: {imu_data.mag_cal_magnitude:.1f} uT   Diff: {mag_diff:.1f} uT"
        mag_color = (100, 255, 100) if mag_diff < 5 else (255, 200, 100)
        mag_surface = self.font.render(mag_text, True, mag_color)
        surface.blit(mag_surface, (x + 15, y + 35))

        # Help text
        help_text = "Press SPACE to toggle 3D view (Orientation / Mag Sphere)"
        help_surface = self.font.render(help_text, True, (150, 150, 150))
        surface.blit(help_surface, (x + 400, y + 35))

# =============================================================================
# MAIN APPLICATION
# =============================================================================

class IMUVisualizer:
    def __init__(self, config: Config):
        self.config = config
        self.running = False
        self.data_queue = queue.Queue(maxsize=100)
        self.mag_points_queue = queue.Queue(maxsize=1000)
        self.serial_handler = SerialHandler(config, self.data_queue, self.mag_points_queue)
        self.current_imu = IMUData()
        self.all_imu_data: List[IMUData] = []  # Store all data for CSV export

    def _init_pygame(self):
        pygame.init()
        pygame.display.set_caption("IMU Visualizer v4 - MotionCal Format")
        self.screen = pygame.display.set_mode(
            (self.config.window_width, self.config.window_height),
            DOUBLEBUF | OPENGL
        )
        self.clock = pygame.time.Clock()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # 3D view on left (35% width)
        view_3d_width = int(self.config.window_width * 0.35)
        self.renderer_3d = Renderer3D(0, 0, view_3d_width, self.config.window_height, self.config)

        # Plots on right (65% width)
        plot_width = self.config.window_width - view_3d_width
        self.plot_renderer = PlotRenderer(self.config)
        self.plot_surface = pygame.Surface((plot_width, self.config.window_height))
        self.plot_x_offset = view_3d_width

    def _process_serial_data(self):
        # Process IMU data
        while not self.data_queue.empty():
            try:
                self.current_imu = self.data_queue.get_nowait()
                self.all_imu_data.append(self.current_imu)  # Store for CSV export
                self.renderer_3d.update_orientation(
                    self.current_imu.roll,
                    self.current_imu.pitch,
                    self.current_imu.yaw
                )
                self.plot_renderer.update(self.current_imu)
            except queue.Empty:
                break

        # Process magnetometer points for sphere
        while not self.mag_points_queue.empty():
            try:
                mx, my, mz = self.mag_points_queue.get_nowait()
                self.renderer_3d.add_mag_point(mx, my, mz)
            except queue.Empty:
                break

    def _save_data_to_csv(self):
        """Save all collected IMU data to a CSV file using pandas."""
        if not self.all_imu_data:
            print("No data to save.")
            return

        # Convert IMUData objects to a list of dictionaries
        data_dicts = []
        for imu in self.all_imu_data:
            data_dicts.append({
                'timestamp': imu.timestamp,
                'yaw': imu.yaw,
                'pitch': imu.pitch,
                'roll': imu.roll,
                'raw_ax': imu.raw_ax,
                'raw_ay': imu.raw_ay,
                'raw_az': imu.raw_az,
                'raw_gx': imu.raw_gx,
                'raw_gy': imu.raw_gy,
                'raw_gz': imu.raw_gz,
                'raw_mx': imu.raw_mx,
                'raw_my': imu.raw_my,
                'raw_mz': imu.raw_mz,
                'uni_ax': imu.uni_ax,
                'uni_ay': imu.uni_ay,
                'uni_az': imu.uni_az,
                'uni_gx': imu.uni_gx,
                'uni_gy': imu.uni_gy,
                'uni_gz': imu.uni_gz,
                'uni_mx': imu.uni_mx,
                'uni_my': imu.uni_my,
                'uni_mz': imu.uni_mz,
                'mag_raw_magnitude': imu.mag_raw_magnitude,
                'mag_cal_magnitude': imu.mag_cal_magnitude,
            })

        df = pd.DataFrame(data_dicts)

        # Generate filename with timestamp
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"imu_data_{timestamp_str}.csv"

        df.to_csv(filename, index=False)
        print(f"Saved {len(self.all_imu_data)} data points to {filename}")

    def _render(self):
        glClearColor(
            self.config.bg_color[0] / 255,
            self.config.bg_color[1] / 255,
            self.config.bg_color[2] / 255,
            1.0
        )
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self.renderer_3d.render(self.config.window_height)

        self.plot_surface.fill(self.config.bg_color)
        plot_width = self.config.window_width - self.plot_x_offset
        self.plot_renderer.render(
            self.plot_surface, 10, 10,
            plot_width - 20, self.config.window_height - 20,
            self.current_imu
        )

        # Connection status
        if self.plot_renderer.font:
            status_color = (100, 255, 100) if self.serial_handler.connected else (255, 100, 100)
            status_text = "Connected" if self.serial_handler.connected else "Disconnected"
            status = self.plot_renderer.font.render(f"Serial: {status_text}", True, status_color)
            self.plot_surface.blit(status, (10, self.config.window_height - 20))

            # View mode indicator
            view_text = "3D View: Orientation" if self.renderer_3d.view_mode == 0 else "3D View: Mag Sphere"
            view_surface = self.plot_renderer.font.render(view_text, True, (200, 200, 100))
            self.plot_surface.blit(view_surface, (150, self.config.window_height - 20))

        self._render_surface_to_gl(self.plot_surface, self.plot_x_offset, 0)
        pygame.display.flip()

    def _render_surface_to_gl(self, surface: pygame.Surface, x: int, y: int):
        texture_data = pygame.image.tostring(surface, 'RGBA', True)
        width, height = surface.get_size()

        glViewport(x, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, width, 0, height, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        texture_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture_id)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
                    GL_RGBA, GL_UNSIGNED_BYTE, texture_data)

        glEnable(GL_TEXTURE_2D)
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(0, 0)
        glTexCoord2f(1, 0); glVertex2f(width, 0)
        glTexCoord2f(1, 1); glVertex2f(width, height)
        glTexCoord2f(0, 1); glVertex2f(0, height)
        glEnd()
        glDisable(GL_TEXTURE_2D)
        glDeleteTextures([texture_id])

    def run(self):
        self._init_pygame()
        self.serial_handler.start()
        self.running = True

        print("="*60)
        print("IMU Visualizer v4 - MotionCal Format")
        print("="*60)
        print(f"Serial port: {self.config.serial_port}")
        print(f"Baud rate: {self.config.baud_rate}")
        print()
        print("Data format expected:")
        print("  Raw:ax,ay,az,gx,gy,gz,mx,my,mz  (uncalibrated, scaled)")
        print("  Uni: ax,ay,az,gx,gy,gz,mx,my,mz  (calibrated, physical units)")
        print("  Ori: yaw,pitch,roll             (orientation in degrees)")
        print()
        print("Controls:")
        print("  SPACE - Toggle 3D view (Orientation / Magnetometer Sphere)")
        print("  ESC   - Exit")
        print("="*60)

        try:
            while self.running:
                for event in pygame.event.get():
                    if event.type == QUIT:
                        self.running = False
                    elif event.type == KEYDOWN:
                        if event.key == K_ESCAPE:
                            self.running = False
                        elif event.key == K_SPACE:
                            self.renderer_3d.toggle_view()

                self._process_serial_data()
                self._render()
                self.clock.tick(self.config.fps)
        finally:
            self.serial_handler.stop()
            self._save_data_to_csv()
            pygame.quit()
            print("IMU Visualizer stopped")

# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    import sys

    config = Config()

    for arg in sys.argv[1:]:
        if arg.startswith("--port="):
            config.serial_port = arg.split("=")[1]
        elif arg.startswith("--baud="):
            config.baud_rate = int(arg.split("=")[1])

    app = IMUVisualizer(config)
    app.run()
