#!/usr/bin/env python3
"""IMU Visualizer v3 - Real-time 3D orientation with magnetometer calibration monitoring"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import threading
import queue
import json
import time
import pandas as pd
from collections import deque
from dataclasses import dataclass
from typing import Optional

# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class Config:
    serial_port: str = "/dev/cu.usbserial-110"
    baud_rate: int = 115200
    command_interval: float = 0.2
    window_width: int = 1600
    window_height: int = 900
    fps: int = 60
    plot_history: int = 200
    plot_height: int = 50
    plot_margin: int = 3
    bg_color: tuple = (30, 30, 35)
    plot_bg_color: tuple = (20, 20, 25)
    grid_color: tuple = (50, 50, 55)
    text_color: tuple = (200, 200, 200)
    debug: bool = False
    csv_file: str = "imu_debug.csv"

# =============================================================================
# DATA STRUCTURE
# =============================================================================

@dataclass
class IMUData:
    timestamp: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0
    temperature: float = 25.0
    # Raw magnetometer data
    mx_raw: float = 0.0
    my_raw: float = 0.0
    mz_raw: float = 0.0
    mag_raw: float = 0.0
    mag_cal: float = 0.0
    # Auto-calibration status
    mcal_valid: int = 0
    mcal_n: int = 0
    moff_x: float = 0.0
    moff_y: float = 0.0
    moff_z: float = 0.0

# =============================================================================
# SERIAL HANDLER
# =============================================================================

class SerialHandler:
    def __init__(self, config: Config, data_queue: queue.Queue):
        self.config = config
        self.data_queue = data_queue
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.last_command_time = 0
        self.connected = False
        self.debug_data = []
        
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
        # Save debug data to CSV
        if self.config.debug and self.debug_data:
            df = pd.DataFrame(self.debug_data)
            df.to_csv(self.config.csv_file, index=False)
            print(f"Saved {len(self.debug_data)} samples to {self.config.csv_file}")
            
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
            
    def _send_command(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                cmd = json.dumps({"T": 126}) + "\n"
                self.serial_port.write(cmd.encode('utf-8'))
                self.serial_port.flush()
            except Exception as e:
                print(f"Send error: {e}")
                self.connected = False

    def _is_imu_data(self, data: dict) -> bool:
        """Check if this JSON is actually IMU data (has required fields)"""
        if data.get('T') == 1002:
            return True
        return False

    def _parse_imu_data(self, line: str) -> Optional[IMUData]:
        try:
            data = json.loads(line)
            if not self._is_imu_data(data):
                return None
            imu = IMUData(timestamp=time.time())
            
            # Euler angles
            imu.roll = float(data.get('roll', data.get('r', data.get('Roll', 0))))
            imu.pitch = float(data.get('pitch', data.get('p', data.get('Pitch', 0))))
            imu.yaw = float(data.get('yaw', data.get('y', data.get('Yaw', 0))))
            
            # Accelerometer
            imu.ax = float(data.get('ax', data.get('accX', 0)))
            imu.ay = float(data.get('ay', data.get('accY', 0)))
            imu.az = float(data.get('az', data.get('accZ', 0)))
            
            # Gyroscope
            imu.gx = float(data.get('gx', data.get('gyroX', 0)))
            imu.gy = float(data.get('gy', data.get('gyroY', 0)))
            imu.gz = float(data.get('gz', data.get('gyroZ', 0)))
            
            # Calibrated Magnetometer
            imu.mx = float(data.get('mx', data.get('magX', 0)))
            imu.my = float(data.get('my', data.get('magY', 0)))
            imu.mz = float(data.get('mz', data.get('magZ', 0)))
            
            # Temperature
            imu.temperature = float(data.get('temp', data.get('temperature', 25)))

            # Raw magnetometer
            imu.mx_raw = float(data.get('mx_raw', 0))
            imu.my_raw = float(data.get('my_raw', 0))
            imu.mz_raw = float(data.get('mz_raw', 0))
            imu.mag_raw = float(data.get('mag_raw', 0))
            imu.mag_cal = float(data.get('mag_cal', 0))
            
            # Auto-calibration status
            imu.mcal_valid = int(data.get('mcal_valid', 0))
            imu.mcal_n = int(data.get('mcal_n', 0))
            imu.moff_x = float(data.get('moff_x', 0))
            imu.moff_y = float(data.get('moff_y', 0))
            imu.moff_z = float(data.get('moff_z', 0))
            
            # Save raw data for debug
            if self.config.debug:
                self.debug_data.append({
                    'timestamp': imu.timestamp,
                    'roll': imu.roll, 'pitch': imu.pitch, 'yaw': imu.yaw,
                    'ax': imu.ax, 'ay': imu.ay, 'az': imu.az,
                    'gx': imu.gx, 'gy': imu.gy, 'gz': imu.gz,
                    'mx': imu.mx, 'my': imu.my, 'mz': imu.mz,
                    'mx_raw': imu.mx_raw, 'my_raw': imu.my_raw, 'mz_raw': imu.mz_raw,
                    'mag_raw': imu.mag_raw, 'mag_cal': imu.mag_cal,
                    'mcal_valid': imu.mcal_valid, 'mcal_n': imu.mcal_n,
                    'moff_x': imu.moff_x, 'moff_y': imu.moff_y, 'moff_z': imu.moff_z,
                    'temp': imu.temperature
                })
            
            return imu
        except json.JSONDecodeError:
            return None
        except (KeyError, ValueError, TypeError) as e:
            print(f"Parse error: {e}")
            return None
            
    def _run(self):
        buffer = ""
        while self.running:
            if not self.connected:
                if not self._connect():
                    time.sleep(2.0)
                    continue
                    
            current_time = time.time()
            if current_time - self.last_command_time >= self.config.command_interval:
                self._send_command()
                self.last_command_time = current_time
                
            try:
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += chunk.decode('utf-8', errors='ignore')
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            imu_data = self._parse_imu_data(line)
                            if imu_data:
                                self.data_queue.put(imu_data)
            except Exception as e:
                print(f"Read error: {e}")
                self.connected = False
                
            time.sleep(0.001)

# =============================================================================
# 3D RENDERER
# =============================================================================

class Renderer3D:
    def __init__(self, x: int, y: int, width: int, height: int):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
    def update_orientation(self, roll: float, pitch: float, yaw: float):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        
    def render(self, window_height: int):
        glViewport(self.x, window_height - self.y - self.height, self.width, self.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.width / self.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(4, 3, 4, 0, 0, 0, 0, 1, 0)
        glEnable(GL_DEPTH_TEST)
        glClear(GL_DEPTH_BUFFER_BIT)
        
        self._draw_reference_frame()
        
        glRotatef(self.yaw, 0, 1, 0)
        glRotatef(self.pitch, 1, 0, 0)
        glRotatef(self.roll, 0, 0, 1)
        
        self._draw_arrow()
        glDisable(GL_DEPTH_TEST)
        
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
# PLOT RENDERER - TWO COLUMN LAYOUT
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
        
        # Left column: Orientation, Gyro, Accel
        left_configs = [
            ("Roll", (255, 100, 100), "°", -180, 180),
            ("Pitch", (100, 255, 100), "°", -90, 90),
            ("Yaw", (100, 100, 255), "°", -180, 180),
            ("Gyro X", (255, 100, 200), "dps", -5, 5),
            ("Gyro Y", (200, 100, 255), "dps", -5, 5),
            ("Gyro Z", (100, 200, 255), "dps", -5, 5),
            ("Acc X", (255, 200, 100), "mg", -50, 50),
            ("Acc Y", (200, 255, 100), "mg", -50, 50),
            ("Acc Z", (100, 255, 200), "mg", 950, 1010),
        ]
        
        # Right column: Magnetometer raw, calibrated, calibration status
        right_configs = [
            ("Mag X Raw", (255, 150, 150), "µT", -100, 100),
            ("Mag Y Raw", (150, 255, 150), "µT", -100, 100),
            ("Mag Z Raw", (150, 150, 255), "µT", -100, 100),
            ("Mag X Cal", (255, 100, 100), "µT", -50, 50),
            ("Mag Y Cal", (100, 255, 100), "µT", -50, 50),
            ("Mag Z Cal", (100, 100, 255), "µT", -50, 50),
            ("|Mag| Raw", (255, 200, 50), "µT", 0, 100),
            ("|Mag| Cal", (50, 200, 255), "µT", 0, 100),
            ("Temp", (255, 255, 100), "°C", 20, 60),
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
        # Left column values
        left_values = [
            imu_data.roll, imu_data.pitch, imu_data.yaw,
            imu_data.gx, imu_data.gy, imu_data.gz,
            imu_data.ax, imu_data.ay, imu_data.az,
        ]
        
        # Right column values
        right_values = [
            imu_data.mx_raw, imu_data.my_raw, imu_data.mz_raw,
            imu_data.mx, imu_data.my, imu_data.mz,
            imu_data.mag_raw, imu_data.mag_cal,
            imu_data.temperature,
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
        label_width = 75
        
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
                val_text = f"{current:+.1f}"
                val_surface = self.font.render(val_text, True, plot.color)
                surface.blit(val_surface, (x + 3, plot_y + plot_height - 14))
                
    def render(self, surface: pygame.Surface, x: int, y: int, width: int, height: int, imu_data: IMUData):
        if not self.font:
            self.init_font()
            
        col_width = width // 2 - 5
        
        # Left column
        self._render_plot_column(surface, self.left_plots, x, y, col_width)
        
        # Right column
        self._render_plot_column(surface, self.right_plots, x + col_width + 10, y, col_width)
        
        # Calibration status panel at bottom
        status_y = y + len(self.left_plots) * (self.config.plot_height + self.config.plot_margin) + 10
        self._render_calibration_status(surface, x, status_y, width, imu_data)
        
    def _render_calibration_status(self, surface: pygame.Surface, x: int, y: int, width: int, imu_data: IMUData):
        # Background
        pygame.draw.rect(surface, (40, 40, 45), (x, y, width, 60))
        pygame.draw.rect(surface, self.config.grid_color, (x, y, width, 60), 1)
        
        # Calibration valid indicator
        if imu_data.mcal_valid:
            status_color = (50, 255, 50)
            status_text = "CAL VALID"
        else:
            status_color = (255, 100, 50)
            status_text = "CALIBRATING..."
            
        # Big status indicator
        pygame.draw.circle(surface, status_color, (x + 30, y + 30), 15)
        status_label = self.font_large.render(status_text, True, status_color)
        surface.blit(status_label, (x + 55, y + 22))
        
        # Sample count
        samples_text = f"Samples: {imu_data.mcal_n}"
        samples_surface = self.font.render(samples_text, True, self.config.text_color)
        surface.blit(samples_surface, (x + 200, y + 10))
        
        # Current offsets
        offsets_text = f"Offsets: X={imu_data.moff_x:+.0f}  Y={imu_data.moff_y:+.0f}  Z={imu_data.moff_z:+.0f}"
        offsets_surface = self.font.render(offsets_text, True, (150, 200, 255))
        surface.blit(offsets_surface, (x + 200, y + 28))
        
        # Magnetometer health
        mag_status = f"|Mag| Raw={imu_data.mag_raw:.0f}µT  Cal={imu_data.mag_cal:.0f}µT"
        mag_surface = self.font.render(mag_status, True, (255, 200, 100))
        surface.blit(mag_surface, (x + 200, y + 44))
        
        # Yaw value (prominent)
        yaw_text = f"YAW: {imu_data.yaw:+.1f}°"
        yaw_surface = self.font_large.render(yaw_text, True, (100, 150, 255))
        surface.blit(yaw_surface, (x + 450, y + 22))

# =============================================================================
# MAIN APPLICATION
# =============================================================================

class IMUVisualizer:
    def __init__(self, config: Config):
        self.config = config
        self.running = False
        self.data_queue = queue.Queue(maxsize=100)
        self.serial_handler = SerialHandler(config, self.data_queue)
        self.current_imu = IMUData()
        
    def _init_pygame(self):
        pygame.init()
        pygame.display.set_caption("IMU Visualizer v3 - Magnetometer Calibration")
        self.screen = pygame.display.set_mode(
            (self.config.window_width, self.config.window_height),
            DOUBLEBUF | OPENGL
        )
        self.clock = pygame.time.Clock()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        # 3D view on left (40% width)
        view_3d_width = int(self.config.window_width * 0.35)
        self.renderer_3d = Renderer3D(0, 0, view_3d_width, self.config.window_height)
        
        # Plots on right (60% width)
        plot_width = self.config.window_width - view_3d_width
        self.plot_renderer = PlotRenderer(self.config)
        self.plot_surface = pygame.Surface((plot_width, self.config.window_height))
        self.plot_x_offset = view_3d_width
        
    def _process_serial_data(self):
        while not self.data_queue.empty():
            try:
                self.current_imu = self.data_queue.get_nowait()
                self.renderer_3d.update_orientation(
                    self.current_imu.roll,
                    self.current_imu.pitch,
                    self.current_imu.yaw
                )
                self.plot_renderer.update(self.current_imu)
            except queue.Empty:
                break
                
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
        print("IMU Visualizer v3 - Magnetometer Calibration Monitor")
        print("="*60)
        print(f"Serial port: {self.config.serial_port}")
        print(f"Debug CSV: {self.config.csv_file}" if self.config.debug else "Debug: OFF")
        print()
        print("CALIBRATION PROCEDURE:")
        print("  1. Let robot sit stationary for a few seconds")
        print("  2. Slowly rotate robot 360° around Z-axis (yaw)")
        print("  3. Watch for 'CAL VALID' indicator to turn green")
        print("  4. Once valid, yaw drift should be significantly reduced")
        print()
        print("Press ESC or close window to exit")
        print("="*60)
        
        try:
            while self.running:
                for event in pygame.event.get():
                    if event.type == QUIT:
                        self.running = False
                    elif event.type == KEYDOWN:
                        if event.key == K_ESCAPE:
                            self.running = False
                            
                self._process_serial_data()
                self._render()
                self.clock.tick(self.config.fps)
        finally:
            self.serial_handler.stop()
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
        elif arg == "--no-debug":
            config.debug = False
            
    app = IMUVisualizer(config)
    app.run()