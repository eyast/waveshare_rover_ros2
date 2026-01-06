#!/usr/bin/env python3
"""
IMU Visualizer v6 - Enhanced Edition
=====================================

A modern, clean visualization tool for 9-DOF IMU data with full control
over the enhanced Madgwick filter features.

FEATURES:
- Real-time 3D orientation visualization with improved rendering
- Clean sensor data plots with proper spacing
- Full control panel for new filter features:
  * Instant initialization from sensors
  * Fast convergence mode
  * Adaptive beta (motion-aware)
  * Convergence monitoring

NEW COMMANDS:
- "Init from Sensors" - Instant correct orientation (no convergence wait!)
- "Fast Convergence"  - Start fast alignment mode
- "Adaptive Beta"     - Toggle motion-aware gain adjustment
- "Get Convergence"   - Check filter status

USAGE:
    python imu_viz_v6.py --port=/dev/cu.usbserial-110
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
import json
from datetime import datetime
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Callable

# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class Config:
    serial_port: str = "/dev/cu.usbserial-110"
    baud_rate: int = 115200
    window_width: int = 1400
    window_height: int = 800
    fps: int = 60
    plot_history: int = 300
    
    # Colors - Modern dark theme
    bg_primary: Tuple[int, int, int] = (18, 18, 24)
    bg_secondary: Tuple[int, int, int] = (25, 25, 35)
    bg_tertiary: Tuple[int, int, int] = (32, 32, 45)
    bg_plot: Tuple[int, int, int] = (15, 15, 22)
    
    accent_blue: Tuple[int, int, int] = (66, 135, 245)
    accent_green: Tuple[int, int, int] = (76, 217, 100)
    accent_orange: Tuple[int, int, int] = (255, 159, 64)
    accent_red: Tuple[int, int, int] = (255, 82, 82)
    accent_purple: Tuple[int, int, int] = (156, 106, 222)
    accent_cyan: Tuple[int, int, int] = (64, 224, 208)
    
    text_primary: Tuple[int, int, int] = (240, 240, 245)
    text_secondary: Tuple[int, int, int] = (160, 160, 175)
    text_muted: Tuple[int, int, int] = (100, 100, 115)
    
    border_color: Tuple[int, int, int] = (55, 55, 70)
    grid_color: Tuple[int, int, int] = (40, 40, 55)

# =============================================================================
# JSON COMMAND TYPES (matching config.h on ESP32)
# =============================================================================

CMD_IMU_STREAM_CTRL = 325
CMD_STREAM_FORMAT = 400
CMD_IMU_CALIBRATE_GYRO = 330
CMD_IMU_CALIBRATE_ACCEL = 331
CMD_IMU_CALIBRATE_ALL = 332
CMD_IMU_FILTER_RESET = 335
CMD_IMU_SET_BETA = 336
CMD_IMU_SET_MAG_SIGNS = 340
CMD_IMU_DEBUG = 345
CMD_IMU_GET_STATUS = 346
CMD_IMU_GET_ORIENTATION = 350

# NEW filter commands
CMD_IMU_INIT_FROM_SENSORS = 351
CMD_IMU_FAST_CONVERGENCE = 352
CMD_IMU_ADAPTIVE_BETA = 353
CMD_IMU_CONVERGENCE_STATUS = 354

# Scale factors for MotionCal format
MOTIONCAL_ACCEL_SCALE = 8192.0
MOTIONCAL_GYRO_SCALE = 16.0
MOTIONCAL_MAG_SCALE = 10.0

# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class IMUData:
    timestamp: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    accel: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gyro: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mag: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mag_magnitude: float = 0.0
    mag_dip: float = 0.0

@dataclass
class FilterStatus:
    convergence_rate: float = 1.0
    converged: bool = False
    active_beta: float = 0.1
    base_beta: float = 0.1
    fast_mode: bool = False
    adaptive: bool = False

# =============================================================================
# MODERN UI COMPONENTS
# =============================================================================

class ModernButton:
    """A modern, clean button with hover effects and optional icon"""
    
    def __init__(self, x: int, y: int, width: int, height: int, text: str,
                 callback: Callable, color: Tuple[int, int, int] = None,
                 icon: str = None, tooltip: str = None):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.callback = callback
        self.base_color = color or (50, 50, 65)
        self.hover_color = tuple(min(255, c + 20) for c in self.base_color)
        self.active_color = tuple(min(255, c + 35) for c in self.base_color)
        self.icon = icon
        self.tooltip = tooltip
        self.hovered = False
        self.pressed = False
        self.enabled = True
        
    def update(self, mouse_pos: Tuple[int, int], mouse_pressed: bool):
        self.hovered = self.rect.collidepoint(mouse_pos) if self.enabled else False
        
    def draw(self, surface: pygame.Surface, font: pygame.font.Font):
        if not self.enabled:
            color = (35, 35, 45)
            text_color = (80, 80, 90)
        elif self.pressed:
            color = self.active_color
            text_color = (255, 255, 255)
        elif self.hovered:
            color = self.hover_color
            text_color = (255, 255, 255)
        else:
            color = self.base_color
            text_color = (200, 200, 210)
        
        # Draw button with rounded corners effect
        pygame.draw.rect(surface, color, self.rect, border_radius=6)
        
        # Subtle border
        border_color = tuple(min(255, c + 30) for c in color)
        pygame.draw.rect(surface, border_color, self.rect, 1, border_radius=6)
        
        # Text
        text_surface = font.render(self.text, True, text_color)
        text_rect = text_surface.get_rect(center=self.rect.center)
        surface.blit(text_surface, text_rect)
        
    def handle_click(self, pos: Tuple[int, int]) -> bool:
        if self.rect.collidepoint(pos) and self.enabled:
            self.callback()
            return True
        return False


class ToggleButton(ModernButton):
    """A toggle button that shows on/off state"""
    
    def __init__(self, x: int, y: int, width: int, height: int, 
                 text_on: str, text_off: str, callback: Callable,
                 initial_state: bool = False):
        super().__init__(x, y, width, height, text_off, callback)
        self.text_on = text_on
        self.text_off = text_off
        self.state = initial_state
        self._update_appearance()
        
    def _update_appearance(self):
        self.text = self.text_on if self.state else self.text_off
        if self.state:
            self.base_color = (40, 100, 60)
            self.hover_color = (50, 120, 70)
        else:
            self.base_color = (70, 50, 50)
            self.hover_color = (90, 60, 60)
    
    def toggle(self):
        self.state = not self.state
        self._update_appearance()
        
    def set_state(self, state: bool):
        self.state = state
        self._update_appearance()


class Slider:
    """A modern slider control"""
    
    def __init__(self, x: int, y: int, width: int, height: int,
                 min_val: float, max_val: float, initial: float,
                 label: str, callback: Callable, format_str: str = "{:.2f}"):
        self.rect = pygame.Rect(x, y, width, height)
        self.track_rect = pygame.Rect(x, y + height//2 - 3, width, 6)
        self.min_val = min_val
        self.max_val = max_val
        self.value = initial
        self.label = label
        self.callback = callback
        self.format_str = format_str
        self.dragging = False
        self.hovered = False
        
    def update(self, mouse_pos: Tuple[int, int], mouse_pressed: bool):
        self.hovered = self.rect.collidepoint(mouse_pos)
        
        if mouse_pressed and self.hovered:
            self.dragging = True
            
        if self.dragging:
            if mouse_pressed:
                rel = (mouse_pos[0] - self.track_rect.x) / self.track_rect.width
                rel = max(0, min(1, rel))
                self.value = self.min_val + rel * (self.max_val - self.min_val)
            else:
                self.dragging = False
                self.callback(self.value)
                
    def draw(self, surface: pygame.Surface, font: pygame.font.Font):
        # Label
        label_text = f"{self.label}: {self.format_str.format(self.value)}"
        label_surface = font.render(label_text, True, (180, 180, 190))
        surface.blit(label_surface, (self.rect.x, self.rect.y - 16))
        
        # Track background
        pygame.draw.rect(surface, (35, 35, 45), self.track_rect, border_radius=3)
        
        # Fill
        fill_width = int((self.value - self.min_val) / (self.max_val - self.min_val) * self.track_rect.width)
        fill_rect = pygame.Rect(self.track_rect.x, self.track_rect.y, fill_width, self.track_rect.height)
        pygame.draw.rect(surface, (66, 135, 245), fill_rect, border_radius=3)
        
        # Handle
        handle_x = self.track_rect.x + fill_width
        handle_color = (100, 160, 255) if self.hovered or self.dragging else (80, 140, 230)
        pygame.draw.circle(surface, handle_color, (handle_x, self.track_rect.centery), 8)
        pygame.draw.circle(surface, (255, 255, 255), (handle_x, self.track_rect.centery), 4)


class StatusIndicator:
    """A small status light indicator"""
    
    def __init__(self, x: int, y: int, label: str):
        self.x = x
        self.y = y
        self.label = label
        self.state = False
        self.color_on = (76, 217, 100)
        self.color_off = (80, 80, 90)
        
    def draw(self, surface: pygame.Surface, font: pygame.font.Font):
        color = self.color_on if self.state else self.color_off
        
        # Glow effect when on
        if self.state:
            for i in range(3):
                alpha = 100 - i * 30
                glow_surf = pygame.Surface((20, 20), pygame.SRCALPHA)
                pygame.draw.circle(glow_surf, (*color, alpha), (10, 10), 8 + i*2)
                surface.blit(glow_surf, (self.x - 10 + 4, self.y - 10 + 4))
        
        pygame.draw.circle(surface, color, (self.x + 4, self.y + 4), 5)
        pygame.draw.circle(surface, (255, 255, 255, 100), (self.x + 4, self.y + 4), 5, 1)
        
        label_surface = font.render(self.label, True, (160, 160, 175))
        surface.blit(label_surface, (self.x + 14, self.y - 2))


# =============================================================================
# SERIAL HANDLER
# =============================================================================

class SerialHandler:
    def __init__(self, config: Config, data_queue: queue.Queue, 
                 response_queue: queue.Queue, status_queue: queue.Queue):
        self.config = config
        self.data_queue = data_queue
        self.response_queue = response_queue
        self.status_queue = status_queue
        self.serial_port = None
        self.running = False
        self.thread = None
        self.connected = False
        self.current_data = IMUData()
        self.line_count = 0
        self.command_queue = queue.Queue()
        self.bytes_received = 0
        self.last_data_time = 0

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

    def send_command(self, cmd: dict):
        self.command_queue.put(cmd)

    def _connect(self) -> bool:
        try:
            self.serial_port = serial.Serial(
                port=self.config.serial_port,
                baudrate=self.config.baud_rate,
                timeout=0.1
            )
            self.connected = True
            print(f"✓ Connected to {self.config.serial_port}")
            return True
        except Exception as e:
            self.connected = False
            return False

    def _parse_line(self, line: str) -> bool:
        line = line.strip()
        if not line:
            return False

        self.current_data.timestamp = time.time()
        self.last_data_time = time.time()
        parsed = False

        # Parse Raw: format (MotionCal)
        if line.startswith("Raw:"):
            try:
                parts = line[4:].split(",")
                if len(parts) == 9:
                    self.current_data.accel = [
                        float(parts[0]) / MOTIONCAL_ACCEL_SCALE,
                        float(parts[1]) / MOTIONCAL_ACCEL_SCALE,
                        float(parts[2]) / MOTIONCAL_ACCEL_SCALE
                    ]
                    self.current_data.gyro = [
                        float(parts[3]) / MOTIONCAL_GYRO_SCALE,
                        float(parts[4]) / MOTIONCAL_GYRO_SCALE,
                        float(parts[5]) / MOTIONCAL_GYRO_SCALE
                    ]
                    self.current_data.mag = [
                        float(parts[6]) / MOTIONCAL_MAG_SCALE,
                        float(parts[7]) / MOTIONCAL_MAG_SCALE,
                        float(parts[8]) / MOTIONCAL_MAG_SCALE
                    ]
                    mx, my, mz = self.current_data.mag
                    self.current_data.mag_magnitude = math.sqrt(mx*mx + my*my + mz*mz)
                    mag_h = math.sqrt(mx*mx + my*my)
                    self.current_data.mag_dip = math.degrees(math.atan2(mz, mag_h)) if mag_h > 0.1 else 0
                    parsed = True
            except (ValueError, IndexError):
                pass
                
        # Parse Ori: format
        elif line.startswith("Ori:") or line.startswith("Ori "):
            try:
                parts = line[4:].strip().split(",")
                if len(parts) == 3:
                    self.current_data.yaw = float(parts[0])
                    self.current_data.pitch = float(parts[1])
                    self.current_data.roll = float(parts[2])
                    parsed = True
            except (ValueError, IndexError):
                pass
                
        # Parse JSON responses
        elif line.startswith("{"):
            try:
                data = json.loads(line)
                
                # Handle filter status specially
                if data.get("T") == CMD_IMU_CONVERGENCE_STATUS:
                    status = FilterStatus(
                        convergence_rate=data.get("rate", 1.0),
                        converged=data.get("converged", False),
                        active_beta=data.get("active_beta", 0.1),
                        base_beta=data.get("base_beta", 0.1),
                        fast_mode=data.get("fast_mode", False),
                        adaptive=data.get("adaptive", False)
                    )
                    try:
                        self.status_queue.put_nowait(status)
                    except queue.Full:
                        pass
                
                # Put all responses in queue for control panel
                try:
                    self.response_queue.put_nowait(data)
                except queue.Full:
                    pass
                
                # Also parse IMU stream data if present
                if data.get("T") == "imu":
                    self.current_data.yaw = data.get("yaw", 0)
                    self.current_data.pitch = data.get("pitch", 0)
                    self.current_data.roll = data.get("roll", 0)
                    if "a" in data:
                        self.current_data.accel = data["a"]
                    if "g" in data:
                        self.current_data.gyro = data["g"]
                    if "m" in data:
                        self.current_data.mag = data["m"]
                        mx, my, mz = self.current_data.mag
                        self.current_data.mag_magnitude = math.sqrt(mx*mx + my*my + mz*mz)
                    parsed = True
            except json.JSONDecodeError:
                pass

        if parsed:
            self.line_count += 1
            if self.line_count % 2 == 0:  # Throttle to 25Hz
                try:
                    self.data_queue.put_nowait(IMUData(
                        timestamp=self.current_data.timestamp,
                        yaw=self.current_data.yaw,
                        pitch=self.current_data.pitch,
                        roll=self.current_data.roll,
                        accel=self.current_data.accel.copy(),
                        gyro=self.current_data.gyro.copy(),
                        mag=self.current_data.mag.copy(),
                        mag_magnitude=self.current_data.mag_magnitude,
                        mag_dip=self.current_data.mag_dip
                    ))
                except queue.Full:
                    pass
        return parsed

    def _run(self):
        buffer = ""
        reconnect_delay = 1.0
        
        while self.running:
            if not self.connected:
                if not self._connect():
                    time.sleep(reconnect_delay)
                    reconnect_delay = min(reconnect_delay * 1.5, 10.0)
                    continue
                reconnect_delay = 1.0

            try:
                # Send queued commands
                while not self.command_queue.empty():
                    cmd = self.command_queue.get_nowait()
                    cmd_str = json.dumps(cmd) + "\n"
                    self.serial_port.write(cmd_str.encode())
                    print(f">>> {cmd_str.strip()}")

                # Read incoming data
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    self.bytes_received += len(chunk)
                    buffer += chunk.decode('utf-8', errors='ignore')
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self._parse_line(line)
                        
            except Exception as e:
                print(f"Serial error: {e}")
                self.connected = False

            time.sleep(0.001)


# =============================================================================
# 3D RENDERER - Enhanced
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
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.smoothing = 0.3

    def update_orientation(self, roll: float, pitch: float, yaw: float):
        self.target_roll = roll
        self.target_pitch = pitch
        self.target_yaw = yaw
        
        # Smooth interpolation
        self.roll += (self.target_roll - self.roll) * self.smoothing
        self.pitch += (self.target_pitch - self.pitch) * self.smoothing
        
        # Handle yaw wraparound
        diff = self.target_yaw - self.yaw
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        self.yaw += diff * self.smoothing

    def render(self, window_height: int):
        glViewport(self.x, window_height - self.y - self.height, self.width, self.height)
        
        # Clear with gradient-like effect
        glClearColor(0.07, 0.07, 0.09, 1.0)
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(40, self.width / self.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        gluLookAt(5, 3.5, 5, 0, 0, 0, 0, 1, 0)
        glEnable(GL_DEPTH_TEST)
        glClear(GL_DEPTH_BUFFER_BIT)

        # Draw grid floor
        self._draw_grid()

        # Draw reference axes (world frame)
        self._draw_axes(2.5, 2.0)

        # Apply rotation (device frame)
        glRotatef(self.yaw, 0, 1, 0)
        glRotatef(self.pitch, 1, 0, 0)
        glRotatef(self.roll, 0, 0, 1)

        # Rotate model 90° so forward aligns with X axis
        glRotatef(90, 0, 1, 0)

        # Draw the device representation
        self._draw_device()

        glDisable(GL_DEPTH_TEST)

    def _draw_grid(self):
        """Draw a subtle reference grid"""
        glLineWidth(1.0)
        glBegin(GL_LINES)
        glColor4f(0.2, 0.2, 0.25, 0.5)
        
        grid_size = 3
        for i in range(-grid_size, grid_size + 1):
            glVertex3f(i, -0.01, -grid_size)
            glVertex3f(i, -0.01, grid_size)
            glVertex3f(-grid_size, -0.01, i)
            glVertex3f(grid_size, -0.01, i)
        glEnd()

    def _draw_axes(self, length: float, line_width: float):
        """Draw world reference axes"""
        glLineWidth(line_width)
        glBegin(GL_LINES)
        
        # X axis - Red
        glColor3f(0.9, 0.3, 0.3)
        glVertex3f(0, 0, 0)
        glVertex3f(length, 0, 0)
        
        # Y axis - Green  
        glColor3f(0.3, 0.9, 0.3)
        glVertex3f(0, 0, 0)
        glVertex3f(0, length, 0)
        
        # Z axis - Blue
        glColor3f(0.3, 0.5, 0.9)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, length)
        
        glEnd()
        
        # Axis labels would go here with text rendering

    def _draw_device(self):
        """Draw a representation of the device (rover-like shape)"""
        # Main body - flat box representing the rover
        body_length = 1.4
        body_width = 0.9
        body_height = 0.25
        
        # Body
        glColor3f(0.3, 0.3, 0.35)
        self._draw_box(0, 0, 0, body_length, body_height, body_width)
        
        # Top surface accent
        glColor3f(0.25, 0.25, 0.3)
        self._draw_box(0, body_height/2 + 0.02, 0, body_length * 0.9, 0.04, body_width * 0.8)
        
        # Direction arrow on top
        glLineWidth(4.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.7, 0.2)
        glVertex3f(0, body_height/2 + 0.08, -0.3)
        glVertex3f(0, body_height/2 + 0.08, 0.5)
        glEnd()
        
        # Arrow head
        glBegin(GL_TRIANGLES)
        glColor3f(1.0, 0.6, 0.1)
        glVertex3f(0, body_height/2 + 0.08, 0.7)
        glVertex3f(0.15, body_height/2 + 0.08, 0.45)
        glVertex3f(-0.15, body_height/2 + 0.08, 0.45)
        glEnd()
        
        # Wheels
        wheel_radius = 0.2
        wheel_width = 0.1
        wheel_positions = [
            (-body_length/2 + 0.2, -body_height/2, body_width/2 + wheel_width/2),
            (-body_length/2 + 0.2, -body_height/2, -body_width/2 - wheel_width/2),
            (body_length/2 - 0.2, -body_height/2, body_width/2 + wheel_width/2),
            (body_length/2 - 0.2, -body_height/2, -body_width/2 - wheel_width/2),
        ]
        
        glColor3f(0.2, 0.2, 0.22)
        for x, y, z in wheel_positions:
            self._draw_cylinder(x, y, z, wheel_radius, wheel_width)

    def _draw_box(self, x: float, y: float, z: float, 
                  length: float, height: float, width: float):
        """Draw a box centered at (x, y, z)"""
        l, h, w = length/2, height/2, width/2
        
        glBegin(GL_QUADS)
        # Top
        glVertex3f(x-l, y+h, z-w); glVertex3f(x+l, y+h, z-w)
        glVertex3f(x+l, y+h, z+w); glVertex3f(x-l, y+h, z+w)
        # Bottom
        glVertex3f(x-l, y-h, z-w); glVertex3f(x-l, y-h, z+w)
        glVertex3f(x+l, y-h, z+w); glVertex3f(x+l, y-h, z-w)
        # Front
        glVertex3f(x-l, y-h, z+w); glVertex3f(x-l, y+h, z+w)
        glVertex3f(x+l, y+h, z+w); glVertex3f(x+l, y-h, z+w)
        # Back
        glVertex3f(x-l, y-h, z-w); glVertex3f(x+l, y-h, z-w)
        glVertex3f(x+l, y+h, z-w); glVertex3f(x-l, y+h, z-w)
        # Left
        glVertex3f(x-l, y-h, z-w); glVertex3f(x-l, y+h, z-w)
        glVertex3f(x-l, y+h, z+w); glVertex3f(x-l, y-h, z+w)
        # Right
        glVertex3f(x+l, y-h, z-w); glVertex3f(x+l, y-h, z+w)
        glVertex3f(x+l, y+h, z+w); glVertex3f(x+l, y+h, z-w)
        glEnd()

    def _draw_cylinder(self, x: float, y: float, z: float, 
                       radius: float, width: float):
        """Draw a simple cylinder (wheel) at position"""
        segments = 12
        glBegin(GL_QUAD_STRIP)
        for i in range(segments + 1):
            angle = 2 * math.pi * i / segments
            dx = math.cos(angle) * radius
            dy = math.sin(angle) * radius
            glVertex3f(x + dx, y + dy, z - width/2)
            glVertex3f(x + dx, y + dy, z + width/2)
        glEnd()


# =============================================================================
# PLOT RENDERER - Enhanced
# =============================================================================

class PlotRenderer:
    def __init__(self, config: Config):
        self.config = config
        self.font = None
        self.font_large = None
        self.font_title = None
        
        # Define plot groups with colors
        self.plot_groups = [
            {
                'title': 'ACCELEROMETER (g)',
                'plots': [
                    {'name': 'X', 'data': deque(maxlen=config.plot_history), 
                     'color': config.accent_red, 'min': -2, 'max': 2},
                    {'name': 'Y', 'data': deque(maxlen=config.plot_history),
                     'color': config.accent_green, 'min': -2, 'max': 2},
                    {'name': 'Z', 'data': deque(maxlen=config.plot_history),
                     'color': config.accent_blue, 'min': -2, 'max': 2},
                ]
            },
            {
                'title': 'GYROSCOPE (°/s)',
                'plots': [
                    {'name': 'X', 'data': deque(maxlen=config.plot_history),
                     'color': config.accent_orange, 'min': -100, 'max': 100},
                    {'name': 'Y', 'data': deque(maxlen=config.plot_history),
                     'color': config.accent_cyan, 'min': -100, 'max': 100},
                    {'name': 'Z', 'data': deque(maxlen=config.plot_history),
                     'color': config.accent_purple, 'min': -100, 'max': 100},
                ]
            },
            {
                'title': 'MAGNETOMETER (µT)',
                'plots': [
                    {'name': 'X', 'data': deque(maxlen=config.plot_history),
                     'color': (255, 100, 150), 'min': -100, 'max': 100},
                    {'name': 'Y', 'data': deque(maxlen=config.plot_history),
                     'color': (100, 255, 150), 'min': -100, 'max': 100},
                    {'name': 'Z', 'data': deque(maxlen=config.plot_history),
                     'color': (150, 100, 255), 'min': -100, 'max': 100},
                ]
            }
        ]

    def init_font(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('SF Mono, Consolas, monospace', 11)
        self.font_large = pygame.font.SysFont('SF Mono, Consolas, monospace', 13, bold=True)
        self.font_title = pygame.font.SysFont('SF Pro Display, Helvetica, Arial', 12, bold=True)

    def update(self, imu: IMUData):
        # Update accelerometer
        for i, plot in enumerate(self.plot_groups[0]['plots']):
            plot['data'].append(imu.accel[i])
            self._auto_scale(plot)
            
        # Update gyroscope
        for i, plot in enumerate(self.plot_groups[1]['plots']):
            plot['data'].append(imu.gyro[i])
            self._auto_scale(plot)
            
        # Update magnetometer
        for i, plot in enumerate(self.plot_groups[2]['plots']):
            plot['data'].append(imu.mag[i])
            self._auto_scale(plot)

    def _auto_scale(self, plot: dict):
        if len(plot['data']) > 20:
            dmin, dmax = min(plot['data']), max(plot['data'])
            margin = max((dmax - dmin) * 0.15, 0.5)
            plot['min'] = dmin - margin
            plot['max'] = dmax + margin

    def render(self, surface: pygame.Surface, x: int, y: int, 
               width: int, height: int, imu: IMUData):
        if not self.font:
            self.init_font()

        # Calculate layout
        group_height = (height - 40) // 3
        plot_height = (group_height - 30) // 3
        
        current_y = y
        
        for group_idx, group in enumerate(self.plot_groups):
            # Group title
            title_surface = self.font_title.render(group['title'], True, self.config.text_secondary)
            surface.blit(title_surface, (x + 5, current_y))
            current_y += 20
            
            # Draw plots for this group
            for plot_idx, plot in enumerate(group['plots']):
                plot_y = current_y + plot_idx * (plot_height + 3)
                self._draw_plot(surface, x, plot_y, width, plot_height, plot)
            
            current_y += 3 * (plot_height + 3) + 15

    def _draw_plot(self, surface: pygame.Surface, x: int, y: int,
                   width: int, height: int, plot: dict):
        label_width = 25
        value_width = 55
        plot_x = x + label_width
        plot_width = width - label_width - value_width - 10
        
        # Plot background
        pygame.draw.rect(surface, self.config.bg_plot, 
                        (plot_x, y, plot_width, height), border_radius=3)
        
        # Zero line
        if plot['min'] < 0 < plot['max']:
            zero_y = y + height - int((0 - plot['min']) / (plot['max'] - plot['min']) * height)
            pygame.draw.line(surface, self.config.grid_color, 
                           (plot_x, zero_y), (plot_x + plot_width, zero_y), 1)
        
        # Data line
        if len(plot['data']) > 1:
            points = []
            for i, val in enumerate(plot['data']):
                px = plot_x + int(i / (self.config.plot_history - 1) * plot_width)
                clamped = max(plot['min'], min(plot['max'], val))
                norm = (clamped - plot['min']) / (plot['max'] - plot['min']) if plot['max'] != plot['min'] else 0.5
                py = y + height - int(norm * height)
                points.append((px, py))
            
            if len(points) > 1:
                pygame.draw.lines(surface, plot['color'], False, points, 2)
        
        # Subtle border
        pygame.draw.rect(surface, self.config.border_color,
                        (plot_x, y, plot_width, height), 1, border_radius=3)
        
        # Label
        label_surface = self.font.render(plot['name'], True, plot['color'])
        surface.blit(label_surface, (x + 2, y + height//2 - 6))
        
        # Current value
        if plot['data']:
            val = plot['data'][-1]
            val_text = f"{val:+7.2f}"
            val_surface = self.font.render(val_text, True, plot['color'])
            surface.blit(val_surface, (plot_x + plot_width + 5, y + height//2 - 6))


# =============================================================================
# CONTROL PANEL - Enhanced with new features
# =============================================================================

class ControlPanel:
    def __init__(self, x: int, y: int, width: int, height: int, 
                 serial_handler: SerialHandler, config: Config):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.serial = serial_handler
        self.config = config
        self.font = None
        self.font_title = None
        self.font_small = None
        
        self.buttons: List[ModernButton] = []
        self.toggles: List[ToggleButton] = []
        self.sliders: List[Slider] = []
        self.indicators: List[StatusIndicator] = []
        
        self.log = deque(maxlen=8)
        self.filter_status = FilterStatus()
        
        # Mag axis signs
        self.mag_signs = [1, 1, -1]  # Default Z inverted for southern hemisphere
        
        self._init_fonts()
        self._create_controls()
        
    def _init_fonts(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('SF Mono, Consolas, monospace', 11)
        self.font_title = pygame.font.SysFont('SF Pro Display, Helvetica, Arial', 13, bold=True)
        self.font_small = pygame.font.SysFont('SF Mono, Consolas, monospace', 10)

    def _log(self, msg: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log.append(f"{timestamp} {msg}")
        print(f"[{timestamp}] {msg}")

    def _create_controls(self):
        bw = self.width - 20  # button width
        bh = 28
        bx = self.x + 10
        cy = self.y + 35  # current y position
        section_gap = 25
        item_gap = 6
        
        # === INITIALIZATION SECTION ===
        self.buttons.append(ModernButton(
            bx, cy, bw, bh, "⚡ Init from Sensors",
            self._cmd_init_from_sensors,
            color=(60, 100, 80)
        ))
        cy += bh + item_gap
        
        self.buttons.append(ModernButton(
            bx, cy, bw, bh, "🔄 Calibrate ALL",
            self._cmd_calibrate,
            color=(100, 70, 70)
        ))
        cy += bh + section_gap
        
        # === FILTER SECTION ===
        self.buttons.append(ModernButton(
            bx, cy, bw, bh, "Reset Filter + Fast Conv",
            self._cmd_reset_filter
        ))
        cy += bh + item_gap
        
        self.buttons.append(ModernButton(
            bx, cy, bw, bh, "Start Fast Convergence",
            self._cmd_fast_convergence,
            color=(70, 70, 100)
        ))
        cy += bh + item_gap + 8
        
        # Beta slider
        self.sliders.append(Slider(
            bx, cy + 16, bw, 20,
            0.01, 1.0, 0.1,
            "Beta", self._cmd_set_beta
        ))
        cy += 50
        
        # Adaptive beta toggle
        self.adaptive_toggle = ToggleButton(
            bx, cy, bw, bh,
            "Adaptive Beta: ON", "Adaptive Beta: OFF",
            self._cmd_toggle_adaptive,
            initial_state=True
        )
        self.toggles.append(self.adaptive_toggle)
        cy += bh + section_gap
        
        # === MAGNETOMETER SECTION ===
        third = (bw - 8) // 3
        
        # Mag sign buttons
        self.mag_x_btn = ModernButton(bx, cy, third, bh, "X: +1", self._toggle_mag_x)
        self.mag_y_btn = ModernButton(bx + third + 4, cy, third, bh, "Y: +1", self._toggle_mag_y)
        self.mag_z_btn = ModernButton(bx + 2*(third + 4), cy, third, bh, "Z: -1", self._toggle_mag_z)
        self.mag_z_btn.base_color = (80, 60, 60)  # Highlight Z as inverted
        self.buttons.extend([self.mag_x_btn, self.mag_y_btn, self.mag_z_btn])
        cy += bh + item_gap
        
        self.buttons.append(ModernButton(
            bx, cy, bw, bh, "Apply Mag Signs",
            self._cmd_apply_mag_signs,
            color=(70, 80, 100)
        ))
        cy += bh + section_gap
        
        # === STATUS SECTION ===
        self.stream_toggle = ToggleButton(
            bx, cy, bw, bh,
            "Stream: ON", "Stream: OFF",
            self._cmd_toggle_stream,
            initial_state=True
        )
        self.toggles.append(self.stream_toggle)
        cy += bh + item_gap
        
        self.buttons.append(ModernButton(
            bx, cy, bw, bh, "Get Status",
            self._cmd_get_status
        ))
        cy += bh + item_gap
        
        self.buttons.append(ModernButton(
            bx, cy, bw, bh, "Get Convergence",
            self._cmd_get_convergence
        ))
        cy += bh + section_gap
        
        # === STATUS INDICATORS ===
        self.ind_converged = StatusIndicator(bx + 5, cy, "Converged")
        self.ind_fast_mode = StatusIndicator(bx + 100, cy, "Fast Mode")
        self.ind_adaptive = StatusIndicator(bx + 5, cy + 18, "Adaptive")
        self.indicators = [self.ind_converged, self.ind_fast_mode, self.ind_adaptive]
        cy += 45
        
        # Store log area position
        self.log_y = cy

    # === COMMAND HANDLERS ===
    
    def _cmd_init_from_sensors(self):
        self._log("Initializing filter from sensors...")
        self.serial.send_command({"T": CMD_IMU_INIT_FROM_SENSORS})
        
    def _cmd_calibrate(self):
        self._log("Calibrating - keep device still and level!")
        self.serial.send_command({"T": CMD_IMU_CALIBRATE_ALL})
        
    def _cmd_reset_filter(self):
        self._log("Resetting filter with fast convergence")
        self.serial.send_command({"T": CMD_IMU_FILTER_RESET})
        
    def _cmd_fast_convergence(self):
        self._log("Starting fast convergence (2s)")
        self.serial.send_command({"T": CMD_IMU_FAST_CONVERGENCE, "duration": 2000, "beta": 2.5})
        
    def _cmd_set_beta(self, value: float):
        self._log(f"Setting beta to {value:.3f}")
        self.serial.send_command({"T": CMD_IMU_SET_BETA, "beta": round(value, 3)})
        
    def _cmd_toggle_adaptive(self):
        self.adaptive_toggle.toggle()
        enabled = self.adaptive_toggle.state
        self._log(f"Adaptive beta: {'enabled' if enabled else 'disabled'}")
        self.serial.send_command({
            "T": CMD_IMU_ADAPTIVE_BETA,
            "enabled": 1 if enabled else 0,
            "stationary": 0.5,
            "motion": 0.05
        })
        
    def _toggle_mag_x(self):
        self.mag_signs[0] *= -1
        self.mag_x_btn.text = f"X: {'+1' if self.mag_signs[0] > 0 else '-1'}"
        self._log(f"Mag X sign: {self.mag_signs[0]}")
        
    def _toggle_mag_y(self):
        self.mag_signs[1] *= -1
        self.mag_y_btn.text = f"Y: {'+1' if self.mag_signs[1] > 0 else '-1'}"
        self._log(f"Mag Y sign: {self.mag_signs[1]}")
        
    def _toggle_mag_z(self):
        self.mag_signs[2] *= -1
        sign_str = '+1' if self.mag_signs[2] > 0 else '-1'
        self.mag_z_btn.text = f"Z: {sign_str}"
        self.mag_z_btn.base_color = (60, 60, 80) if self.mag_signs[2] > 0 else (80, 60, 60)
        self._log(f"Mag Z sign: {self.mag_signs[2]}")
        
    def _cmd_apply_mag_signs(self):
        self._log(f"Applying mag signs: {self.mag_signs}")
        self.serial.send_command({
            "T": CMD_IMU_SET_MAG_SIGNS,
            "x": self.mag_signs[0],
            "y": self.mag_signs[1],
            "z": self.mag_signs[2]
        })
        
    def _cmd_toggle_stream(self):
        self.stream_toggle.toggle()
        enabled = self.stream_toggle.state
        self._log(f"Stream: {'enabled' if enabled else 'disabled'}")
        self.serial.send_command({"T": CMD_IMU_STREAM_CTRL, "cmd": 1 if enabled else 0})
        
    def _cmd_get_status(self):
        self._log("Requesting status...")
        self.serial.send_command({"T": CMD_IMU_GET_STATUS})
        
    def _cmd_get_convergence(self):
        self._log("Requesting convergence status...")
        self.serial.send_command({"T": CMD_IMU_CONVERGENCE_STATUS})

    def update(self, mouse_pos: Tuple[int, int], mouse_pressed: bool):
        for btn in self.buttons:
            btn.update(mouse_pos, mouse_pressed)
        for toggle in self.toggles:
            toggle.update(mouse_pos, mouse_pressed)
        for slider in self.sliders:
            slider.update(mouse_pos, mouse_pressed)

    def handle_click(self, pos: Tuple[int, int]) -> bool:
        for btn in self.buttons:
            if btn.handle_click(pos):
                return True
        for toggle in self.toggles:
            if toggle.handle_click(pos):
                return True
        return False

    def update_filter_status(self, status: FilterStatus):
        self.filter_status = status
        self.ind_converged.state = status.converged
        self.ind_fast_mode.state = status.fast_mode
        self.ind_adaptive.state = status.adaptive

    def handle_response(self, response: dict):
        """Process JSON responses from ESP32"""
        t = response.get("T")
        
        if t == CMD_IMU_INIT_FROM_SENSORS:
            if response.get("status") == "initialized":
                yaw = response.get("yaw", 0)
                pitch = response.get("pitch", 0)
                roll = response.get("roll", 0)
                self._log(f"✓ Init: Y={yaw:.0f} P={pitch:.0f} R={roll:.0f}")
            else:
                self._log(f"✗ Init failed: {response.get('message', 'unknown')}")
                
        elif t == CMD_IMU_FAST_CONVERGENCE:
            self._log(f"✓ Fast convergence started")
            self.ind_fast_mode.state = True
            
        elif t == CMD_IMU_ADAPTIVE_BETA:
            enabled = response.get("enabled", False)
            self.ind_adaptive.state = enabled
            self._log(f"✓ Adaptive: {'ON' if enabled else 'OFF'}")
            
        elif t == CMD_IMU_CONVERGENCE_STATUS:
            rate = response.get("rate", 0)
            converged = response.get("converged", False)
            active_beta = response.get("active_beta", 0)
            self._log(f"Conv: rate={rate:.4f} β={active_beta:.2f}")
            self.ind_converged.state = converged
            self.ind_fast_mode.state = response.get("fast_mode", False)
            self.ind_adaptive.state = response.get("adaptive", False)
            
        elif t == CMD_IMU_GET_STATUS:
            if "mag_dip" in response:
                dip = response["mag_dip"]
                ok = -75 < dip < -55
                status = "✓ OK" if ok else "✗ WRONG"
                self._log(f"Dip: {dip:.1f}° {status}")
            if "active_beta" in response:
                self._log(f"Active β: {response['active_beta']:.3f}")
                
        elif t == CMD_IMU_CALIBRATE_ALL:
            self._log(f"✓ Calibration complete")
            
        elif t == CMD_IMU_FILTER_RESET:
            self._log(f"✓ Filter reset")
            
        elif t == CMD_IMU_SET_BETA:
            self._log(f"✓ Beta: {response.get('beta', '?')}")
            
        elif "status" in response:
            self._log(f">>> {response['status']}")

    def draw(self, surface: pygame.Surface):
        # Panel background
        pygame.draw.rect(surface, self.config.bg_secondary,
                        (self.x, self.y, self.width, self.height))
        
        # Title bar
        pygame.draw.rect(surface, self.config.bg_tertiary,
                        (self.x, self.y, self.width, 28))
        title = self.font_title.render("CONTROLS", True, self.config.text_primary)
        surface.blit(title, (self.x + 10, self.y + 6))
        
        # Draw all controls
        for btn in self.buttons:
            btn.draw(surface, self.font)
        for toggle in self.toggles:
            toggle.draw(surface, self.font)
        for slider in self.sliders:
            slider.draw(surface, self.font)
        for indicator in self.indicators:
            indicator.draw(surface, self.font_small)
        
        # Log area
        log_height = self.height - self.log_y - 10
        log_rect = pygame.Rect(self.x + 5, self.log_y, self.width - 10, log_height)
        pygame.draw.rect(surface, self.config.bg_plot, log_rect, border_radius=4)
        pygame.draw.rect(surface, self.config.border_color, log_rect, 1, border_radius=4)
        
        # Log title
        log_title = self.font_small.render("EVENT LOG", True, self.config.text_muted)
        surface.blit(log_title, (self.x + 10, self.log_y + 4))
        
        # Log entries
        for i, msg in enumerate(self.log):
            if i >= 7:
                break
            color = self.config.accent_green if "✓" in msg else \
                    self.config.accent_red if "✗" in msg else \
                    self.config.accent_orange if ">>>" in msg else \
                    self.config.text_secondary
            
            # Truncate long messages
            display_msg = msg[:35] + "..." if len(msg) > 38 else msg
            txt = self.font_small.render(display_msg, True, color)
            surface.blit(txt, (self.x + 10, self.log_y + 18 + i * 14))


# =============================================================================
# ORIENTATION DISPLAY
# =============================================================================

class OrientationDisplay:
    """Shows current orientation values prominently"""
    
    def __init__(self, config: Config):
        self.config = config
        self.font = None
        self.font_large = None
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.mag_magnitude = 0.0
        self.mag_dip = 0.0
        
    def init_font(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('SF Mono, Consolas, monospace', 12)
        self.font_large = pygame.font.SysFont('SF Mono, Consolas, monospace', 24, bold=True)
        
    def update(self, imu: IMUData):
        self.yaw = imu.yaw
        self.pitch = imu.pitch
        self.roll = imu.roll
        self.mag_magnitude = imu.mag_magnitude
        self.mag_dip = imu.mag_dip
        
    def draw(self, surface: pygame.Surface, x: int, y: int, width: int, height: int):
        if not self.font:
            self.init_font()
            
        # Background
        pygame.draw.rect(surface, self.config.bg_tertiary,
                        (x, y, width, height), border_radius=6)
        
        # Title
        title = self.font.render("ORIENTATION", True, self.config.text_muted)
        surface.blit(title, (x + 10, y + 8))
        
        # Values
        col_width = width // 3
        values = [
            ("YAW", self.yaw, self.config.accent_blue),
            ("PITCH", self.pitch, self.config.accent_green),
            ("ROLL", self.roll, self.config.accent_orange),
        ]
        
        for i, (label, value, color) in enumerate(values):
            cx = x + col_width * i + col_width // 2
            
            # Label
            lbl = self.font.render(label, True, self.config.text_muted)
            lbl_rect = lbl.get_rect(center=(cx, y + 28))
            surface.blit(lbl, lbl_rect)
            
            # Value
            val_text = f"{value:+.1f}°"
            val = self.font_large.render(val_text, True, color)
            val_rect = val.get_rect(center=(cx, y + 52))
            surface.blit(val, val_rect)
        
        # Mag info
        mag_text = f"|M|: {self.mag_magnitude:.1f} µT   Dip: {self.mag_dip:.1f}°"
        dip_ok = -75 < self.mag_dip < -55
        mag_color = self.config.accent_green if dip_ok else self.config.accent_red
        mag_surf = self.font.render(mag_text, True, mag_color)
        surface.blit(mag_surf, (x + 10, y + height - 22))


# =============================================================================
# CONNECTION STATUS BAR
# =============================================================================

class StatusBar:
    """Shows connection status and data rate"""
    
    def __init__(self, config: Config):
        self.config = config
        self.font = None
        self.connected = False
        self.last_data_time = 0
        self.data_rate = 0
        self.last_count = 0
        self.last_rate_update = time.time()
        
    def init_font(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('SF Mono, Consolas, monospace', 11)
        
    def update(self, serial_handler: SerialHandler):
        self.connected = serial_handler.connected
        self.last_data_time = serial_handler.last_data_time
        
        # Calculate data rate
        now = time.time()
        if now - self.last_rate_update >= 1.0:
            self.data_rate = serial_handler.line_count - self.last_count
            self.last_count = serial_handler.line_count
            self.last_rate_update = now
        
    def draw(self, surface: pygame.Surface, x: int, y: int, width: int, height: int):
        if not self.font:
            self.init_font()
            
        # Background
        pygame.draw.rect(surface, self.config.bg_tertiary, (x, y, width, height))
        
        # Connection indicator
        color = self.config.accent_green if self.connected else self.config.accent_red
        pygame.draw.circle(surface, color, (x + 15, y + height // 2), 5)
        
        status_text = "Connected" if self.connected else "Disconnected"
        status = self.font.render(status_text, True, color)
        surface.blit(status, (x + 28, y + (height - status.get_height()) // 2))
        
        # Data rate
        if self.connected:
            rate_text = f"{self.data_rate} Hz"
            rate = self.font.render(rate_text, True, self.config.text_secondary)
            surface.blit(rate, (x + 140, y + (height - rate.get_height()) // 2))
            
            # Data freshness
            age = time.time() - self.last_data_time if self.last_data_time > 0 else 999
            if age < 0.5:
                fresh_color = self.config.accent_green
                fresh_text = "LIVE"
            elif age < 2:
                fresh_color = self.config.accent_orange
                fresh_text = "STALE"
            else:
                fresh_color = self.config.accent_red
                fresh_text = "NO DATA"
                
            fresh = self.font.render(fresh_text, True, fresh_color)
            surface.blit(fresh, (x + width - fresh.get_width() - 15, 
                                y + (height - fresh.get_height()) // 2))


# =============================================================================
# MAIN APPLICATION
# =============================================================================

class IMUVisualizer:
    def __init__(self, config: Config):
        self.config = config
        self.running = False
        
        self.data_queue = queue.Queue(maxsize=100)
        self.response_queue = queue.Queue(maxsize=50)
        self.status_queue = queue.Queue(maxsize=10)
        
        self.serial_handler = SerialHandler(
            config, self.data_queue, self.response_queue, self.status_queue
        )
        self.current_imu = IMUData()

    def _init_pygame(self):
        pygame.init()
        pygame.display.set_caption("IMU Visualizer v6 - Enhanced Edition")
        
        self.screen = pygame.display.set_mode(
            (self.config.window_width, self.config.window_height),
            DOUBLEBUF | OPENGL
        )
        self.clock = pygame.time.Clock()
        
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # Layout calculation
        control_width = 240
        status_bar_height = 28
        orientation_height = 80
        
        view_3d_width = 450
        view_3d_height = self.config.window_height - status_bar_height - 10
        
        plot_x = view_3d_width + 10
        plot_width = self.config.window_width - view_3d_width - control_width - 30
        plot_height = self.config.window_height - status_bar_height - orientation_height - 20
        
        # Create components
        self.renderer_3d = Renderer3D(5, status_bar_height + 5, view_3d_width, view_3d_height)
        
        self.plot_renderer = PlotRenderer(self.config)
        self.plot_surface = pygame.Surface((plot_width, plot_height))
        self.plot_rect = pygame.Rect(plot_x, status_bar_height + 5, plot_width, plot_height)
        
        self.orientation_display = OrientationDisplay(self.config)
        self.orientation_rect = pygame.Rect(
            plot_x, self.config.window_height - orientation_height - 5,
            plot_width, orientation_height
        )
        self.orientation_surface = pygame.Surface((plot_width, orientation_height))
        
        self.control_panel = ControlPanel(
            0, 0, control_width, self.config.window_height - status_bar_height,
            self.serial_handler, self.config
        )
        self.control_surface = pygame.Surface((control_width, self.config.window_height - status_bar_height))
        self.control_x = self.config.window_width - control_width
        
        self.status_bar = StatusBar(self.config)
        self.status_surface = pygame.Surface((self.config.window_width, status_bar_height))

    def _process_data(self):
        # Process IMU data
        while not self.data_queue.empty():
            try:
                self.current_imu = self.data_queue.get_nowait()
                self.renderer_3d.update_orientation(
                    self.current_imu.roll, 
                    self.current_imu.pitch, 
                    self.current_imu.yaw
                )
                self.plot_renderer.update(self.current_imu)
                self.orientation_display.update(self.current_imu)
            except queue.Empty:
                break
        
        # Process responses
        while not self.response_queue.empty():
            try:
                self.control_panel.handle_response(self.response_queue.get_nowait())
            except queue.Empty:
                break
                
        # Process filter status
        while not self.status_queue.empty():
            try:
                self.control_panel.update_filter_status(self.status_queue.get_nowait())
            except queue.Empty:
                break

    def _render(self):
        glClearColor(0.07, 0.07, 0.09, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Render 3D view
        self.renderer_3d.render(self.config.window_height)

        # Render plots
        self.plot_surface.fill(self.config.bg_primary)
        self.plot_renderer.render(
            self.plot_surface, 5, 5,
            self.plot_rect.width - 10, self.plot_rect.height - 10,
            self.current_imu
        )
        self._blit_surface(self.plot_surface, self.plot_rect.x, 
                          self.config.window_height - self.plot_rect.y - self.plot_rect.height)
        
        # Render orientation display
        self.orientation_surface.fill(self.config.bg_primary)
        self.orientation_display.draw(
            self.orientation_surface, 0, 0,
            self.orientation_rect.width, self.orientation_rect.height
        )
        self._blit_surface(self.orientation_surface, self.orientation_rect.x,
                          self.config.window_height - self.orientation_rect.y - self.orientation_rect.height)

        # Render control panel
        self.control_surface.fill(self.config.bg_primary)
        self.control_panel.draw(self.control_surface)
        self._blit_surface(self.control_surface, self.control_x, 0)
        
        # Render status bar
        self.status_bar.update(self.serial_handler)
        self.status_surface.fill(self.config.bg_primary)
        self.status_bar.draw(self.status_surface, 0, 0, 
                            self.config.window_width, self.status_surface.get_height())
        self._blit_surface(self.status_surface, 0, 
                          self.config.window_height - self.status_surface.get_height())

        pygame.display.flip()

    def _blit_surface(self, surface: pygame.Surface, x: int, y: int):
        """Blit a pygame surface to the OpenGL context"""
        data = pygame.image.tostring(surface, 'RGBA', True)
        w, h = surface.get_size()
        
        glViewport(x, y, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, w, 0, h, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)

        glEnable(GL_TEXTURE_2D)
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(0, 0)
        glTexCoord2f(1, 0); glVertex2f(w, 0)
        glTexCoord2f(1, 1); glVertex2f(w, h)
        glTexCoord2f(0, 1); glVertex2f(0, h)
        glEnd()
        glDisable(GL_TEXTURE_2D)
        glDeleteTextures([tex])

    def run(self):
        self._init_pygame()
        self.serial_handler.start()
        self.running = True

        print("=" * 60)
        print("  IMU Visualizer v6 - Enhanced Edition")
        print("=" * 60)
        print(f"  Serial Port: {self.config.serial_port}")
        print()
        print("  NEW FEATURES:")
        print("    • Init from Sensors - Instant correct orientation!")
        print("    • Fast Convergence  - Quick alignment mode")
        print("    • Adaptive Beta     - Motion-aware gain")
        print("    • Convergence Status - Monitor filter state")
        print()
        print("  KEYBOARD:")
        print("    ESC - Exit")
        print("=" * 60)

        try:
            while self.running:
                mouse_pos = pygame.mouse.get_pos()
                mouse_pressed = pygame.mouse.get_pressed()[0]
                
                # Convert mouse pos to control panel coords
                ctrl_pos = (mouse_pos[0] - self.control_x, mouse_pos[1])
                self.control_panel.update(ctrl_pos, mouse_pressed)

                for event in pygame.event.get():
                    if event.type == QUIT:
                        self.running = False
                    elif event.type == KEYDOWN:
                        if event.key == K_ESCAPE:
                            self.running = False
                    elif event.type == MOUSEBUTTONDOWN and event.button == 1:
                        self.control_panel.handle_click(ctrl_pos)

                self._process_data()
                self._render()
                self.clock.tick(self.config.fps)
                
        finally:
            self.serial_handler.stop()
            pygame.quit()
            print("\n✓ Stopped cleanly")


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    import sys
    
    config = Config()
    
    # Parse command line arguments
    for arg in sys.argv[1:]:
        if arg.startswith("--port="):
            config.serial_port = arg.split("=", 1)[1]
        elif arg.startswith("--baud="):
            config.baud_rate = int(arg.split("=", 1)[1])
        elif arg in ["-h", "--help"]:
            print(__doc__)
            sys.exit(0)
            
    app = IMUVisualizer(config)
    app.run()