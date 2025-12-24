#!/usr/bin/env python3
"""
IMU Visualizer v5 - With Control Panel

WHAT THIS DOES:
- Shows live 3D orientation from your IMU
- Plots sensor data (accelerometer, gyroscope, magnetometer)
- Lets you send commands to calibrate and configure the IMU

BUTTON DESCRIPTIONS:
- "Calibrate ALL": Recalibrates gyro + accel. Keep device STILL and LEVEL.
                   You'll see "calibrating..." then "complete" in the log.
                   
- "Reset Filter":  Resets the Madgwick orientation filter to default state.
                   Use if orientation looks wrong after moving the device.
                   
- "Beta" slider:   Controls filter responsiveness (0.01=smooth, 0.5=fast).
                   Drag and release to send new value.
                   
- "Stream ON/OFF": Toggles whether IMU data is being sent from the device.

- X/Y/Z buttons:   Toggle magnetometer axis sign (+1 or -1).
                   Click to toggle, then click "Apply" to send to device.
                   Z should be -1 for your hardware (Melbourne, southern hemisphere).
                   
- "Apply Mag Signs": Sends the current X/Y/Z signs to the device.
                     Check the log - if dip angle becomes ~-65°, it's correct.
                     
- "Request Status": Gets current sensor values and magnetic dip angle.
                    Dip should be around -65° for Melbourne. If positive, 
                    your Z axis sign is wrong.
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
import json
from datetime import datetime
from collections import deque
from dataclasses import dataclass

# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class Config:
    serial_port: str = "/dev/cu.usbserial-110"
    baud_rate: int = 115200
    window_width: int = 1100  # Narrower window
    window_height: int = 700
    fps: int = 60
    plot_history: int = 200
    plot_height: int = 30
    plot_margin: int = 2
    bg_color: tuple = (30, 30, 35)
    plot_bg_color: tuple = (20, 20, 25)
    grid_color: tuple = (50, 50, 55)
    text_color: tuple = (200, 200, 200)

# =============================================================================
# JSON COMMAND TYPES (must match config.h on ESP32)
# =============================================================================

CMD_IMU_STREAM_CTRL = 325
CMD_IMU_CALIBRATE_ALL = 332
CMD_IMU_FILTER_RESET = 335
CMD_IMU_SET_BETA = 336
CMD_IMU_SET_MAG_SIGNS = 340
CMD_IMU_GET_STATUS = 346

# =============================================================================
# SCALE FACTORS
# =============================================================================

MOTIONCAL_ACCEL_SCALE = 8192.0
MOTIONCAL_GYRO_SCALE = 16.0
MOTIONCAL_MAG_SCALE = 10.0

# =============================================================================
# DATA STRUCTURE
# =============================================================================

@dataclass
class IMUData:
    timestamp: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    raw_ax: float = 0.0
    raw_ay: float = 0.0
    raw_az: float = 0.0
    raw_gx: float = 0.0
    raw_gy: float = 0.0
    raw_gz: float = 0.0
    raw_mx: float = 0.0
    raw_my: float = 0.0
    raw_mz: float = 0.0
    mag_magnitude: float = 0.0

# =============================================================================
# UI BUTTON
# =============================================================================

class Button:
    def __init__(self, x, y, width, height, text, callback, 
                 color=(60, 60, 70), hover_color=(80, 80, 95)):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.callback = callback
        self.hovered = False
        
    def update(self, mouse_pos):
        self.hovered = self.rect.collidepoint(mouse_pos)
        
    def draw(self, surface, font):
        color = self.hover_color if self.hovered else self.color
        pygame.draw.rect(surface, color, self.rect)
        pygame.draw.rect(surface, (100, 100, 110), self.rect, 1)
        text_surface = font.render(self.text, True, (220, 220, 220))
        text_rect = text_surface.get_rect(center=self.rect.center)
        surface.blit(text_surface, text_rect)
        
    def handle_click(self, pos):
        if self.rect.collidepoint(pos):
            self.callback()
            return True
        return False

# =============================================================================
# SERIAL HANDLER
# =============================================================================

class SerialHandler:
    def __init__(self, config, data_queue, response_queue):
        self.config = config
        self.data_queue = data_queue
        self.response_queue = response_queue
        self.serial_port = None
        self.running = False
        self.thread = None
        self.connected = False
        self.current_data = IMUData()
        self.line_count = 0
        self.command_queue = queue.Queue()

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

    def send_command(self, cmd):
        self.command_queue.put(cmd)

    def _connect(self):
        try:
            self.serial_port = serial.Serial(
                port=self.config.serial_port,
                baudrate=self.config.baud_rate,
                timeout=0.1
            )
            self.connected = True
            print(f"Connected to {self.config.serial_port}")
            return True
        except Exception as e:
            self.connected = False
            print(f"Connection failed: {e}")
            return False

    def _parse_line(self, line):
        line = line.strip()
        if not line:
            return False

        self.current_data.timestamp = time.time()
        parsed = False

        # Parse Raw: format (MotionCal)
        if line.startswith("Raw:"):
            try:
                parts = line[4:].split(",")
                if len(parts) == 9:
                    self.current_data.raw_ax = float(parts[0]) / MOTIONCAL_ACCEL_SCALE
                    self.current_data.raw_ay = float(parts[1]) / MOTIONCAL_ACCEL_SCALE
                    self.current_data.raw_az = float(parts[2]) / MOTIONCAL_ACCEL_SCALE
                    self.current_data.raw_gx = float(parts[3]) / MOTIONCAL_GYRO_SCALE
                    self.current_data.raw_gy = float(parts[4]) / MOTIONCAL_GYRO_SCALE
                    self.current_data.raw_gz = float(parts[5]) / MOTIONCAL_GYRO_SCALE
                    self.current_data.raw_mx = float(parts[6]) / MOTIONCAL_MAG_SCALE
                    self.current_data.raw_my = float(parts[7]) / MOTIONCAL_MAG_SCALE
                    self.current_data.raw_mz = float(parts[8]) / MOTIONCAL_MAG_SCALE
                    self.current_data.mag_magnitude = math.sqrt(
                        self.current_data.raw_mx**2 +
                        self.current_data.raw_my**2 +
                        self.current_data.raw_mz**2
                    )
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
                # Put response in queue for control panel to handle
                self.response_queue.put(data)
                
                # Also parse IMU stream data if present
                if data.get("T") == "imu":
                    self.current_data.yaw = data.get("yaw", 0)
                    self.current_data.pitch = data.get("pitch", 0)
                    self.current_data.roll = data.get("roll", 0)
                    parsed = True
            except json.JSONDecodeError:
                pass

        if parsed:
            self.line_count += 1
            if self.line_count % 2 == 0:
                try:
                    self.data_queue.put_nowait(IMUData(
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
                        mag_magnitude=self.current_data.mag_magnitude,
                    ))
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
                # Send queued commands
                while not self.command_queue.empty():
                    cmd = self.command_queue.get_nowait()
                    cmd_str = json.dumps(cmd) + "\n"
                    self.serial_port.write(cmd_str.encode())
                    print(f">>> Sent: {cmd_str.strip()}")

                # Read incoming data
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += chunk.decode('utf-8', errors='ignore')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self._parse_line(line)
            except Exception as e:
                print(f"Serial error: {e}")
                self.connected = False

            time.sleep(0.001)

# =============================================================================
# 3D RENDERER (Simplified)
# =============================================================================

class Renderer3D:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update_orientation(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def render(self, window_height):
        glViewport(self.x, window_height - self.y - self.height, self.width, self.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.width / self.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        gluLookAt(4, 3, 4, 0, 0, 0, 0, 1, 0)
        glEnable(GL_DEPTH_TEST)
        glClear(GL_DEPTH_BUFFER_BIT)

        # Reference axes
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.3, 0.3); glVertex3f(0,0,0); glVertex3f(2,0,0)  # X red
        glColor3f(0.3, 1.0, 0.3); glVertex3f(0,0,0); glVertex3f(0,2,0)  # Y green
        glColor3f(0.3, 0.3, 1.0); glVertex3f(0,0,0); glVertex3f(0,0,2)  # Z blue
        glEnd()

        # Apply rotation
        glRotatef(self.yaw, 0, 1, 0)
        glRotatef(self.pitch, 1, 0, 0)
        glRotatef(self.roll, 0, 0, 1)

        # Draw arrow
        glLineWidth(3.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.8, 0.0)
        glVertex3f(0, 0, -0.8)
        glVertex3f(0, 0, 1.2)
        glEnd()

        glBegin(GL_TRIANGLES)
        glColor3f(1.0, 0.6, 0.0)
        glVertex3f(0, 0, 1.6); glVertex3f(0.25, 0.25, 1.2); glVertex3f(-0.25, 0.25, 1.2)
        glVertex3f(0, 0, 1.6); glVertex3f(-0.25, -0.25, 1.2); glVertex3f(0.25, -0.25, 1.2)
        glEnd()

        glDisable(GL_DEPTH_TEST)

# =============================================================================
# PLOT RENDERER
# =============================================================================

class PlotRenderer:
    def __init__(self, config):
        self.config = config
        self.plots = []
        self.font = None

        # 9 plots: Acc XYZ, Gyro XYZ, Mag XYZ
        names = ["AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ", "MagX", "MagY", "MagZ"]
        colors = [(255,120,120), (120,255,120), (120,120,255),
                  (255,200,120), (200,255,120), (120,255,200),
                  (255,80,80), (80,255,80), (80,80,255)]
        
        for name, color in zip(names, colors):
            self.plots.append({
                'name': name,
                'data': deque(maxlen=config.plot_history),
                'color': color,
                'min': -1, 'max': 1
            })

    def init_font(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('monospace', 10)
        self.font_large = pygame.font.SysFont('monospace', 11, bold=True)

    def update(self, imu):
        values = [imu.raw_ax, imu.raw_ay, imu.raw_az,
                  imu.raw_gx, imu.raw_gy, imu.raw_gz,
                  imu.raw_mx, imu.raw_my, imu.raw_mz]
        
        for plot, val in zip(self.plots, values):
            plot['data'].append(val)
            if len(plot['data']) > 10:
                dmin, dmax = min(plot['data']), max(plot['data'])
                margin = (dmax - dmin) * 0.1 + 0.1
                plot['min'], plot['max'] = dmin - margin, dmax + margin

    def render(self, surface, x, y, width, height, imu):
        if not self.font:
            self.init_font()

        ph = self.config.plot_height
        pm = self.config.plot_margin
        lw = 45  # label width

        for i, plot in enumerate(self.plots):
            py = y + i * (ph + pm)
            px = x + lw
            pw = width - lw - 5

            pygame.draw.rect(surface, self.config.plot_bg_color, (px, py, pw, ph))
            pygame.draw.rect(surface, self.config.grid_color, (px, py, pw, ph), 1)

            if len(plot['data']) > 1:
                points = []
                for j, val in enumerate(plot['data']):
                    ptx = px + int(j / (self.config.plot_history - 1) * pw)
                    clamped = max(plot['min'], min(plot['max'], val))
                    norm = (clamped - plot['min']) / (plot['max'] - plot['min']) if plot['max'] != plot['min'] else 0.5
                    pty = py + ph - int(norm * ph)
                    points.append((ptx, pty))
                pygame.draw.lines(surface, plot['color'], False, points, 2)

            lbl = self.font.render(plot['name'], True, self.config.text_color)
            surface.blit(lbl, (x + 2, py + 2))
            
            if plot['data']:
                val_txt = self.font.render(f"{plot['data'][-1]:+.1f}", True, plot['color'])
                surface.blit(val_txt, (x + 2, py + ph - 11))

        # Status bar
        sy = y + len(self.plots) * (ph + pm) + 5
        pygame.draw.rect(surface, (40, 40, 45), (x, sy, width, 45))
        
        txt = f"YAW:{imu.yaw:+6.1f}  PITCH:{imu.pitch:+6.1f}  ROLL:{imu.roll:+6.1f}"
        surface.blit(self.font_large.render(txt, True, (100, 200, 255)), (x + 5, sy + 5))
        
        mag_txt = f"|Mag|: {imu.mag_magnitude:.1f} uT"
        surface.blit(self.font.render(mag_txt, True, (200, 200, 200)), (x + 5, sy + 22))

# =============================================================================
# CONTROL PANEL
# =============================================================================

class ControlPanel:
    def __init__(self, x, y, width, height, serial_handler):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.serial = serial_handler
        self.font = None
        self.buttons = []
        self.log = deque(maxlen=6)
        
        # Mag axis signs (default: Z inverted for southern hemisphere)
        self.mag_x = 1
        self.mag_y = 1
        self.mag_z = -1
        
        # Beta slider state
        self.beta = 0.1
        self.beta_rect = None
        self.dragging_beta = False
        
        self._create_controls()
        
    def _log(self, msg):
        self.log.append(msg)
        print(f"[UI] {msg}")
        
    def _create_controls(self):
        bw = self.width - 16  # button width
        bh = 22               # button height
        bx = self.x + 8       # button x
        y = self.y + 25
        
        # Calibrate button
        self.buttons.append(Button(bx, y, bw, bh, "Calibrate ALL (keep still!)",
            self._cmd_calibrate, color=(100, 65, 65), hover_color=(120, 80, 80)))
        y += bh + 20
        
        # Reset filter
        self.buttons.append(Button(bx, y, bw, bh, "Reset Filter", self._cmd_reset))
        y += bh + 8
        
        # Beta slider position
        self.beta_rect = pygame.Rect(bx, y + 14, bw, 14)
        y += 45
        
        # Stream toggle
        self.stream_on = True
        self.buttons.append(Button(bx, y, bw, bh, "Stream: ON", self._cmd_toggle_stream))
        self.stream_btn_idx = len(self.buttons) - 1
        y += bh + 20
        
        # Mag axis buttons - FIXED: using proper method references
        third = (bw - 8) // 3
        self.buttons.append(Button(bx, y, third, bh, "X:+1", self._toggle_x))
        self.buttons.append(Button(bx + third + 4, y, third, bh, "Y:+1", self._toggle_y))
        self.buttons.append(Button(bx + 2*(third + 4), y, third, bh, "Z:-1", self._toggle_z))
        self.mag_btn_x = len(self.buttons) - 3
        self.mag_btn_y = len(self.buttons) - 2
        self.mag_btn_z = len(self.buttons) - 1
        y += bh + 5
        
        # Apply mag signs
        self.buttons.append(Button(bx, y, bw, bh, "Apply Mag Signs", self._cmd_apply_mag,
            color=(60, 75, 90), hover_color=(75, 90, 110)))
        y += bh + 20
        
        # Get status
        self.buttons.append(Button(bx, y, bw, bh, "Request Status", self._cmd_status))
        
    def _cmd_calibrate(self):
        self._log("Calibrating... keep device still and level")
        self.serial.send_command({"T": CMD_IMU_CALIBRATE_ALL})
        
    def _cmd_reset(self):
        self._log("Filter reset")
        self.serial.send_command({"T": CMD_IMU_FILTER_RESET})
        
    def _cmd_set_beta(self):
        self._log(f"Beta set to {self.beta:.2f}")
        self.serial.send_command({"T": CMD_IMU_SET_BETA, "beta": round(self.beta, 3)})
        
    def _cmd_toggle_stream(self):
        self.stream_on = not self.stream_on
        self.buttons[self.stream_btn_idx].text = f"Stream: {'ON' if self.stream_on else 'OFF'}"
        self._log(f"Stream {'enabled' if self.stream_on else 'disabled'}")
        self.serial.send_command({"T": CMD_IMU_STREAM_CTRL, "cmd": 1 if self.stream_on else 0})
        
    def _toggle_x(self):
        self.mag_x *= -1
        self.buttons[self.mag_btn_x].text = f"X:{'+1' if self.mag_x > 0 else '-1'}"
        self._log(f"Mag X sign: {'+1' if self.mag_x > 0 else '-1'}")
        
    def _toggle_y(self):
        self.mag_y *= -1
        self.buttons[self.mag_btn_y].text = f"Y:{'+1' if self.mag_y > 0 else '-1'}"
        self._log(f"Mag Y sign: {'+1' if self.mag_y > 0 else '-1'}")
        
    def _toggle_z(self):
        self.mag_z *= -1
        self.buttons[self.mag_btn_z].text = f"Z:{'+1' if self.mag_z > 0 else '-1'}"
        self._log(f"Mag Z sign: {'+1' if self.mag_z > 0 else '-1'}")
        
    def _cmd_apply_mag(self):
        self._log(f"Applying mag signs: X={self.mag_x}, Y={self.mag_y}, Z={self.mag_z}")
        self.serial.send_command({
            "T": CMD_IMU_SET_MAG_SIGNS,
            "x": self.mag_x, "y": self.mag_y, "z": self.mag_z
        })
        
    def _cmd_status(self):
        self._log("Requesting status from device...")
        self.serial.send_command({"T": CMD_IMU_GET_STATUS})
        
    def init_font(self):
        pygame.font.init()
        self.font = pygame.font.SysFont('monospace', 10)
        self.font_title = pygame.font.SysFont('monospace', 11, bold=True)
        
    def update(self, mouse_pos, mouse_pressed):
        for btn in self.buttons:
            btn.update(mouse_pos)
            
        # Beta slider
        if self.beta_rect:
            if mouse_pressed[0] and self.beta_rect.collidepoint(mouse_pos):
                self.dragging_beta = True
            if self.dragging_beta:
                if mouse_pressed[0]:
                    rel = (mouse_pos[0] - self.beta_rect.x) / self.beta_rect.width
                    self.beta = 0.01 + max(0, min(1, rel)) * 0.49  # Range: 0.01 to 0.5
                else:
                    self.dragging_beta = False
                    self._cmd_set_beta()
            
    def handle_click(self, pos):
        for btn in self.buttons:
            if btn.handle_click(pos):
                return True
        return False
    
    def handle_response(self, response):
        """Process JSON responses from ESP32"""
        if "status" in response:
            self._log(f">>> {response['status']}")
        if "mag_dip" in response:
            dip = response["mag_dip"]
            ok = -75 < dip < -55
            self._log(f"Dip angle: {dip:.1f}° {'(OK!)' if ok else '(WRONG!)'}")
        if "beta" in response:
            self._log(f"Beta confirmed: {response['beta']:.3f}")
        if "enabled" in response:
            self._log(f"Stream: {'ON' if response['enabled'] else 'OFF'}")
        
    def draw(self, surface):
        if not self.font:
            self.init_font()
            
        # Background
        pygame.draw.rect(surface, (35, 35, 40), (self.x, self.y, self.width, self.height))
        pygame.draw.rect(surface, (60, 60, 70), (self.x, self.y, self.width, self.height), 1)
        
        # Title
        surface.blit(self.font_title.render("CONTROLS", True, (200, 200, 200)), 
                     (self.x + 8, self.y + 6))
        
        # Draw buttons
        for btn in self.buttons:
            btn.draw(surface, self.font)
            
        # Beta slider
        if self.beta_rect:
            pygame.draw.rect(surface, (40, 40, 45), self.beta_rect)
            pygame.draw.rect(surface, (60, 60, 70), self.beta_rect, 1)
            fill_w = int((self.beta - 0.01) / 0.49 * self.beta_rect.width)
            pygame.draw.rect(surface, (80, 120, 180), 
                           (self.beta_rect.x, self.beta_rect.y, fill_w, self.beta_rect.height))
            pygame.draw.rect(surface, (200, 200, 210),
                           (self.beta_rect.x + fill_w - 2, self.beta_rect.y - 2, 4, self.beta_rect.height + 4))
            lbl = self.font.render(f"Beta: {self.beta:.2f} (drag to adjust)", True, (180, 180, 180))
            surface.blit(lbl, (self.beta_rect.x, self.beta_rect.y - 14))
            
        # Log area
        log_y = self.y + self.height - 95
        pygame.draw.rect(surface, (25, 25, 30), (self.x + 4, log_y, self.width - 8, 90))
        pygame.draw.rect(surface, (50, 50, 55), (self.x + 4, log_y, self.width - 8, 90), 1)
        surface.blit(self.font.render("Response Log:", True, (140, 140, 140)), (self.x + 8, log_y + 3))
        
        for i, msg in enumerate(self.log):
            color = (100, 255, 100) if "OK" in msg else (255, 200, 100) if "WRONG" in msg else (180, 180, 180)
            txt = self.font.render(msg[:30], True, color)
            surface.blit(txt, (self.x + 8, log_y + 16 + i * 12))

# =============================================================================
# MAIN APPLICATION
# =============================================================================

class IMUVisualizer:
    def __init__(self, config):
        self.config = config
        self.running = False
        self.data_queue = queue.Queue(maxsize=100)
        self.response_queue = queue.Queue(maxsize=50)
        self.serial_handler = SerialHandler(config, self.data_queue, self.response_queue)
        self.current_imu = IMUData()

    def _init_pygame(self):
        pygame.init()
        pygame.display.set_caption("IMU Visualizer v5 - Control Panel")
        self.screen = pygame.display.set_mode(
            (self.config.window_width, self.config.window_height),
            DOUBLEBUF | OPENGL
        )
        self.clock = pygame.time.Clock()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # Layout: 3D view | Plots | Control Panel
        ctrl_w = 190
        view_3d_w = int((self.config.window_width - ctrl_w) * 0.40)
        plot_w = self.config.window_width - ctrl_w - view_3d_w

        self.renderer_3d = Renderer3D(0, 0, view_3d_w, self.config.window_height)
        self.plot_renderer = PlotRenderer(self.config)
        self.plot_surface = pygame.Surface((plot_w, self.config.window_height))
        self.plot_x = view_3d_w

        self.control_panel = ControlPanel(0, 0, ctrl_w, self.config.window_height, self.serial_handler)
        self.ctrl_surface = pygame.Surface((ctrl_w, self.config.window_height))
        self.ctrl_x = self.config.window_width - ctrl_w

    def _process_data(self):
        while not self.data_queue.empty():
            try:
                self.current_imu = self.data_queue.get_nowait()
                self.renderer_3d.update_orientation(
                    self.current_imu.roll, self.current_imu.pitch, self.current_imu.yaw)
                self.plot_renderer.update(self.current_imu)
            except queue.Empty:
                break
                
        while not self.response_queue.empty():
            try:
                self.control_panel.handle_response(self.response_queue.get_nowait())
            except queue.Empty:
                break

    def _render(self):
        glClearColor(0.12, 0.12, 0.14, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self.renderer_3d.render(self.config.window_height)

        # Plots
        self.plot_surface.fill(self.config.bg_color)
        pw = self.ctrl_x - self.plot_x
        self.plot_renderer.render(self.plot_surface, 8, 8, pw - 16, 
                                  self.config.window_height - 16, self.current_imu)
        
        # Connection status
        if self.plot_renderer.font:
            color = (100, 255, 100) if self.serial_handler.connected else (255, 100, 100)
            txt = "Connected" if self.serial_handler.connected else "Disconnected"
            self.plot_surface.blit(self.plot_renderer.font.render(f"Serial: {txt}", True, color),
                                  (8, self.config.window_height - 16))
        
        self._blit_surface(self.plot_surface, self.plot_x)

        # Control panel
        self.ctrl_surface.fill(self.config.bg_color)
        self.control_panel.draw(self.ctrl_surface)
        self._blit_surface(self.ctrl_surface, self.ctrl_x)

        pygame.display.flip()

    def _blit_surface(self, surface, x):
        data = pygame.image.tostring(surface, 'RGBA', True)
        w, h = surface.get_size()
        glViewport(x, 0, w, h)
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

        print("=" * 55)
        print("IMU Visualizer v5")
        print("=" * 55)
        print(f"Port: {self.config.serial_port}")
        print()
        print("KEYBOARD:")
        print("  ESC - Exit")
        print()
        print("See docstring at top of script for button descriptions")
        print("=" * 55)

        try:
            while self.running:
                mouse_pos = pygame.mouse.get_pos()
                mouse_pressed = pygame.mouse.get_pressed()
                
                # Convert mouse pos to control panel coords
                ctrl_pos = (mouse_pos[0] - self.ctrl_x, mouse_pos[1])
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
            print("Stopped")

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
