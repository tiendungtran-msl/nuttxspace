#!/usr/bin/env python3
"""
UAV Ground Station - Professional Flight Display

A professional ground station display for UAV sensor data and state estimation.
Features Primary Flight Display (PFD) style interface with:
- Artificial Horizon (Attitude Indicator)
- Heading Indicator (Compass Rose)
- IMU Data Panels
- Gyro Bias Monitoring
- Multi-IMU Support

Protocol:
    "ATT,<id>,<roll>,<pitch>,<yaw>,<bias_x>,<bias_y>,<bias_z>\n"
    "ATT,<id>,<roll>,<pitch>,<yaw>,<timestamp>\n" (legacy)

Requirements:
    pip install pyserial pygame

Usage:
    python3 uav_ground_station.py                    # Interactive mode
    python3 uav_ground_station.py --port /dev/ttyUSB0
    python3 uav_ground_station.py --sim              # Simulation mode
    python3 uav_ground_station.py --help

Author: UAV States Project
Version: 1.0
"""

import sys
import os
import argparse
import math
import time
import threading
import random
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Callable, Tuple

try:
    import pygame
    from pygame import gfxdraw
except ImportError:
    print("=" * 60)
    print("  Missing dependency: pygame")
    print("  Install: pip3 install pygame")
    print("=" * 60)
    sys.exit(1)

try:
    import serial
    from serial.tools import list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. Serial input disabled.")
    print("Install: pip3 install pyserial")


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class Config:
    """Application configuration"""
    WINDOW_WIDTH: int = 1400
    WINDOW_HEIGHT: int = 900
    FPS: int = 60
    
    # Colors (R, G, B)
    BG_COLOR: Tuple[int, int, int] = (20, 25, 35)
    PANEL_BG: Tuple[int, int, int] = (30, 35, 45)
    PANEL_BORDER: Tuple[int, int, int] = (60, 70, 90)
    TEXT_PRIMARY: Tuple[int, int, int] = (220, 230, 240)
    TEXT_SECONDARY: Tuple[int, int, int] = (140, 150, 170)
    TEXT_ACCENT: Tuple[int, int, int] = (100, 200, 255)
    
    # PFD Colors
    SKY_COLOR: Tuple[int, int, int] = (50, 120, 200)
    GROUND_COLOR: Tuple[int, int, int] = (120, 80, 40)
    HORIZON_LINE: Tuple[int, int, int] = (255, 255, 255)
    PITCH_LINES: Tuple[int, int, int] = (255, 255, 255)
    AIRCRAFT_SYMBOL: Tuple[int, int, int] = (255, 200, 0)
    
    # Status Colors
    STATUS_OK: Tuple[int, int, int] = (50, 200, 100)
    STATUS_WARN: Tuple[int, int, int] = (255, 200, 50)
    STATUS_ERROR: Tuple[int, int, int] = (255, 80, 80)
    STATUS_OFFLINE: Tuple[int, int, int] = (100, 100, 100)


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class IMUData:
    """Data from a single IMU"""
    imu_id: int
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    gyro_bias: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    timestamp: int = 0
    last_update: float = field(default_factory=time.time)
    update_count: int = 0
    history: deque = field(default_factory=lambda: deque(maxlen=300))
    
    def update(self, roll: float, pitch: float, yaw: float, 
               timestamp: int = 0, bias: Optional[List[float]] = None):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.timestamp = timestamp
        if bias:
            self.gyro_bias = bias
        self.last_update = time.time()
        self.update_count += 1
        self.history.append((time.time(), roll, pitch, yaw))
    
    def get_update_rate(self) -> float:
        if len(self.history) < 2:
            return 0.0
        dt = self.history[-1][0] - self.history[0][0]
        return len(self.history) / dt if dt > 0 else 0.0
    
    def get_age(self) -> float:
        return time.time() - self.last_update
    
    def is_active(self) -> bool:
        return self.get_age() < 1.0


# =============================================================================
# Serial Reader
# =============================================================================

class SerialReader(threading.Thread):
    """Background thread for reading serial data"""
    
    def __init__(self, port: str, baudrate: int, callback: Callable):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self.running = False
        self.ser = None
        self.bytes_received = 0
        self.messages_received = 0
        self.error_message = ""
        self.connected = False
    
    def run(self):
        self.running = True
        buffer = ""
        
        while self.running:
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                    self.connected = True
                    self.error_message = ""
                except Exception as e:
                    self.error_message = str(e)
                    self.connected = False
                    time.sleep(1.0)
                    continue
            
            try:
                raw = self.ser.read(512)
                if raw:
                    self.bytes_received += len(raw)
                    buffer += raw.decode('utf-8', errors='ignore')
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('ATT,'):
                            self._parse_attitude(line)
                            
            except Exception as e:
                self.error_message = str(e)
                self.connected = False
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None
                time.sleep(0.5)
    
    def _parse_attitude(self, line: str):
        try:
            parts = line.split(',')
            
            if len(parts) == 8:
                # New format: ATT,id,r,p,y,bx,by,bz
                imu_id = int(parts[1])
                roll = float(parts[2])
                pitch = float(parts[3])
                yaw = float(parts[4])
                bias = [float(parts[5]), float(parts[6]), float(parts[7])]
                self.callback(imu_id, roll, pitch, yaw, 0, bias)
                self.messages_received += 1
                
            elif len(parts) >= 6:
                # Legacy format: ATT,id,r,p,y,ts
                imu_id = int(parts[1])
                roll = float(parts[2])
                pitch = float(parts[3])
                yaw = float(parts[4])
                ts = int(parts[5])
                self.callback(imu_id, roll, pitch, yaw, ts, None)
                self.messages_received += 1
                
        except (ValueError, IndexError):
            pass
    
    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except:
                pass


# =============================================================================
# Simulator
# =============================================================================

class Simulator(threading.Thread):
    """Generates simulated IMU data for testing"""
    
    def __init__(self, callback: Callable, num_imus: int = 4):
        super().__init__(daemon=True)
        self.callback = callback
        self.num_imus = num_imus
        self.running = False
        self.time_offset = 0.0
        
        # Simulation parameters per IMU
        self.params = []
        for i in range(num_imus):
            self.params.append({
                'roll_freq': 0.1 + i * 0.02,
                'pitch_freq': 0.08 + i * 0.015,
                'yaw_freq': 0.05 + i * 0.01,
                'roll_amp': 15 + i * 2,
                'pitch_amp': 10 + i * 1.5,
                'yaw_amp': 30 + i * 5,
                'roll_offset': random.uniform(-5, 5),
                'pitch_offset': random.uniform(-3, 3),
                'yaw_offset': i * 90,
                'bias': [random.uniform(-0.01, 0.01) for _ in range(3)]
            })
    
    def run(self):
        self.running = True
        start_time = time.time()
        
        while self.running:
            t = time.time() - start_time + self.time_offset
            
            for i in range(self.num_imus):
                p = self.params[i]
                
                # Generate smooth sinusoidal motion with some noise
                roll = p['roll_amp'] * math.sin(2 * math.pi * p['roll_freq'] * t) + p['roll_offset']
                pitch = p['pitch_amp'] * math.sin(2 * math.pi * p['pitch_freq'] * t + 0.5) + p['pitch_offset']
                yaw = p['yaw_amp'] * math.sin(2 * math.pi * p['yaw_freq'] * t) + p['yaw_offset']
                
                # Add small noise
                roll += random.gauss(0, 0.3)
                pitch += random.gauss(0, 0.2)
                yaw += random.gauss(0, 0.5)
                
                self.callback(i, roll, pitch, yaw, int(t * 1e6), p['bias'])
            
            time.sleep(0.02)  # 50 Hz
    
    def stop(self):
        self.running = False


# =============================================================================
# UI Components
# =============================================================================

class UIComponent:
    """Base class for UI components"""
    
    def __init__(self, x: int, y: int, width: int, height: int, config: Config):
        self.rect = pygame.Rect(x, y, width, height)
        self.config = config
    
    def draw(self, surface: pygame.Surface):
        raise NotImplementedError
    
    def draw_panel_bg(self, surface: pygame.Surface, title: str = ""):
        # Background
        pygame.draw.rect(surface, self.config.PANEL_BG, self.rect, border_radius=8)
        pygame.draw.rect(surface, self.config.PANEL_BORDER, self.rect, width=2, border_radius=8)
        
        # Title
        if title:
            font = pygame.font.SysFont('Arial', 14, bold=True)
            text = font.render(title, True, self.config.TEXT_SECONDARY)
            surface.blit(text, (self.rect.x + 10, self.rect.y + 8))


class ArtificialHorizon(UIComponent):
    """Primary Flight Display - Attitude Indicator"""
    
    def __init__(self, x: int, y: int, size: int, config: Config):
        super().__init__(x, y, size, size, config)
        self.size = size
        self.center = (x + size // 2, y + size // 2)
        self.radius = size // 2 - 10
        
    def draw(self, surface: pygame.Surface, roll: float, pitch: float):
        cx, cy = self.center
        r = self.radius
        
        # Create clipping mask
        clip_surface = pygame.Surface((self.size, self.size), pygame.SRCALPHA)
        pygame.draw.circle(clip_surface, (255, 255, 255, 255), (self.size // 2, self.size // 2), r)
        
        # Create horizon surface
        horizon_surface = pygame.Surface((self.size * 2, self.size * 2))
        h_cx, h_cy = self.size, self.size
        
        # Calculate pitch offset (pixels per degree)
        pitch_scale = r / 30.0  # 30 degrees fills half the display
        pitch_offset = pitch * pitch_scale
        
        # Draw sky and ground
        horizon_surface.fill(self.config.SKY_COLOR)
        ground_rect = pygame.Rect(0, h_cy + pitch_offset, self.size * 2, self.size)
        pygame.draw.rect(horizon_surface, self.config.GROUND_COLOR, ground_rect)
        
        # Horizon line
        pygame.draw.line(horizon_surface, self.config.HORIZON_LINE,
                        (0, h_cy + pitch_offset), (self.size * 2, h_cy + pitch_offset), 3)
        
        # Pitch ladder
        for angle in range(-30, 35, 10):
            if angle == 0:
                continue
            y_pos = h_cy + pitch_offset - angle * pitch_scale
            line_width = 60 if abs(angle) <= 20 else 40
            
            # Draw pitch line
            pygame.draw.line(horizon_surface, self.config.PITCH_LINES,
                           (h_cx - line_width, y_pos), (h_cx + line_width, y_pos), 2)
            
            # Draw angle text
            font = pygame.font.SysFont('Arial', 12, bold=True)
            text = font.render(str(abs(angle)), True, self.config.PITCH_LINES)
            horizon_surface.blit(text, (h_cx - line_width - 25, y_pos - 6))
            horizon_surface.blit(text, (h_cx + line_width + 8, y_pos - 6))
        
        # Rotate horizon surface
        rotated = pygame.transform.rotate(horizon_surface, roll)
        rot_rect = rotated.get_rect(center=(self.size // 2, self.size // 2))
        
        # Apply to clipped surface
        temp_surface = pygame.Surface((self.size, self.size))
        temp_surface.fill((0, 0, 0))
        temp_surface.blit(rotated, rot_rect)
        temp_surface.blit(clip_surface, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
        
        # Draw to main surface
        surface.blit(temp_surface, (self.rect.x, self.rect.y))
        
        # Draw fixed aircraft symbol
        self._draw_aircraft_symbol(surface, cx, cy)
        
        # Draw roll indicator arc
        self._draw_roll_indicator(surface, cx, cy, r, roll)
        
        # Draw border
        pygame.draw.circle(surface, self.config.PANEL_BORDER, (cx, cy), r + 5, 3)
        
        # Draw attitude values
        self._draw_attitude_values(surface, roll, pitch)
    
    def _draw_aircraft_symbol(self, surface: pygame.Surface, cx: int, cy: int):
        color = self.config.AIRCRAFT_SYMBOL
        # Center dot
        pygame.draw.circle(surface, color, (cx, cy), 6)
        pygame.draw.circle(surface, (0, 0, 0), (cx, cy), 4)
        
        # Wings
        pygame.draw.line(surface, color, (cx - 80, cy), (cx - 20, cy), 4)
        pygame.draw.line(surface, color, (cx + 20, cy), (cx + 80, cy), 4)
        
        # Wing tips
        pygame.draw.line(surface, color, (cx - 80, cy), (cx - 80, cy + 15), 4)
        pygame.draw.line(surface, color, (cx + 80, cy), (cx + 80, cy + 15), 4)
    
    def _draw_roll_indicator(self, surface: pygame.Surface, cx: int, cy: int, r: int, roll: float):
        # Roll arc at top
        arc_r = r + 15
        
        # Draw tick marks
        for angle in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            rad = math.radians(angle - 90)
            x1 = cx + int((arc_r - 10) * math.cos(rad))
            y1 = cy + int((arc_r - 10) * math.sin(rad))
            x2 = cx + int((arc_r) * math.cos(rad))
            y2 = cy + int((arc_r) * math.sin(rad))
            
            width = 3 if angle % 30 == 0 else 2
            pygame.draw.line(surface, self.config.TEXT_PRIMARY, (x1, y1), (x2, y2), width)
        
        # Roll pointer (triangle)
        roll_rad = math.radians(-roll - 90)
        ptr_x = cx + int((arc_r - 5) * math.cos(roll_rad))
        ptr_y = cy + int((arc_r - 5) * math.sin(roll_rad))
        
        # Calculate triangle points
        angle1 = roll_rad + math.radians(150)
        angle2 = roll_rad + math.radians(210)
        pts = [
            (ptr_x, ptr_y),
            (ptr_x + int(12 * math.cos(angle1)), ptr_y + int(12 * math.sin(angle1))),
            (ptr_x + int(12 * math.cos(angle2)), ptr_y + int(12 * math.sin(angle2)))
        ]
        pygame.draw.polygon(surface, self.config.AIRCRAFT_SYMBOL, pts)
    
    def _draw_attitude_values(self, surface: pygame.Surface, roll: float, pitch: float):
        font = pygame.font.SysFont('Consolas', 16, bold=True)
        
        # Roll value (bottom left)
        roll_text = f"ROLL {roll:+6.1f}°"
        text = font.render(roll_text, True, self.config.TEXT_ACCENT)
        surface.blit(text, (self.rect.x + 10, self.rect.bottom - 25))
        
        # Pitch value (bottom right)
        pitch_text = f"PITCH {pitch:+6.1f}°"
        text = font.render(pitch_text, True, self.config.TEXT_ACCENT)
        surface.blit(text, (self.rect.right - 130, self.rect.bottom - 25))


class HeadingIndicator(UIComponent):
    """Compass Rose / Heading Indicator"""
    
    def __init__(self, x: int, y: int, width: int, height: int, config: Config):
        super().__init__(x, y, width, height, config)
        
    def draw(self, surface: pygame.Surface, heading: float):
        self.draw_panel_bg(surface)
        
        cx = self.rect.centerx
        cy = self.rect.y + 50
        
        # Normalize heading to 0-360
        heading = heading % 360
        if heading < 0:
            heading += 360
        
        # Draw compass tape
        tape_width = self.rect.width - 40
        tape_height = 40
        tape_rect = pygame.Rect(self.rect.x + 20, cy, tape_width, tape_height)
        pygame.draw.rect(surface, (40, 45, 55), tape_rect, border_radius=4)
        
        # Draw tick marks and labels
        pixels_per_degree = tape_width / 60  # Show 60 degrees
        
        for deg_offset in range(-35, 36):
            deg = (heading + deg_offset) % 360
            x = cx + int(deg_offset * pixels_per_degree)
            
            if self.rect.x + 20 < x < self.rect.right - 20:
                if deg % 10 == 0:
                    pygame.draw.line(surface, self.config.TEXT_PRIMARY,
                                   (x, cy + 5), (x, cy + 20), 2)
                    
                    # Cardinal/intercardinal labels
                    labels = {0: 'N', 90: 'E', 180: 'S', 270: 'W',
                             45: 'NE', 135: 'SE', 225: 'SW', 315: 'NW'}
                    
                    if deg in labels:
                        font = pygame.font.SysFont('Arial', 14, bold=True)
                        color = self.config.AIRCRAFT_SYMBOL if deg in [0, 90, 180, 270] else self.config.TEXT_PRIMARY
                        text = font.render(labels[deg], True, color)
                    else:
                        font = pygame.font.SysFont('Arial', 11)
                        text = font.render(str(int(deg)), True, self.config.TEXT_SECONDARY)
                    
                    text_rect = text.get_rect(centerx=x, top=cy + 22)
                    surface.blit(text, text_rect)
                    
                elif deg % 5 == 0:
                    pygame.draw.line(surface, self.config.TEXT_SECONDARY,
                                   (x, cy + 10), (x, cy + 18), 1)
        
        # Center pointer (triangle)
        pts = [(cx, cy), (cx - 8, cy - 12), (cx + 8, cy - 12)]
        pygame.draw.polygon(surface, self.config.AIRCRAFT_SYMBOL, pts)
        
        # Heading value
        font = pygame.font.SysFont('Consolas', 24, bold=True)
        hdg_text = f"{int(heading):03d}°"
        text = font.render(hdg_text, True, self.config.TEXT_ACCENT)
        text_rect = text.get_rect(centerx=cx, top=cy + tape_height + 10)
        
        # Background box for heading
        bg_rect = text_rect.inflate(20, 6)
        pygame.draw.rect(surface, (50, 55, 65), bg_rect, border_radius=4)
        pygame.draw.rect(surface, self.config.TEXT_ACCENT, bg_rect, width=1, border_radius=4)
        surface.blit(text, text_rect)


class IMUPanel(UIComponent):
    """Panel showing data from one IMU"""
    
    def __init__(self, x: int, y: int, width: int, height: int, config: Config, imu_id: int):
        super().__init__(x, y, width, height, config)
        self.imu_id = imu_id
        self.selected = False
        
    def draw(self, surface: pygame.Surface, imu: IMUData):
        # Background with selection highlight
        if self.selected:
            pygame.draw.rect(surface, self.config.TEXT_ACCENT, self.rect.inflate(4, 4), 
                           width=2, border_radius=10)
        
        pygame.draw.rect(surface, self.config.PANEL_BG, self.rect, border_radius=8)
        
        # Status indicator
        age = imu.get_age()
        if age < 0.5:
            status_color = self.config.STATUS_OK
            status_text = "ACTIVE"
        elif age < 2.0:
            status_color = self.config.STATUS_WARN
            status_text = "STALE"
        else:
            status_color = self.config.STATUS_OFFLINE
            status_text = "OFFLINE"
        
        # Header
        pygame.draw.rect(surface, status_color, 
                        pygame.Rect(self.rect.x, self.rect.y, self.rect.width, 28),
                        border_top_left_radius=8, border_top_right_radius=8)
        
        font_header = pygame.font.SysFont('Arial', 14, bold=True)
        header = font_header.render(f"IMU {self.imu_id}", True, (255, 255, 255))
        surface.blit(header, (self.rect.x + 10, self.rect.y + 6))
        
        status = font_header.render(status_text, True, (255, 255, 255))
        surface.blit(status, (self.rect.right - status.get_width() - 10, self.rect.y + 6))
        
        # Data
        y = self.rect.y + 38
        font = pygame.font.SysFont('Consolas', 15)
        font_small = pygame.font.SysFont('Consolas', 12)
        
        if age < 2.0:
            # Attitude
            for label, value in [("Roll", imu.roll), ("Pitch", imu.pitch), ("Yaw", imu.yaw)]:
                text = font.render(f"{label:6s} {value:+7.2f}°", True, self.config.TEXT_PRIMARY)
                surface.blit(text, (self.rect.x + 10, y))
                y += 20
            
            y += 5
            
            # Gyro bias
            pygame.draw.line(surface, self.config.PANEL_BORDER, 
                           (self.rect.x + 10, y), (self.rect.right - 10, y))
            y += 8
            
            bias_label = font_small.render("Gyro Bias (rad/s)", True, self.config.TEXT_SECONDARY)
            surface.blit(bias_label, (self.rect.x + 10, y))
            y += 16
            
            for i, axis in enumerate(['X', 'Y', 'Z']):
                bias_val = imu.gyro_bias[i] if i < len(imu.gyro_bias) else 0
                text = font_small.render(f"{axis}: {bias_val:+.5f}", True, self.config.TEXT_SECONDARY)
                surface.blit(text, (self.rect.x + 10, y))
                y += 14
            
            y += 5
            
            # Update rate
            rate = imu.get_update_rate()
            rate_color = self.config.STATUS_OK if rate > 40 else self.config.STATUS_WARN
            rate_text = font_small.render(f"Rate: {rate:.0f} Hz", True, rate_color)
            surface.blit(rate_text, (self.rect.x + 10, y))
        else:
            # Offline message
            font_msg = pygame.font.SysFont('Arial', 13)
            msg = font_msg.render("No data received", True, self.config.TEXT_SECONDARY)
            msg_rect = msg.get_rect(center=(self.rect.centerx, self.rect.centery + 20))
            surface.blit(msg, msg_rect)


class StatusBar(UIComponent):
    """Bottom status bar"""
    
    def __init__(self, x: int, y: int, width: int, height: int, config: Config):
        super().__init__(x, y, width, height, config)
        
    def draw(self, surface: pygame.Surface, connection_info: dict):
        pygame.draw.rect(surface, self.config.PANEL_BG, self.rect)
        pygame.draw.line(surface, self.config.PANEL_BORDER,
                        (self.rect.x, self.rect.y), (self.rect.right, self.rect.y), 2)
        
        font = pygame.font.SysFont('Consolas', 12)
        x = self.rect.x + 15
        y = self.rect.y + 8
        
        # Connection status
        if connection_info.get('mode') == 'simulation':
            status_text = "● SIMULATION MODE"
            status_color = self.config.STATUS_WARN
        elif connection_info.get('connected'):
            status_text = f"● Connected: {connection_info.get('port', 'N/A')}"
            status_color = self.config.STATUS_OK
        else:
            status_text = f"○ Disconnected: {connection_info.get('error', 'No connection')}"
            status_color = self.config.STATUS_ERROR
        
        text = font.render(status_text, True, status_color)
        surface.blit(text, (x, y))
        
        # Stats
        x = self.rect.centerx - 100
        msgs = connection_info.get('messages', 0)
        bytes_rx = connection_info.get('bytes', 0)
        stats = f"Messages: {msgs:,}  |  Bytes: {bytes_rx:,}"
        text = font.render(stats, True, self.config.TEXT_SECONDARY)
        surface.blit(text, (x, y))
        
        # Time
        time_str = time.strftime("%H:%M:%S")
        text = font.render(time_str, True, self.config.TEXT_SECONDARY)
        surface.blit(text, (self.rect.right - 80, y))


class HelpOverlay:
    """Help overlay panel"""
    
    def __init__(self, config: Config):
        self.config = config
        self.visible = False
        
    def draw(self, surface: pygame.Surface):
        if not self.visible:
            return
            
        # Dim background
        overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 180))
        surface.blit(overlay, (0, 0))
        
        # Help box
        box_width, box_height = 400, 350
        box_x = (surface.get_width() - box_width) // 2
        box_y = (surface.get_height() - box_height) // 2
        box_rect = pygame.Rect(box_x, box_y, box_width, box_height)
        
        pygame.draw.rect(surface, self.config.PANEL_BG, box_rect, border_radius=12)
        pygame.draw.rect(surface, self.config.TEXT_ACCENT, box_rect, width=2, border_radius=12)
        
        # Title
        font_title = pygame.font.SysFont('Arial', 20, bold=True)
        title = font_title.render("Keyboard Shortcuts", True, self.config.TEXT_ACCENT)
        surface.blit(title, (box_x + 20, box_y + 20))
        
        # Shortcuts
        font = pygame.font.SysFont('Consolas', 14)
        shortcuts = [
            ("1-4", "Select IMU 1-4"),
            ("H", "Toggle this help"),
            ("R", "Reset statistics"),
            ("SPACE", "Pause/Resume"),
            ("Q / ESC", "Quit application"),
            ("", ""),
            ("", "Connection:"),
            ("--port", "Specify serial port"),
            ("--baud", "Specify baud rate"),
            ("--sim", "Simulation mode"),
        ]
        
        y = box_y + 60
        for key, desc in shortcuts:
            if key:
                key_text = font.render(f"{key:12s}", True, self.config.AIRCRAFT_SYMBOL)
                desc_text = font.render(desc, True, self.config.TEXT_PRIMARY)
                surface.blit(key_text, (box_x + 30, y))
                surface.blit(desc_text, (box_x + 150, y))
            elif desc:
                desc_text = font.render(desc, True, self.config.TEXT_SECONDARY)
                surface.blit(desc_text, (box_x + 30, y))
            y += 24
        
        # Close hint
        hint = font.render("Press H to close", True, self.config.TEXT_SECONDARY)
        surface.blit(hint, (box_x + box_width // 2 - hint.get_width() // 2, box_y + box_height - 35))


# =============================================================================
# Main Application
# =============================================================================

class GroundStation:
    """Main application class"""
    
    def __init__(self, args):
        self.args = args
        self.config = Config()
        self.running = True
        self.paused = False
        self.selected_imu = 0
        
        # Data
        self.imus = [IMUData(i) for i in range(4)]
        
        # Connection
        self.data_source = None
        self.connection_info = {
            'mode': 'disconnected',
            'connected': False,
            'port': None,
            'error': '',
            'messages': 0,
            'bytes': 0
        }
        
        # Initialize pygame
        pygame.init()
        pygame.display.set_caption("UAV Ground Station")
        
        self.screen = pygame.display.set_mode(
            (self.config.WINDOW_WIDTH, self.config.WINDOW_HEIGHT)
        )
        self.clock = pygame.time.Clock()
        
        # Create UI components
        self._create_ui()
        
    def _create_ui(self):
        """Create UI components"""
        cfg = self.config
        
        # Main PFD (left side)
        pfd_size = 400
        pfd_x = 40
        pfd_y = 50
        self.attitude_indicator = ArtificialHorizon(pfd_x, pfd_y, pfd_size, cfg)
        
        # Heading indicator (below PFD)
        self.heading_indicator = HeadingIndicator(
            pfd_x, pfd_y + pfd_size + 20, pfd_size, 120, cfg
        )
        
        # IMU panels (right side)
        panel_width = 200
        panel_height = 200
        panel_x = 500
        panel_y = 50
        panel_gap = 15
        
        self.imu_panels = []
        for i in range(4):
            row = i // 2
            col = i % 2
            x = panel_x + col * (panel_width + panel_gap)
            y = panel_y + row * (panel_height + panel_gap)
            panel = IMUPanel(x, y, panel_width, panel_height, cfg, i)
            self.imu_panels.append(panel)
        
        self.imu_panels[0].selected = True
        
        # Status bar
        self.status_bar = StatusBar(
            0, cfg.WINDOW_HEIGHT - 35, cfg.WINDOW_WIDTH, 35, cfg
        )
        
        # Help overlay
        self.help_overlay = HelpOverlay(cfg)
        
    def on_attitude_update(self, imu_id: int, roll: float, pitch: float, 
                           yaw: float, timestamp: int, bias: Optional[List[float]]):
        """Callback for attitude data updates"""
        if 0 <= imu_id < len(self.imus) and not self.paused:
            self.imus[imu_id].update(roll, pitch, yaw, timestamp, bias)
    
    def start_connection(self):
        """Start data source (serial or simulator)"""
        if self.args.sim:
            self.data_source = Simulator(self.on_attitude_update)
            self.connection_info['mode'] = 'simulation'
            self.connection_info['connected'] = True
            self.data_source.start()
            print("Starting simulation mode...")
            
        elif self.args.port and SERIAL_AVAILABLE:
            self.data_source = SerialReader(
                self.args.port, self.args.baud, self.on_attitude_update
            )
            self.connection_info['mode'] = 'serial'
            self.connection_info['port'] = self.args.port
            self.data_source.start()
            print(f"Connecting to {self.args.port} @ {self.args.baud}...")
            
        else:
            self.connection_info['mode'] = 'disconnected'
            self.connection_info['error'] = 'No port specified. Use --port or --sim'
    
    def update_connection_info(self):
        """Update connection info from data source"""
        if self.data_source is None:
            return
            
        if isinstance(self.data_source, SerialReader):
            self.connection_info['connected'] = self.data_source.connected
            self.connection_info['error'] = self.data_source.error_message
            self.connection_info['messages'] = self.data_source.messages_received
            self.connection_info['bytes'] = self.data_source.bytes_received
        elif isinstance(self.data_source, Simulator):
            total_msgs = sum(imu.update_count for imu in self.imus)
            self.connection_info['messages'] = total_msgs
            self.connection_info['bytes'] = total_msgs * 50  # Approximate
    
    def handle_events(self) -> bool:
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
                
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    return False
                    
                elif event.key in [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4]:
                    new_imu = event.key - pygame.K_1
                    self.imu_panels[self.selected_imu].selected = False
                    self.selected_imu = new_imu
                    self.imu_panels[self.selected_imu].selected = True
                    
                elif event.key == pygame.K_h:
                    self.help_overlay.visible = not self.help_overlay.visible
                    
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                    
                elif event.key == pygame.K_r:
                    # Reset statistics
                    for imu in self.imus:
                        imu.history.clear()
                        imu.update_count = 0
                    if self.data_source and hasattr(self.data_source, 'messages_received'):
                        self.data_source.messages_received = 0
                        self.data_source.bytes_received = 0
                        
            if event.type == pygame.MOUSEBUTTONDOWN:
                # Check if clicked on an IMU panel
                for i, panel in enumerate(self.imu_panels):
                    if panel.rect.collidepoint(event.pos):
                        self.imu_panels[self.selected_imu].selected = False
                        self.selected_imu = i
                        self.imu_panels[i].selected = True
                        break
        
        return True
    
    def draw(self):
        """Draw all UI components"""
        self.screen.fill(self.config.BG_COLOR)
        
        # Get selected IMU data
        imu = self.imus[self.selected_imu]
        
        # Draw title
        font_title = pygame.font.SysFont('Arial', 24, bold=True)
        title = font_title.render("UAV Ground Station", True, self.config.TEXT_PRIMARY)
        self.screen.blit(title, (20, 12))
        
        # Pause indicator
        if self.paused:
            pause_font = pygame.font.SysFont('Arial', 18, bold=True)
            pause_text = pause_font.render("⏸ PAUSED", True, self.config.STATUS_WARN)
            self.screen.blit(pause_text, (200, 15))
        
        # Draw PFD components
        self.attitude_indicator.draw(self.screen, imu.roll, imu.pitch)
        self.heading_indicator.draw(self.screen, imu.yaw)
        
        # Draw IMU panels
        for i, panel in enumerate(self.imu_panels):
            panel.draw(self.screen, self.imus[i])
        
        # Additional info panel (right of IMU panels)
        self._draw_info_panel()
        
        # Draw status bar
        self.update_connection_info()
        self.status_bar.draw(self.screen, self.connection_info)
        
        # Draw help overlay
        self.help_overlay.draw(self.screen)
        
        # Help hint
        if not self.help_overlay.visible:
            font = pygame.font.SysFont('Arial', 12)
            hint = font.render("Press H for help", True, self.config.TEXT_SECONDARY)
            self.screen.blit(hint, (self.config.WINDOW_WIDTH - 110, 15))
        
        pygame.display.flip()
    
    def _draw_info_panel(self):
        """Draw additional information panel"""
        x = 920
        y = 50
        width = 250
        height = 430
        
        rect = pygame.Rect(x, y, width, height)
        pygame.draw.rect(self.screen, self.config.PANEL_BG, rect, border_radius=8)
        pygame.draw.rect(self.screen, self.config.PANEL_BORDER, rect, width=2, border_radius=8)
        
        # Title
        font_title = pygame.font.SysFont('Arial', 14, bold=True)
        title = font_title.render("System Status", True, self.config.TEXT_SECONDARY)
        self.screen.blit(title, (x + 10, y + 10))
        
        # Active IMUs count
        font = pygame.font.SysFont('Consolas', 13)
        py = y + 40
        
        active_count = sum(1 for imu in self.imus if imu.is_active())
        active_color = self.config.STATUS_OK if active_count > 0 else self.config.STATUS_ERROR
        
        text = font.render(f"Active IMUs: {active_count}/4", True, active_color)
        self.screen.blit(text, (x + 15, py))
        py += 25
        
        pygame.draw.line(self.screen, self.config.PANEL_BORDER, 
                        (x + 15, py), (x + width - 15, py))
        py += 15
        
        # All IMU summary
        for i, imu in enumerate(self.imus):
            if imu.is_active():
                color = self.config.STATUS_OK if i != self.selected_imu else self.config.TEXT_ACCENT
                line1 = f"IMU{i}: R{imu.roll:+6.1f} P{imu.pitch:+6.1f}"
                line2 = f"      Y{imu.yaw:+6.1f} @ {imu.get_update_rate():.0f}Hz"
            else:
                color = self.config.STATUS_OFFLINE
                line1 = f"IMU{i}: Offline"
                line2 = ""
            
            text = font.render(line1, True, color)
            self.screen.blit(text, (x + 15, py))
            py += 16
            
            if line2:
                text = font.render(line2, True, color)
                self.screen.blit(text, (x + 15, py))
                py += 16
            
            py += 8
        
        # Attitude comparison (if multiple IMUs active)
        active_imus = [imu for imu in self.imus if imu.is_active()]
        if len(active_imus) >= 2:
            py += 5
            pygame.draw.line(self.screen, self.config.PANEL_BORDER, 
                            (x + 15, py), (x + width - 15, py))
            py += 12
            
            font_small = pygame.font.SysFont('Consolas', 12)
            title = font_small.render("Attitude Spread:", True, self.config.TEXT_SECONDARY)
            self.screen.blit(title, (x + 15, py))
            py += 18
            
            rolls = [imu.roll for imu in active_imus]
            pitches = [imu.pitch for imu in active_imus]
            yaws = [imu.yaw for imu in active_imus]
            
            roll_spread = max(rolls) - min(rolls)
            pitch_spread = max(pitches) - min(pitches)
            yaw_spread = max(yaws) - min(yaws)
            
            for name, val in [("Roll", roll_spread), ("Pitch", pitch_spread), ("Yaw", yaw_spread)]:
                color = self.config.STATUS_OK if val < 5 else self.config.STATUS_WARN
                text = font_small.render(f"  {name}: ±{val/2:.2f}°", True, color)
                self.screen.blit(text, (x + 15, py))
                py += 16
    
    def run(self):
        """Main application loop"""
        print("=" * 60)
        print("  UAV Ground Station")
        print("=" * 60)
        
        self.start_connection()
        
        while self.running:
            self.running = self.handle_events()
            self.draw()
            self.clock.tick(self.config.FPS)
        
        # Cleanup
        if self.data_source:
            self.data_source.stop()
        pygame.quit()
        print("\nGoodbye!")


# =============================================================================
# Port Selection UI
# =============================================================================

def select_port_interactive() -> Optional[str]:
    """Interactive port selection dialog"""
    if not SERIAL_AVAILABLE:
        print("Serial not available. Use --sim for simulation mode.")
        return None
    
    ports = list(list_ports.comports())
    
    if not ports:
        print("No serial ports found.")
        return None
    
    print("\nAvailable serial ports:")
    print("-" * 40)
    
    for i, port in enumerate(ports, 1):
        print(f"  [{i}] {port.device}")
        if port.description and port.description != 'n/a':
            print(f"      {port.description}")
    
    print("-" * 40)
    print("  [S] Simulation mode")
    print("  [Q] Quit")
    print()
    
    while True:
        try:
            choice = input("Select port (number or S/Q): ").strip().upper()
            
            if choice == 'Q':
                return None
            elif choice == 'S':
                return 'SIM'
            else:
                idx = int(choice) - 1
                if 0 <= idx < len(ports):
                    return ports[idx].device
                print("Invalid selection.")
        except ValueError:
            print("Invalid input.")
        except KeyboardInterrupt:
            return None


# =============================================================================
# Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='UAV Ground Station - Professional Flight Display',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --sim                     # Simulation mode
  %(prog)s --port /dev/ttyUSB0       # Connect to serial port
  %(prog)s --port COM3 --baud 921600 # Windows with custom baud rate
  %(prog)s                           # Interactive port selection
        """
    )
    
    parser.add_argument('--port', '-p', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--sim', '-s', action='store_true', help='Simulation mode (no hardware needed)')
    
    args = parser.parse_args()
    
    # Interactive mode if no port/sim specified
    if not args.port and not args.sim:
        port = select_port_interactive()
        if port is None:
            print("No port selected. Exiting.")
            return
        elif port == 'SIM':
            args.sim = True
        else:
            args.port = port
    
    # Run application
    app = GroundStation(args)
    app.run()


if __name__ == '__main__':
    main()
