"""
UAV Ground Control Station - Telemetry Simulator

Tạo dữ liệu giả lập để test GUI mà không cần kết nối thực.
"""

import math
import time
import threading
from typing import Optional

from PySide6.QtCore import QObject, Signal

from .packet import TelemetryData


class TelemetrySimulator(QObject):
    """
    Simulates telemetry data for testing GUI.
    
    Generates realistic-looking IMU, attitude, and sensor data
    without needing actual hardware connection.
    """
    
    # Signals (same as SerialReceiver for compatibility)
    packet_received = Signal(object)    # TelemetryData
    connection_changed = Signal(bool)   # connected state
    error_occurred = Signal(str)        # error message
    stats_updated = Signal(dict)        # statistics
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self._running = False
        self._thread: Optional[threading.Thread] = None
        
        # Simulation state
        self._time = 0.0
        self._sequence = 0
        
        # Simulated motion parameters
        self._roll_freq = 0.5    # Hz
        self._pitch_freq = 0.3
        self._yaw_freq = 0.1
        
        self._roll_amp = 0.3     # rad (~17 deg)
        self._pitch_amp = 0.2    # rad (~11 deg)
        self._yaw_drift = 0.05   # rad/s
        
        # Statistics
        self._stats = {
            'bytes_received': 0,
            'packets_decoded': 0,
            'sequence_drops': 0,
            'crc_errors': 0
        }
    
    def start(self, rate_hz: int = 100):
        """Start simulation at specified rate"""
        if self._running:
            return
        
        self._running = True
        self._rate_hz = rate_hz
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        
        self.connection_changed.emit(True)
    
    def stop(self):
        """Stop simulation"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        
        self.connection_changed.emit(False)
    
    def _run_loop(self):
        """Main simulation loop"""
        period = 1.0 / self._rate_hz
        start_time = time.time()
        
        while self._running:
            loop_start = time.time()
            
            # Generate data
            data = self._generate_packet()
            
            # Emit signal
            self.packet_received.emit(data)
            
            # Update stats
            self._stats['bytes_received'] += 128
            self._stats['packets_decoded'] += 1
            
            # Emit stats every second
            if self._sequence % self._rate_hz == 0:
                self.stats_updated.emit(self._stats.copy())
            
            # Sleep to maintain rate
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            self._time = time.time() - start_time
    
    def _generate_packet(self) -> TelemetryData:
        """Generate simulated telemetry data"""
        t = self._time
        
        # Simulate oscillating attitude
        roll = self._roll_amp * math.sin(2 * math.pi * self._roll_freq * t)
        pitch = self._pitch_amp * math.sin(2 * math.pi * self._pitch_freq * t)
        yaw = (self._yaw_drift * t) % (2 * math.pi) - math.pi
        
        # Convert euler to quaternion
        qw, qx, qy, qz = self._euler_to_quat(roll, pitch, yaw)
        
        # Simulate gyro (derivative of angles + noise)
        noise = 0.01
        gyro_x = self._roll_amp * 2 * math.pi * self._roll_freq * math.cos(2 * math.pi * self._roll_freq * t)
        gyro_y = self._pitch_amp * 2 * math.pi * self._pitch_freq * math.cos(2 * math.pi * self._pitch_freq * t)
        gyro_z = self._yaw_drift
        
        # Add noise
        import random
        gyro_x += random.gauss(0, noise)
        gyro_y += random.gauss(0, noise)
        gyro_z += random.gauss(0, noise)
        
        # Simulate accelerometer (gravity + centrifugal + noise)
        # In level flight, accel ≈ [0, 0, -9.81] in body frame
        ax = 9.81 * math.sin(pitch) + random.gauss(0, 0.1)
        ay = -9.81 * math.sin(roll) * math.cos(pitch) + random.gauss(0, 0.1)
        az = -9.81 * math.cos(roll) * math.cos(pitch) + random.gauss(0, 0.1)
        
        # Simulate magnetometer (assuming North = +X in NED)
        mag_field = 0.5  # Gauss
        mx = mag_field * math.cos(yaw) * math.cos(pitch)
        my = mag_field * math.sin(yaw) * math.cos(pitch)
        mz = mag_field * math.sin(pitch)
        
        # Simulate barometer
        base_altitude = 100.0  # meters
        altitude = base_altitude + 5.0 * math.sin(0.1 * t)  # slow drift
        pressure = 101325 * math.exp(-altitude / 8500)  # barometric formula
        
        # Simulate GPS
        base_lat = 21.028511   # Hanoi
        base_lon = 105.804817
        lat = int((base_lat + 0.0001 * math.sin(0.05 * t)) * 1e7)
        lon = int((base_lon + 0.0001 * math.cos(0.05 * t)) * 1e7)
        alt_msl = int(altitude * 1000)  # mm
        ground_speed = int(5 * 100)  # 5 m/s in cm/s
        
        # System status
        health_level = 0  # good
        healthy_imus = 0x0F  # all 4 healthy
        sensor_flags = 0x3F  # all sensors ok
        
        # Create packet
        data = TelemetryData(
            # Header
            timestamp_us=int(t * 1_000_000),
            sequence=self._sequence & 0xFFFF,
            version=1,
            
            # IMU
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            accel_x=ax,
            accel_y=ay,
            accel_z=az,
            temperature=25.0 + 2.0 * math.sin(0.01 * t),
            
            # Magnetometer
            mag_x=mx,
            mag_y=my,
            mag_z=mz,
            
            # Barometer
            pressure=pressure,
            altitude=altitude,
            
            # GPS
            latitude=lat,
            longitude=lon,
            altitude_msl=alt_msl,
            ground_speed=ground_speed,
            heading=int(math.degrees(yaw) * 100),
            fix_type=3,  # 3D fix
            satellites=12,
            hdop=100,
            vdop=150,
            
            # Attitude
            qw=qw,
            qx=qx,
            qy=qy,
            qz=qz,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            innovation_var=0.01,
            
            # Status
            health_level=health_level,
            healthy_imus=healthy_imus,
            sensor_flags=sensor_flags,
            cpu_load=150,  # 15%
            battery_mv=12600,  # 12.6V
            loop_count=self._sequence,
            
            # PC timestamp
            pc_timestamp_ms=int(time.time() * 1000)
        )
        
        self._sequence += 1
        return data
    
    @staticmethod
    def _euler_to_quat(roll: float, pitch: float, yaw: float):
        """Convert Euler angles (ZYX) to quaternion [w, x, y, z]"""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qw, qx, qy, qz
