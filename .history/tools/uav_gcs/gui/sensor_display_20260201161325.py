"""
UAV Ground Control Station - Fast Sensor Display

Hiển thị số liệu sensor nhanh, chính xác như STM32 debug.
Không dùng plot - chỉ số liệu text với update rate cao.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QFrame, QScrollArea
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QColor, QPalette

import math


class ValueLabel(QLabel):
    """
    High-performance value label với color coding.
    Chỉ update khi giá trị thay đổi đáng kể.
    """
    
    def __init__(self, width: int = 100, decimals: int = 3, unit: str = "", parent=None):
        super().__init__(parent)
        
        self._decimals = decimals
        self._unit = unit
        self._last_value = None
        self._threshold = 10 ** (-decimals)  # Update threshold
        
        # Monospace font for aligned numbers
        font = QFont("Consolas, Monaco, monospace", 10)
        font.setStyleHint(QFont.Monospace)
        self.setFont(font)
        
        self.setFixedWidth(width)
        self.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.setStyleSheet("""
            QLabel {
                background-color: #1a1a1a;
                color: #00ff00;
                padding: 2px 5px;
                border: 1px solid #333;
                border-radius: 2px;
            }
        """)
        
        self.setText("---")
    
    def set_value(self, value: float, force: bool = False):
        """Update value - only if changed significantly"""
        if not force and self._last_value is not None:
            if abs(value - self._last_value) < self._threshold:
                return
        
        self._last_value = value
        
        # Format with fixed width
        if self._decimals <= 0:
            text = f"{int(round(value)):>8d}"
        elif abs(value) < 1000:
            text = f"{value:>{8}.{self._decimals}f}"
        else:
            precision = max(self._decimals - 1, 1)
            text = f"{value:>{8}.{precision}e}"
        
        if self._unit:
            text = f"{text} {self._unit}"
        
        self.setText(text)
    
    def set_color(self, color: str):
        """Set text color: 'green', 'yellow', 'red', 'white'"""
        colors = {
            'green': '#00ff00',
            'yellow': '#ffff00',
            'red': '#ff4444',
            'white': '#ffffff',
            'cyan': '#00ffff',
            'magenta': '#ff00ff'
        }
        c = colors.get(color, '#00ff00')
        self.setStyleSheet(f"""
            QLabel {{
                background-color: #1a1a1a;
                color: {c};
                padding: 2px 5px;
                border: 1px solid #333;
                border-radius: 2px;
            }}
        """)


class SensorGroup(QGroupBox):
    """Group box cho một loại sensor"""
    
    def __init__(self, title: str, parent=None):
        super().__init__(title, parent)
        
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #aaaaaa;
                border: 1px solid #444;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        
        self._layout = QGridLayout(self)
        self._layout.setSpacing(4)
        self._layout.setContentsMargins(8, 12, 8, 8)
        self._row = 0
        self._labels = {}
    
    def add_value(self, name: str, width: int = 100, decimals: int = 3, unit: str = "") -> ValueLabel:
        """Add a labeled value display"""
        name_label = QLabel(name + ":")
        name_label.setStyleSheet("color: #888; font-size: 10px;")
        name_label.setFixedWidth(60)
        
        value_label = ValueLabel(width, decimals, unit)
        
        self._layout.addWidget(name_label, self._row, 0)
        self._layout.addWidget(value_label, self._row, 1)
        
        self._labels[name] = value_label
        self._row += 1
        
        return value_label
    
    def add_vector(self, name: str, components: list = ['X', 'Y', 'Z'], 
                   decimals: int = 3, unit: str = "") -> dict:
        """Add a vector display (X, Y, Z)"""
        labels = {}
        
        name_label = QLabel(name + ":")
        name_label.setStyleSheet("color: #888; font-size: 10px;")
        
        self._layout.addWidget(name_label, self._row, 0)
        
        vec_layout = QHBoxLayout()
        vec_layout.setSpacing(4)
        
        for comp in components:
            comp_label = QLabel(comp)
            comp_label.setStyleSheet("color: #666; font-size: 9px;")
            comp_label.setFixedWidth(12)
            
            value_label = ValueLabel(70, decimals, "")
            labels[comp] = value_label
            
            vec_layout.addWidget(comp_label)
            vec_layout.addWidget(value_label)
        
        if unit:
            unit_label = QLabel(unit)
            unit_label.setStyleSheet("color: #666; font-size: 9px;")
            vec_layout.addWidget(unit_label)
        
        vec_layout.addStretch()
        
        vec_widget = QWidget()
        vec_widget.setLayout(vec_layout)
        self._layout.addWidget(vec_widget, self._row, 1)
        
        self._labels[name] = labels
        self._row += 1
        
        return labels
    
    def get_label(self, name: str):
        return self._labels.get(name)


class FastSensorDisplay(QWidget):
    """
    Fast sensor data display panel.
    
    Hiển thị tất cả sensor data với update rate cao.
    Layout tối ưu cho việc đọc số liệu nhanh.
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup optimized layout"""
        main_layout = QHBoxLayout(self)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Left column - Raw sensors
        left_col = QVBoxLayout()
        left_col.setSpacing(8)
        
        # IMU Group
        self._imu_group = SensorGroup("IMU (Raw)")
        self._gyro = self._imu_group.add_vector("Gyro", ['X', 'Y', 'Z'], 4, "rad/s")
        self._accel = self._imu_group.add_vector("Accel", ['X', 'Y', 'Z'], 3, "m/s²")
        self._imu_temp = self._imu_group.add_value("Temp", 80, 1, "°C")
        left_col.addWidget(self._imu_group)
        
        # Magnetometer Group
        self._mag_group = SensorGroup("Magnetometer")
        self._mag = self._mag_group.add_vector("Field", ['X', 'Y', 'Z'], 4, "Ga")
        self._mag_heading = self._mag_group.add_value("Heading", 80, 1, "°")
        left_col.addWidget(self._mag_group)
        
        # Barometer Group
        self._baro_group = SensorGroup("Barometer")
        self._pressure = self._baro_group.add_value("Pressure", 100, 1, "Pa")
        self._baro_alt = self._baro_group.add_value("Altitude", 80, 2, "m")
        left_col.addWidget(self._baro_group)
        
        left_col.addStretch()
        main_layout.addLayout(left_col)
        
        # Middle column - Estimated/Fused
        mid_col = QVBoxLayout()
        mid_col.setSpacing(8)
        
        # Attitude Group (Estimated)
        self._att_group = SensorGroup("Attitude (Estimated)")
        self._euler = self._att_group.add_vector("Euler", ['R', 'P', 'Y'], 2, "°")
        self._quat = self._att_group.add_vector("Quat", ['W', 'X', 'Y', 'Z'], 4, "")
        self._rates = self._att_group.add_vector("Rates", ['R', 'P', 'Y'], 3, "°/s")
        mid_col.addWidget(self._att_group)
        
        # Position/Velocity Group
        self._pos_group = SensorGroup("Position (GPS)")
        self._lat = self._pos_group.add_value("Lat", 120, 6, "°")
        self._lon = self._pos_group.add_value("Lon", 120, 6, "°")
        self._alt_msl = self._pos_group.add_value("Alt MSL", 80, 1, "m")
        self._gnd_speed = self._pos_group.add_value("GndSpd", 80, 1, "m/s")
        self._gps_heading = self._pos_group.add_value("Track", 80, 1, "°")
        mid_col.addWidget(self._pos_group)
        
        mid_col.addStretch()
        main_layout.addLayout(mid_col)
        
        # Right column - Status & Debug
        right_col = QVBoxLayout()
        right_col.setSpacing(8)
        
        # GPS Status
        self._gps_status = SensorGroup("GPS Status")
        self._fix_type = self._gps_status.add_value("Fix", 60, 0, "")
        self._sats = self._gps_status.add_value("Sats", 60, 0, "")
        self._hdop = self._gps_status.add_value("HDOP", 60, 1, "")
        self._vdop = self._gps_status.add_value("VDOP", 60, 1, "")
        right_col.addWidget(self._gps_status)
        
        # System Status
        self._sys_group = SensorGroup("System Status")
        self._health = self._sys_group.add_value("Health", 60, 0, "")
        self._cpu = self._sys_group.add_value("CPU", 60, 1, "%")
        self._battery = self._sys_group.add_value("Battery", 80, 2, "V")
        self._loop_cnt = self._sys_group.add_value("Loop#", 80, 0, "")
        right_col.addWidget(self._sys_group)
        
        # Timing/Debug
        self._debug_group = SensorGroup("Timing")
        self._timestamp = self._debug_group.add_value("MCU Time", 100, 3, "s")
        self._sequence = self._debug_group.add_value("Seq#", 80, 0, "")
        self._rate = self._debug_group.add_value("Rate", 60, 0, "Hz")
        right_col.addWidget(self._debug_group)
        
        right_col.addStretch()
        main_layout.addLayout(right_col)
    
    def update_data(self, data):
        """Update all displays with new telemetry data"""
        # IMU
        self._gyro['X'].set_value(data.gyro_x)
        self._gyro['Y'].set_value(data.gyro_y)
        self._gyro['Z'].set_value(data.gyro_z)
        
        self._accel['X'].set_value(data.accel_x)
        self._accel['Y'].set_value(data.accel_y)
        self._accel['Z'].set_value(data.accel_z)
        
        self._imu_temp.set_value(data.imu_temp)
        
        # Magnetometer
        self._mag['X'].set_value(data.mag_x)
        self._mag['Y'].set_value(data.mag_y)
        self._mag['Z'].set_value(data.mag_z)
        
        # Calculate mag heading
        mag_heading = math.degrees(math.atan2(data.mag_y, data.mag_x))
        self._mag_heading.set_value(mag_heading)
        
        # Barometer
        self._pressure.set_value(data.pressure)
        self._baro_alt.set_value(data.baro_alt)
        
        # Attitude (convert rad to deg for display)
        self._euler['R'].set_value(math.degrees(data.roll))
        self._euler['P'].set_value(math.degrees(data.pitch))
        self._euler['Y'].set_value(math.degrees(data.yaw))
        
        self._quat['W'].set_value(data.qw)
        self._quat['X'].set_value(data.qx)
        self._quat['Y'].set_value(data.qy)
        self._quat['Z'].set_value(data.qz)
        
        # Angular rates from gyro (convert to deg/s)
        self._rates['R'].set_value(math.degrees(data.gyro_x))
        self._rates['P'].set_value(math.degrees(data.gyro_y))
        self._rates['Y'].set_value(math.degrees(data.gyro_z))
        
        # GPS Position
        self._lat.set_value(data.latitude)
        self._lon.set_value(data.longitude)
        self._alt_msl.set_value(data.altitude_msl)
        self._gnd_speed.set_value(data.ground_speed)
        self._gps_heading.set_value(data.heading)
        
        # GPS Status
        fix_names = {0: "None", 1: "2D", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK"}
        self._fix_type.setText(fix_names.get(data.fix_type, "?"))
        self._fix_type.set_color('green' if data.fix_type >= 3 else 'yellow' if data.fix_type >= 2 else 'red')
        
        self._sats.set_value(data.satellites)
        self._sats.set_color('green' if data.satellites >= 8 else 'yellow' if data.satellites >= 4 else 'red')
        
        self._hdop.set_value(data.hdop)
        self._vdop.set_value(data.vdop)
        
        # System Status
        health_names = {0: "GOOD", 1: "WARN", 2: "CRIT", 3: "FAIL"}
        health_colors = {0: 'green', 1: 'yellow', 2: 'red', 3: 'red'}
        self._health.setText(health_names.get(data.health_level, "?"))
        self._health.set_color(health_colors.get(data.health_level, 'white'))
        
        self._cpu.set_value(data.cpu_load)
        self._cpu.set_color('green' if data.cpu_load < 70 else 'yellow' if data.cpu_load < 90 else 'red')
        
        self._battery.set_value(data.battery_mv / 1000.0)
        
        self._loop_cnt.set_value(data.loop_count)
        
        # Timing
        self._timestamp.set_value(data.timestamp_us / 1_000_000.0)
        self._sequence.set_value(data.sequence)
    
    def update_rate(self, rate_hz: float):
        """Update displayed rate"""
        self._rate.set_value(rate_hz)
        self._rate.set_color('green' if rate_hz >= 90 else 'yellow' if rate_hz >= 50 else 'red')
    
    def clear(self):
        """Clear all displays"""
        # Reset all to "---"
        pass
