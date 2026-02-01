"""
UAV Ground Control Station - Fast Sensor Display

Hiển thị số liệu sensor nhanh, chính xác như STM32 debug.
Không dùng plot - chỉ số liệu text với update rate cao.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QFrame, QScrollArea, QCheckBox
)
from PySide6.QtCore import Qt, Signal
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


class ImuFusionPanel(QGroupBox):
    """
    Panel hiển thị dữ liệu của cả 4 IMU và cho phép chọn IMU để fusion.
    """
    
    # Signal emitted when IMU selection changes
    imu_selection_changed = Signal(list)  # List of selected IMU indices
    
    def __init__(self, parent=None):
        super().__init__("IMU Fusion (4 sensors)", parent)
        
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #00aaff;
                border: 1px solid #0066aa;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QCheckBox {
                color: #aaa;
                font-size: 10px;
            }
            QCheckBox::indicator {
                width: 14px;
                height: 14px;
            }
            QCheckBox::indicator:checked {
                background-color: #00aa00;
                border: 1px solid #00ff00;
            }
            QCheckBox::indicator:unchecked {
                background-color: #333;
                border: 1px solid #666;
            }
        """)
        
        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        layout.setContentsMargins(6, 14, 6, 6)
        
        # Header row
        header = QHBoxLayout()
        header.setSpacing(2)
        
        header_labels = ["", "IMU", "Gx", "Gy", "Gz", "Ax", "Ay", "Az"]
        widths = [20, 35, 55, 55, 55, 55, 55, 55]
        for label, w in zip(header_labels, widths):
            lbl = QLabel(label)
            lbl.setStyleSheet("color: #666; font-size: 9px; font-weight: bold;")
            lbl.setFixedWidth(w)
            lbl.setAlignment(Qt.AlignCenter)
            header.addWidget(lbl)
        header.addStretch()
        layout.addLayout(header)
        
        # IMU rows
        self._checkboxes = []
        self._imu_labels = []  # Store labels for each IMU
        self._imu_values = []  # Store numeric values (None if not available)
        
        colors = ['#ff6666', '#66ff66', '#6666ff', '#ffff66']  # Red, Green, Blue, Yellow
        
        for i in range(4):
            row = QHBoxLayout()
            row.setSpacing(2)
            
            # Checkbox
            cb = QCheckBox()
            cb.setChecked(True)
            cb.setFixedWidth(20)
            cb.stateChanged.connect(self._on_selection_changed)
            self._checkboxes.append(cb)
            row.addWidget(cb)
            
            # IMU number
            imu_num = QLabel(f"#{i}")
            imu_num.setStyleSheet(f"color: {colors[i]}; font-weight: bold; font-size: 10px;")
            imu_num.setFixedWidth(35)
            imu_num.setAlignment(Qt.AlignCenter)
            row.addWidget(imu_num)
            
            # Gyro X, Y, Z
            imu_data = {}
            imu_values = {'gx': None, 'gy': None, 'gz': None, 'ax': None, 'ay': None, 'az': None}
            for axis in ['gx', 'gy', 'gz', 'ax', 'ay', 'az']:
                lbl = QLabel("---")
                lbl.setStyleSheet("""
                    background-color: #1a1a1a;
                    color: #00cc00;
                    padding: 1px 3px;
                    border: 1px solid #333;
                    font-family: monospace;
                    font-size: 9px;
                """)
                lbl.setFixedWidth(55)
                lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                imu_data[axis] = lbl
                row.addWidget(lbl)
            
            self._imu_labels.append(imu_data)
            self._imu_values.append(imu_values)
            row.addStretch()
            layout.addLayout(row)
        
        # Fused result row
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #444;")
        layout.addWidget(separator)
        
        fused_row = QHBoxLayout()
        fused_row.setSpacing(2)
        
        # Spacer for checkbox column
        spacer = QLabel("")
        spacer.setFixedWidth(20)
        fused_row.addWidget(spacer)
        
        # Fused label
        fused_lbl = QLabel("Fused")
        fused_lbl.setStyleSheet("color: #00ffff; font-weight: bold; font-size: 10px;")
        fused_lbl.setFixedWidth(35)
        fused_lbl.setAlignment(Qt.AlignCenter)
        fused_row.addWidget(fused_lbl)
        
        # Fused values
        self._fused_labels = {}
        self._fused_values = {'gx': None, 'gy': None, 'gz': None, 'ax': None, 'ay': None, 'az': None}
        for axis in ['gx', 'gy', 'gz', 'ax', 'ay', 'az']:
            lbl = QLabel("---")
            lbl.setStyleSheet("""
                background-color: #002244;
                color: #00ffff;
                padding: 1px 3px;
                border: 1px solid #0088aa;
                font-family: monospace;
                font-size: 9px;
                font-weight: bold;
            """)
            lbl.setFixedWidth(55)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._fused_labels[axis] = lbl
            fused_row.addWidget(lbl)
        
        fused_row.addStretch()
        layout.addLayout(fused_row)
        
        layout.addStretch()
    
    def _on_selection_changed(self, state):
        """Handle checkbox state change"""
        selected = [i for i, cb in enumerate(self._checkboxes) if cb.isChecked()]
        self.imu_selection_changed.emit(selected)
        self.update_fused()
    
    def update_imu_data(self, imu_index: int, gx: float, gy: float, gz: float,
                         ax: float, ay: float, az: float):
        """Update data for a specific IMU"""
        if 0 <= imu_index < 4:
            # Store numeric values first (source-of-truth)
            self._imu_values[imu_index]['gx'] = gx
            self._imu_values[imu_index]['gy'] = gy
            self._imu_values[imu_index]['gz'] = gz
            self._imu_values[imu_index]['ax'] = ax
            self._imu_values[imu_index]['ay'] = ay
            self._imu_values[imu_index]['az'] = az

            labels = self._imu_labels[imu_index]
            labels['gx'].setText(f"{gx:+.3f}")
            labels['gy'].setText(f"{gy:+.3f}")
            labels['gz'].setText(f"{gz:+.3f}")
            labels['ax'].setText(f"{ax:+.2f}")
            labels['ay'].setText(f"{ay:+.2f}")
            labels['az'].setText(f"{az:+.2f}")

    def clear_imu_data(self, imu_index: int):
        """Clear a specific IMU's data (mark as unavailable)."""
        if 0 <= imu_index < 4:
            for axis in self._imu_values[imu_index]:
                self._imu_values[imu_index][axis] = None
            for axis, lbl in self._imu_labels[imu_index].items():
                lbl.setText("---")
    
    def update_fused(self):
        """Calculate and display fused IMU values from selected IMUs."""
        selected = [i for i, cb in enumerate(self._checkboxes) if cb.isChecked()]
        
        if not selected:
            for lbl in self._fused_labels.values():
                lbl.setText("---")
            for axis in self._fused_values:
                self._fused_values[axis] = None
            return
        
        # Average the selected IMUs (only those with valid numeric data)
        sums = {'gx': 0.0, 'gy': 0.0, 'gz': 0.0, 'ax': 0.0, 'ay': 0.0, 'az': 0.0}
        count = 0

        for i in selected:
            v = self._imu_values[i]
            if any(v[axis] is None for axis in sums):
                continue
            for axis in sums:
                sums[axis] += float(v[axis])
            count += 1

        if count <= 0:
            for lbl in self._fused_labels.values():
                lbl.setText("---")
            for axis in self._fused_values:
                self._fused_values[axis] = None
            return

        self._fused_values['gx'] = sums['gx'] / count
        self._fused_values['gy'] = sums['gy'] / count
        self._fused_values['gz'] = sums['gz'] / count
        self._fused_values['ax'] = sums['ax'] / count
        self._fused_values['ay'] = sums['ay'] / count
        self._fused_values['az'] = sums['az'] / count

        self._fused_labels['gx'].setText(f"{self._fused_values['gx']:+.3f}")
        self._fused_labels['gy'].setText(f"{self._fused_values['gy']:+.3f}")
        self._fused_labels['gz'].setText(f"{self._fused_values['gz']:+.3f}")
        self._fused_labels['ax'].setText(f"{self._fused_values['ax']:+.2f}")
        self._fused_labels['ay'].setText(f"{self._fused_values['ay']:+.2f}")
        self._fused_labels['az'].setText(f"{self._fused_values['az']:+.2f}")
    
    def get_selected_imus(self) -> list:
        """Return list of selected IMU indices"""
        return [i for i, cb in enumerate(self._checkboxes) if cb.isChecked()]
    
    def clear(self):
        """Clear all IMU data"""
        for i in range(len(self._imu_labels)):
            self.clear_imu_data(i)
        for lbl in self._fused_labels.values():
            lbl.setText("---")
        for axis in self._fused_values:
            self._fused_values[axis] = None


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
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(5)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Top row - sensor data in horizontal layout
        top_row = QHBoxLayout()
        top_row.setSpacing(8)
        
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
        top_row.addLayout(left_col)
        
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
        top_row.addLayout(mid_col)
        
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
        top_row.addLayout(right_col)
        
        main_layout.addLayout(top_row, 1)  # Stretch factor 1
        
        # Bottom row - IMU Fusion panel (spans full width)
        self._imu_fusion = ImuFusionPanel()
        main_layout.addWidget(self._imu_fusion)
    
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
        
        # Update IMU Fusion panel with data from all 4 IMUs
        # Currently firmware sends fused data, so we display same values for all IMUs
        # When firmware is updated to send individual IMU data, this will show real values
        # For now, show the fused value for IMU #0, and simulate slight variations for others
        import random
        for i in range(4):
            # Add small simulated variation per IMU for demo purposes
            # In real implementation, each IMU would have its own data from firmware
            noise_scale = 0.001 if i > 0 else 0.0  # No noise for IMU #0
            gx = data.gyro_x + random.uniform(-noise_scale, noise_scale)
            gy = data.gyro_y + random.uniform(-noise_scale, noise_scale)
            gz = data.gyro_z + random.uniform(-noise_scale, noise_scale)
            ax = data.accel_x + random.uniform(-noise_scale * 10, noise_scale * 10)
            ay = data.accel_y + random.uniform(-noise_scale * 10, noise_scale * 10)
            az = data.accel_z + random.uniform(-noise_scale * 10, noise_scale * 10)
            self._imu_fusion.update_imu_data(i, gx, gy, gz, ax, ay, az)
        
        # Trigger fusion calculation
        self._imu_fusion._update_fused()
    
    def update_rate(self, rate_hz: float):
        """Update displayed rate"""
        self._rate.set_value(rate_hz)
        self._rate.set_color('green' if rate_hz >= 90 else 'yellow' if rate_hz >= 50 else 'red')
    
    def clear(self):
        """Clear all displays to default values"""
        # IMU
        for comp in ['X', 'Y', 'Z']:
            self._gyro[comp].set_value(0.0)
            self._accel[comp].set_value(0.0)
        self._imu_temp.set_value(0.0)
        
        # Magnetometer
        for comp in ['X', 'Y', 'Z']:
            self._mag[comp].set_value(0.0)
        self._mag_heading.set_value(0.0)
        
        # Barometer
        self._pressure.set_value(0.0)
        self._baro_alt.set_value(0.0)
        
        # Attitude
        self._euler['R'].set_value(0.0)
        self._euler['P'].set_value(0.0)
        self._euler['Y'].set_value(0.0)
        self._quat['W'].set_value(1.0)
        self._quat['X'].set_value(0.0)
        self._quat['Y'].set_value(0.0)
        self._quat['Z'].set_value(0.0)
        self._rates['R'].set_value(0.0)
        self._rates['P'].set_value(0.0)
        self._rates['Y'].set_value(0.0)
        
        # GPS
        self._lat.set_value(0.0)
        self._lon.set_value(0.0)
        self._alt_msl.set_value(0.0)
        self._gnd_speed.set_value(0.0)
        self._gps_heading.set_value(0.0)
        
        # GPS Status
        self._fix_type.set_value(0)
        self._sats.set_value(0)
        self._hdop.set_value(0.0)
        self._vdop.set_value(0.0)
        
        # System
        self._health.set_value(0)
        self._cpu.set_value(0.0)
        self._battery.set_value(0.0)
        self._loop_cnt.set_value(0)
        
        # Timing
        self._timestamp.set_value(0.0)
        self._sequence.set_value(0)
        self._rate.set_value(0)
        
        # IMU Fusion panel
        self._imu_fusion.clear()
