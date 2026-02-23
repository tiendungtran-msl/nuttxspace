"""
UAV Ground Control Station - Control Panel

Left panel với connection controls và status display.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QComboBox, QLabel, QProgressBar,
    QGridLayout, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QPalette

import serial.tools.list_ports

from ..protocol.packet import TELEM_PACKET_SIZE


class StatusIndicator(QFrame):
    """Circular status indicator (LED style)"""
    
    COLORS = {
        'off': '#404040',
        'green': '#00ff00',
        'yellow': '#ffff00',
        'red': '#ff0000',
        'blue': '#0088ff',
    }
    
    def __init__(self, size: int = 12, parent=None):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self._color = 'off'
        self._update_style()
    
    def set_color(self, color: str):
        """Set indicator color"""
        self._color = color
        self._update_style()
    
    def _update_style(self):
        """Update widget style"""
        color = self.COLORS.get(self._color, self.COLORS['off'])
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {color};
                border-radius: 6px;
                border: 1px solid #606060;
            }}
        """)


class ControlPanel(QWidget):
    """
    Left control panel.
    
    Contains:
    - Connection controls
    - Status indicators
    - System health display
    """
    
    # Signals
    connect_requested = Signal(str, int)  # port, baudrate
    disconnect_requested = Signal()
    record_requested = Signal(bool)       # start/stop
    demo_requested = Signal(bool)         # start/stop demo mode
    reset_data_requested = Signal()       # reset all data
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setFixedWidth(250)
        self._demo_mode = False
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup widget layout"""
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Connection group
        conn_group = QGroupBox("Connection")
        conn_layout = QVBoxLayout(conn_group)
        conn_layout.setSpacing(6)
        
        # Port selection row
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        port_row.addWidget(self._port_combo, 1)  # stretch factor 1
        conn_layout.addLayout(port_row)
        
        # Port buttons row (refresh + auto-detect)
        port_btn_row = QHBoxLayout()
        self._refresh_btn = QPushButton("⟳ Refresh")
        self._refresh_btn.setToolTip("Refresh port list")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        port_btn_row.addWidget(self._refresh_btn)
        self._auto_btn = QPushButton("🔍 Auto")
        self._auto_btn.setToolTip("Auto-detect telemetry port")
        self._auto_btn.clicked.connect(self._auto_detect_port)
        port_btn_row.addWidget(self._auto_btn)
        conn_layout.addLayout(port_btn_row)
        
        # Baudrate
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud:"))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(['57600', '115200', '230400', '460800', '921600'])
        self._baud_combo.setCurrentText('921600')
        self._baud_combo.currentTextChanged.connect(self._update_baud_warning)
        baud_layout.addWidget(self._baud_combo)
        baud_layout.addStretch()
        conn_layout.addLayout(baud_layout)

        # Baud adequacy warning (scientific, not spammy)
        self._baud_warning = QLabel("")
        self._baud_warning.setStyleSheet("color: #888; font-size: 9px;")
        self._baud_warning.setWordWrap(True)
        conn_layout.addWidget(self._baud_warning)
        
        # Connect button
        btn_layout = QHBoxLayout()
        self._connect_btn = QPushButton("▶ Connect")
        self._connect_btn.clicked.connect(self._on_connect_clicked)
        btn_layout.addWidget(self._connect_btn)
        conn_layout.addLayout(btn_layout)
        
        # Demo mode button
        self._demo_btn = QPushButton("🎬 Demo Mode")
        self._demo_btn.setStyleSheet("QPushButton { color: #00aaff; }")
        self._demo_btn.clicked.connect(self._on_demo_clicked)
        conn_layout.addWidget(self._demo_btn)
        
        # Reset data button
        self._reset_btn = QPushButton("🗑 Reset Data")
        self._reset_btn.setStyleSheet("QPushButton { color: #ff8800; }")
        self._reset_btn.clicked.connect(self._on_reset_clicked)
        conn_layout.addWidget(self._reset_btn)
        
        layout.addWidget(conn_group)
        
        # Status group
        status_group = QGroupBox("Connection Status")
        status_layout = QGridLayout(status_group)
        
        # Connected indicator
        status_layout.addWidget(QLabel("Status:"), 0, 0)
        self._conn_indicator = StatusIndicator()
        status_layout.addWidget(self._conn_indicator, 0, 1)
        self._conn_label = QLabel("Disconnected")
        status_layout.addWidget(self._conn_label, 0, 2)
        
        # Packet counter
        status_layout.addWidget(QLabel("Packets:"), 1, 0)
        self._packet_label = QLabel("0")
        self._packet_label.setStyleSheet("font-family: monospace;")
        status_layout.addWidget(self._packet_label, 1, 1, 1, 2)
        
        # Drop counter
        status_layout.addWidget(QLabel("Drops:"), 2, 0)
        self._drops_label = QLabel("0")
        self._drops_label.setStyleSheet("font-family: monospace;")
        status_layout.addWidget(self._drops_label, 2, 1, 1, 2)
        
        # Rate
        status_layout.addWidget(QLabel("Rate:"), 3, 0)
        self._rate_label = QLabel("0 Hz")
        self._rate_label.setStyleSheet("font-family: monospace;")
        status_layout.addWidget(self._rate_label, 3, 1, 1, 2)
        
        layout.addWidget(status_group)
        
        # System Health group
        health_group = QGroupBox("System Health")
        health_layout = QGridLayout(health_group)
        
        # IMU status
        health_layout.addWidget(QLabel("IMU:"), 0, 0)
        self._imu_indicator = StatusIndicator()
        health_layout.addWidget(self._imu_indicator, 0, 1)
        self._imu_label = QLabel("--")
        health_layout.addWidget(self._imu_label, 0, 2)
        
        # Baro status
        health_layout.addWidget(QLabel("Baro:"), 1, 0)
        self._baro_indicator = StatusIndicator()
        health_layout.addWidget(self._baro_indicator, 1, 1)
        self._baro_label = QLabel("--")
        health_layout.addWidget(self._baro_label, 1, 2)
        
        # Mag status
        health_layout.addWidget(QLabel("Mag:"), 2, 0)
        self._mag_indicator = StatusIndicator()
        health_layout.addWidget(self._mag_indicator, 2, 1)
        self._mag_label = QLabel("--")
        health_layout.addWidget(self._mag_label, 2, 2)
        
        # GPS status
        health_layout.addWidget(QLabel("GPS:"), 3, 0)
        self._gps_indicator = StatusIndicator()
        health_layout.addWidget(self._gps_indicator, 3, 1)
        self._gps_label = QLabel("--")
        health_layout.addWidget(self._gps_label, 3, 2)
        
        # EKF status
        health_layout.addWidget(QLabel("EKF:"), 4, 0)
        self._ekf_indicator = StatusIndicator()
        health_layout.addWidget(self._ekf_indicator, 4, 1)
        self._ekf_label = QLabel("--")
        health_layout.addWidget(self._ekf_label, 4, 2)
        
        layout.addWidget(health_group)
        
        # System info group
        info_group = QGroupBox("System Info")
        info_layout = QGridLayout(info_group)
        
        info_layout.addWidget(QLabel("CPU:"), 0, 0)
        self._cpu_label = QLabel("--")
        self._cpu_label.setStyleSheet("font-family: monospace;")
        info_layout.addWidget(self._cpu_label, 0, 1)
        
        info_layout.addWidget(QLabel("Battery:"), 1, 0)
        self._battery_label = QLabel("--")
        self._battery_label.setStyleSheet("font-family: monospace;")
        info_layout.addWidget(self._battery_label, 1, 1)
        
        info_layout.addWidget(QLabel("Uptime:"), 2, 0)
        self._uptime_label = QLabel("--")
        self._uptime_label.setStyleSheet("font-family: monospace;")
        info_layout.addWidget(self._uptime_label, 2, 1)
        
        layout.addWidget(info_group)
        
        # Recording group
        rec_group = QGroupBox("Recording")
        rec_layout = QVBoxLayout(rec_group)
        
        self._record_btn = QPushButton("⏺ Start Recording")
        self._record_btn.setCheckable(True)
        self._record_btn.clicked.connect(self._on_record_clicked)
        rec_layout.addWidget(self._record_btn)
        
        self._record_label = QLabel("Not recording")
        self._record_label.setStyleSheet("color: gray;")
        rec_layout.addWidget(self._record_label)
        
        layout.addWidget(rec_group)
        
        # Stretch at bottom
        layout.addStretch()
        
        # Initial port refresh
        self._refresh_ports()
        self._update_baud_warning(self._baud_combo.currentText())

    def _update_baud_warning(self, baud_text: str):
        """Warn if selected baud is insufficient for 212B@100Hz telemetry."""
        try:
            baud = int(baud_text)
        except Exception:
            self._baud_warning.setText("")
            return

        expected_rate_hz = 100
        required_bps = TELEM_PACKET_SIZE * expected_rate_hz * 10  # 8N1 ~= 10 bits/byte
        required_baud = int(required_bps)
        max_hz = baud / (TELEM_PACKET_SIZE * 10)

        if baud < required_baud:
            self._baud_warning.setStyleSheet("color: #ff6666; font-size: 9px;")
            self._baud_warning.setText(
                f"Warning: {baud} too low for {TELEM_PACKET_SIZE}B@{expected_rate_hz}Hz. "
                f"Max ~{max_hz:.0f}Hz (need ≥{required_baud})."
            )
        else:
            self._baud_warning.setStyleSheet("color: #888; font-size: 9px;")
            self._baud_warning.setText(
                f"Info: {TELEM_PACKET_SIZE}B@{expected_rate_hz}Hz needs ≥{required_baud} baud."
            )
    
    def _refresh_ports(self):
        """Refresh available serial ports"""
        self._port_combo.clear()
        
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self._port_combo.addItem(port.device)
        
        # Add common defaults if empty
        if self._port_combo.count() == 0:
            self._port_combo.addItems(['/dev/ttyUSB0', '/dev/ttyACM0', 'COM3'])
        
        # Reset to recommended baud for 212B@100Hz
        self._baud_combo.setCurrentText('921600')
    
    def _auto_detect_port(self):
        """Auto-detect port with telemetry traffic by checking for data activity"""
        import serial
        import time
        
        self._auto_btn.setEnabled(False)
        self._auto_btn.setText("...")
        
        # Get all available ports
        ports = [self._port_combo.itemText(i) for i in range(self._port_combo.count())]
        
        best_port = None
        best_baud = 921600
        max_bytes = 0
        
        # Test each port at multiple bauds
        for port_name in ports:
            for test_baud in [921600, 460800, 230400, 115200]:
                try:
                    ser = serial.Serial(port_name, test_baud, timeout=0.1)
                    time.sleep(0.1)  # Let buffer fill
                    data = ser.read(1024)
                    ser.close()
                    
                    if len(data) > max_bytes:
                        max_bytes = len(data)
                        best_port = port_name
                        best_baud = test_baud
                        
                        # If we see magic bytes, this is very likely the right port
                        if b'\xAA\x55' in data:
                            break
                except Exception:
                    pass
            
            if best_port and b'\xAA\x55' in (data if 'data' in locals() else b''):
                break  # Found magic, stop searching
        
        self._auto_btn.setEnabled(True)
        self._auto_btn.setText("🔍")
        
        if best_port:
            # Set detected port and baud
            idx = self._port_combo.findText(best_port)
            if idx >= 0:
                self._port_combo.setCurrentIndex(idx)
            self._baud_combo.setCurrentText(str(best_baud))
            
            msg = f"Detected: {best_port} @ {best_baud} ({max_bytes}B in 100ms)"
            print(f"[AutoDetect] {msg}")
        else:
            print("[AutoDetect] No active telemetry port found")
    
    def _on_connect_clicked(self):
        """Handle connect button click"""
        if self._connect_btn.text().startswith("▶"):
            # Connect
            port = self._port_combo.currentText()
            baud = int(self._baud_combo.currentText())
            self.connect_requested.emit(port, baud)
        else:
            # Disconnect
            self.disconnect_requested.emit()
    
    def _on_record_clicked(self, checked: bool):
        """Handle record button click"""
        self.record_requested.emit(checked)
    
    def _on_demo_clicked(self):
        """Handle demo mode button click"""
        self._demo_mode = not self._demo_mode
        
        if self._demo_mode:
            self._demo_btn.setText("■ Stop Demo")
            self._demo_btn.setStyleSheet("QPushButton { color: #ff6600; }")
            self._connect_btn.setEnabled(False)
        else:
            self._demo_btn.setText("🎬 Demo Mode")
            self._demo_btn.setStyleSheet("QPushButton { color: #00aaff; }")
            self._connect_btn.setEnabled(True)
        
        self.demo_requested.emit(self._demo_mode)
    
    def _on_reset_clicked(self):
        """Handle reset data button click"""
        self.reset_data_requested.emit()
    
    def set_connected(self, connected: bool):
        """Update connection state"""
        if connected:
            self._connect_btn.setText("■ Disconnect")
            self._conn_indicator.set_color('green')
            self._conn_label.setText("Connected")
        else:
            self._connect_btn.setText("▶ Connect")
            self._conn_indicator.set_color('off')
            self._conn_label.setText("Disconnected")
    
    def update_stats(self, stats: dict):
        """Update connection statistics"""
        self._packet_label.setText(f"{stats.get('packets_decoded', 0):,}")
        self._drops_label.setText(f"{stats.get('sequence_drops', 0)}")
        
        rate = stats.get('packets_per_second', 0)
        self._rate_label.setText(f"{rate:.1f} Hz")
    
    def update_health(self, data):
        """Update system health from telemetry data"""
        # Sensor flags
        flags = data.sensor_flags
        
        # IMU
        imu_ok = (flags & 0x01) != 0
        self._imu_indicator.set_color('green' if imu_ok else 'red')
        healthy_count = bin(data.healthy_imus).count('1')
        self._imu_label.setText(f"{healthy_count}/4 OK")
        
        # Baro
        baro_ok = (flags & 0x02) != 0
        self._baro_indicator.set_color('green' if baro_ok else 'red')
        self._baro_label.setText("OK" if baro_ok else "FAIL")
        
        # Mag
        mag_ok = (flags & 0x04) != 0
        self._mag_indicator.set_color('green' if mag_ok else 'red')
        self._mag_label.setText("OK" if mag_ok else "FAIL")
        
        # GPS
        gps_ok = (flags & 0x08) != 0
        if gps_ok:
            fix_text = {0: 'No fix', 2: '2D', 3: '3D'}.get(data.fix_type, '?')
            self._gps_indicator.set_color('green' if data.fix_type >= 3 else 'yellow')
            self._gps_label.setText(f"{fix_text} ({data.satellites})")
        else:
            self._gps_indicator.set_color('red')
            self._gps_label.setText("FAIL")
        
        # EKF
        ekf_ok = (flags & 0x10) != 0
        self._ekf_indicator.set_color('green' if ekf_ok else 'yellow')
        self._ekf_label.setText("Converged" if ekf_ok else "Init")
        
        # System info
        self._cpu_label.setText(f"{data.cpu_load:.1f}%")
        self._battery_label.setText(f"{data.battery_mv / 1000.0:.2f}V")
        
        # Uptime from timestamp
        uptime_sec = data.timestamp_us / 1_000_000
        hours = int(uptime_sec // 3600)
        mins = int((uptime_sec % 3600) // 60)
        secs = int(uptime_sec % 60)
        self._uptime_label.setText(f"{hours:02d}:{mins:02d}:{secs:02d}")
    
    def set_recording(self, recording: bool, filename: str = ""):
        """Update recording state"""
        if recording:
            self._record_btn.setText("⏹ Stop Recording")
            self._record_label.setText(f"Recording: {filename}")
            self._record_label.setStyleSheet("color: red;")
        else:
            self._record_btn.setText("⏺ Start Recording")
            self._record_btn.setChecked(False)
            self._record_label.setText("Not recording")
            self._record_label.setStyleSheet("color: gray;")
    
    def reset_display(self):
        """Reset all display fields to default values"""
        # Connection stats
        self._packet_label.setText("0")
        self._drops_label.setText("0")
        self._rate_label.setText("0 Hz")
        
        # Health indicators
        self._imu_indicator.set_color('off')
        self._imu_label.setText("--")
        self._baro_indicator.set_color('off')
        self._baro_label.setText("--")
        self._mag_indicator.set_color('off')
        self._mag_label.setText("--")
        self._gps_indicator.set_color('off')
        self._gps_label.setText("--")
        self._ekf_indicator.set_color('off')
        self._ekf_label.setText("--")
        
        # System info
        self._cpu_label.setText("--")
        self._battery_label.setText("--")
        self._uptime_label.setText("--")
