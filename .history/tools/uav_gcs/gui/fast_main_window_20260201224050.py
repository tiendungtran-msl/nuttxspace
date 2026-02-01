"""
UAV Ground Control Station - Fast Main Window

Optimized for high-speed data display like STM32 IDE debug.
- No heavy plots (PyQtGraph removed)
- Simple 3D view at lower rate
- Fast text updates for sensor data
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QStatusBar, QLabel, QMenuBar,
    QMessageBox, QFrame
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction

from .control_panel import ControlPanel
from .attitude_view import AttitudeViewer
from .sensor_display import FastSensorDisplay

from ..protocol import SerialReceiver, TelemetryData, TelemetrySimulator
from ..data import DataManager, DataLogger


class FastMainWindow(QMainWindow):
    """
    Optimized GCS window for fast sensor display.
    
    Layout:
    ┌─────────────────────────────────────────────────┐
    │ Menu Bar                                         │
    ├────────────┬────────────────────────────────────┤
    │            │         3D Attitude (compact)      │
    │  Control   ├────────────────────────────────────┤
    │   Panel    │                                    │
    │            │       Fast Sensor Display          │
    │            │   (All sensor data, text-based)    │
    │            │                                    │
    ├────────────┴────────────────────────────────────┤
    │ Status Bar                        Rate: 100 Hz  │
    └─────────────────────────────────────────────────┘
    """
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("UAV GCS - Fast Debug View")
        self.setMinimumSize(1000, 600)
        
        # Core components
        self._receiver = SerialReceiver(self)
        self._simulator = TelemetrySimulator(self)
        self._data_manager = DataManager(buffer_size=10000)  # Smaller buffer
        self._logger = DataLogger()
        
        # Rate tracking
        self._packet_count = 0
        self._last_rate_time = 0
        self._current_rate = 0.0
        
        self._setup_ui()
        self._setup_connections()
        self._setup_timers()
    
    def _setup_ui(self):
        """Setup optimized UI"""
        central = QWidget()
        self.setCentralWidget(central)
        
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        # Left panel - controls (narrower)
        self._control_panel = ControlPanel()
        self._control_panel.setFixedWidth(220)
        main_layout.addWidget(self._control_panel)
        
        # Right side - vertical splitter
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(5)
        
        # 3D Attitude view (compact height)
        self._attitude_view = AttitudeViewer()
        self._attitude_view.setFixedHeight(250)  # Compact
        right_layout.addWidget(self._attitude_view)
        
        # Separator line
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("background-color: #444;")
        right_layout.addWidget(line)
        
        # Fast sensor display (main area)
        self._sensor_display = FastSensorDisplay()
        right_layout.addWidget(self._sensor_display, 1)  # Expand
        
        main_layout.addWidget(right_widget, 1)
        
        # Menu bar (minimal)
        self._setup_menu()
        
        # Status bar
        self._setup_status_bar()
    
    def _setup_menu(self):
        """Minimal menu"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("File")
        
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # View menu
        view_menu = menubar.addMenu("View")
        
        reset_3d = QAction("Reset 3D View", self)
        reset_3d.triggered.connect(self._attitude_view.reset_view)
        view_menu.addAction(reset_3d)
        
        clear_data = QAction("Clear Data", self)
        clear_data.triggered.connect(self._clear_data)
        view_menu.addAction(clear_data)
        
        # Help menu
        help_menu = menubar.addMenu("Help")
        
        about_action = QAction("About", self)
        about_action.triggered.connect(self._show_about)
        help_menu.addAction(about_action)
    
    def _setup_status_bar(self):
        """Setup status bar with key metrics"""
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        
        # RX activity indicator (blinks on packet received)
        self._rx_indicator = QLabel("●")
        self._rx_indicator.setStyleSheet("color: #404040; font-size: 16px; padding: 0 5px;")
        self._rx_blink_state = False
        self._rx_blink_count = 0
        
        # Permanent widgets (right side)
        self._status_rate = QLabel("Rate: --- Hz")
        self._status_rate.setStyleSheet("color: #00ff00; font-family: monospace; padding: 0 10px;")
        
        self._status_packets = QLabel("Pkts: 0")
        self._status_packets.setStyleSheet("color: #aaa; font-family: monospace; padding: 0 10px;")
        
        self._status_drops = QLabel("Drops: 0")
        self._status_drops.setStyleSheet("color: #aaa; font-family: monospace; padding: 0 10px;")
        
        self._status_bar.addPermanentWidget(self._rx_indicator)
        self._status_bar.addPermanentWidget(self._status_packets)
        self._status_bar.addPermanentWidget(self._status_drops)
        self._status_bar.addPermanentWidget(self._status_rate)
        
        self._status_bar.showMessage("Ready - Click 'Demo Mode' to test")
    
    def _setup_connections(self):
        """Setup signal connections"""
        # Control panel
        self._control_panel.connect_requested.connect(self._on_connect)
        self._control_panel.disconnect_requested.connect(self._on_disconnect)
        self._control_panel.record_requested.connect(self._on_record)
        self._control_panel.demo_requested.connect(self._on_demo)
        self._control_panel.reset_data_requested.connect(self._on_reset_data)
        
        # Serial receiver
        self._receiver.packet_received.connect(self._on_packet_received)
        self._receiver.connection_changed.connect(self._on_connection_changed)
        self._receiver.error_occurred.connect(self._on_error)
        
        # Simulator
        self._simulator.packet_received.connect(self._on_packet_received)
        self._simulator.connection_changed.connect(self._on_connection_changed)
    
    def _setup_timers(self):
        """Setup update timers - optimized rates"""
        # Fast data display update (100 Hz for responsive feel)
        self._data_timer = QTimer(self)
        self._data_timer.timeout.connect(self._update_data_display)
        self._data_timer.start(10)  # 100 Hz
        
        # 3D view update (30 Hz is enough for smooth visual)
        self._3d_timer = QTimer(self)
        self._3d_timer.timeout.connect(self._update_3d_view)
        self._3d_timer.start(33)  # 30 Hz
        
        # Rate calculation (1 Hz)
        self._rate_timer = QTimer(self)
        self._rate_timer.timeout.connect(self._update_rate)
        self._rate_timer.start(1000)  # 1 Hz
    
    def _on_connect(self, port: str, baudrate: int):
        """Handle connect request"""
        self._status_bar.showMessage(f"Connecting to {port}...")
        success = self._receiver.connect(port, baudrate)
        
        if success:
            self._status_bar.showMessage(f"Connected to {port} @ {baudrate}")
        else:
            self._status_bar.showMessage("Connection failed")
    
    def _on_disconnect(self):
        """Handle disconnect request"""
        self._receiver.disconnect()
        self._status_bar.showMessage("Disconnected")
    
    def _on_demo(self, start: bool):
        """Handle demo mode request"""
        if start:
            self._simulator.start(rate_hz=100)
            self._status_bar.showMessage("🎬 Demo Mode @ 100 Hz")
        else:
            self._simulator.stop()
            # Reset all data and displays when demo stops
            self._clear_data()
            self._reset_indicators()
            self._status_bar.showMessage("Demo stopped - Data cleared")
    
    def _on_record(self, start: bool):
        """Handle record request"""
        if start:
            filename = self._logger.start_session("GCS Recording")
            self._data_manager.set_log_callback(self._logger.log)
            self._control_panel.set_recording(True, filename)
            self._status_bar.showMessage(f"Recording to {filename}")
        else:
            self._logger.stop_session()
            self._data_manager.set_log_callback(None)
            self._control_panel.set_recording(False)
            self._status_bar.showMessage("Recording stopped")
    
    def _on_packet_received(self, data: TelemetryData):
        """Handle new packet - just forward to data manager"""
        self._data_manager.on_packet_received(data)
        self._packet_count += 1
        
        # Blink RX indicator (toggle every few packets for visibility)
        self._rx_blink_count += 1
        if self._rx_blink_count >= 5:  # Blink every 5 packets
            self._rx_blink_count = 0
            self._rx_blink_state = not self._rx_blink_state
            color = "#00ff00" if self._rx_blink_state else "#004400"
            self._rx_indicator.setStyleSheet(f"color: {color}; font-size: 16px; padding: 0 5px;")
    
    def _on_connection_changed(self, connected: bool):
        """Handle connection state change"""
        self._control_panel.set_connected(connected)
        if not connected:
            self._status_bar.showMessage("Disconnected")
    
    def _on_error(self, message: str):
        """Handle error"""
        self._status_bar.showMessage(f"Error: {message}")
    
    def _update_data_display(self):
        """Update sensor data display (100 Hz)"""
        data = self._data_manager.get_latest()
        if data is None:
            return
        
        # Update fast sensor display
        self._sensor_display.update_data(data)
        
        # Update health in control panel
        self._control_panel.update_health(data)
    
    def _update_3d_view(self):
        """Update 3D attitude view (30 Hz)"""
        data = self._data_manager.get_latest()
        if data is None:
            return
        
        # Update 3D attitude
        self._attitude_view.update_attitude(
            data.qw, data.qx, data.qy, data.qz,
            data.roll, data.pitch, data.yaw
        )
    
    def _update_rate(self):
        """Calculate and display update rate (1 Hz)"""
        self._current_rate = self._packet_count
        self._packet_count = 0
        
        # Update displays
        self._sensor_display.update_rate(self._current_rate)
        
        color = "#00ff00" if self._current_rate >= 90 else "#ffff00" if self._current_rate >= 50 else "#ff4444"
        self._status_rate.setStyleSheet(f"color: {color}; font-family: monospace; padding: 0 10px;")
        self._status_rate.setText(f"Rate: {self._current_rate:.0f} Hz")
        
        # Update packet count
        stats = self._data_manager.get_stats()
        self._status_packets.setText(f"Pkts: {stats.packets_received}")
        self._status_drops.setText(f"Drops: {stats.packets_dropped}")
    
    def _clear_data(self):
        """Clear all data"""
        self._data_manager.clear()
        self._sensor_display.clear()
        self._packet_count = 0
        self._current_rate = 0.0
    
    def _reset_indicators(self):
        """Reset all status indicators to default state"""
        # Reset RX indicator
        self._rx_indicator.setStyleSheet("color: #404040; font-size: 16px; padding: 0 5px;")
        self._rx_blink_state = False
        self._rx_blink_count = 0
        
        # Reset status bar labels
        self._status_rate.setText("Rate: --- Hz")
        self._status_rate.setStyleSheet("color: #00ff00; font-family: monospace; padding: 0 10px;")
        self._status_packets.setText("Pkts: 0")
        self._status_drops.setText("Drops: 0")
    
    def _on_reset_data(self):
        """Handle reset data request from control panel"""
        self._clear_data()
        self._reset_indicators()
        self._status_bar.showMessage("Data cleared")
    
    def _show_about(self):
        """Show about dialog"""
        QMessageBox.about(
            self,
            "About UAV GCS",
            "UAV Ground Control Station - Fast Debug View\n\n"
            "Optimized for high-speed sensor data display.\n\n"
            "Features:\n"
            "- 100 Hz data update rate\n"
            "- Minimal latency display\n"
            "- 3D attitude visualization\n"
            "- Color-coded health status\n\n"
            "Version 2.0 - Performance Edition"
        )
    
    def closeEvent(self, event):
        """Handle window close"""
        # Stop all
        self._receiver.disconnect()
        self._simulator.stop()
        self._logger.stop_session()
        
        # Stop timers
        self._data_timer.stop()
        self._3d_timer.stop()
        self._rate_timer.stop()
        
        event.accept()
