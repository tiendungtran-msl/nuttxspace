"""
UAV Ground Control Station - Main Window

Main application window với complete layout.
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QStatusBar, QLabel, QMenuBar, QMenu,
    QMessageBox, QFileDialog
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction

from .control_panel import ControlPanel
from .attitude_view import AttitudeViewer
from .plot_panel import PlotPanel
from .data_table import DataTable

from ..protocol import SerialReceiver, TelemetryData
from ..data import DataManager, DataLogger


class MainWindow(QMainWindow):
    """
    Main GCS window.
    
    Layout:
    ┌─────────────────────────────────────────────────┐
    │ Menu Bar                                         │
    ├─────────────────────────────────────────────────┤
    │ Control Panel │           3D View               │
    │               │                                 │
    │               ├─────────────────────────────────┤
    │               │           Plots                 │
    │               │                                 │
    │               ├─────────────────────────────────┤
    │               │         Data Table              │
    ├─────────────────────────────────────────────────┤
    │ Status Bar                                       │
    └─────────────────────────────────────────────────┘
    """
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("UAV Ground Control Station")
        self.setMinimumSize(1200, 800)
        
        # Core components
        self._receiver = SerialReceiver(self)
        self._data_manager = DataManager(buffer_size=50000)
        self._logger = DataLogger()
        
        self._setup_ui()
        self._setup_connections()
        self._setup_timers()
    
    def _setup_ui(self):
        """Setup main window UI"""
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        # Left panel - controls
        self._control_panel = ControlPanel()
        main_layout.addWidget(self._control_panel)
        
        # Right side - splitter for 3D/plots/table
        right_splitter = QSplitter(Qt.Vertical)
        
        # 3D attitude view
        self._attitude_view = AttitudeViewer()
        right_splitter.addWidget(self._attitude_view)
        
        # Plot panel
        self._plot_panel = PlotPanel()
        right_splitter.addWidget(self._plot_panel)
        
        # Data table
        self._data_table = DataTable()
        right_splitter.addWidget(self._data_table)
        
        # Set splitter proportions
        right_splitter.setSizes([300, 400, 150])
        
        main_layout.addWidget(right_splitter, 1)
        
        # Menu bar
        self._setup_menu()
        
        # Status bar
        self._setup_status_bar()
    
    def _setup_menu(self):
        """Setup menu bar"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("File")
        
        open_action = QAction("Open Log...", self)
        open_action.triggered.connect(self._open_log)
        file_menu.addAction(open_action)
        
        file_menu.addSeparator()
        
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
        """Setup status bar"""
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        
        # Permanent widgets
        self._status_rx = QLabel("Rx: 0 KB")
        self._status_packets = QLabel("Packets: 0")
        self._status_drops = QLabel("Drops: 0")
        self._status_fps = QLabel("FPS: 0")
        
        for label in [self._status_rx, self._status_packets, 
                      self._status_drops, self._status_fps]:
            label.setStyleSheet("padding: 0 10px;")
            self._status_bar.addPermanentWidget(label)
        
        self._status_bar.showMessage("Ready")
    
    def _setup_connections(self):
        """Setup signal connections"""
        # Control panel
        self._control_panel.connect_requested.connect(self._on_connect)
        self._control_panel.disconnect_requested.connect(self._on_disconnect)
        self._control_panel.record_requested.connect(self._on_record)
        
        # Serial receiver
        self._receiver.packet_received.connect(self._on_packet_received)
        self._receiver.connection_changed.connect(self._on_connection_changed)
        self._receiver.error_occurred.connect(self._on_error)
        self._receiver.stats_updated.connect(self._on_stats_updated)
    
    def _setup_timers(self):
        """Setup GUI update timers"""
        # GUI update timer (60 Hz)
        self._gui_timer = QTimer(self)
        self._gui_timer.timeout.connect(self._update_gui)
        self._gui_timer.start(16)  # ~60 FPS
        
        # Plot update timer (30 Hz)
        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._update_plots)
        self._plot_timer.start(33)  # ~30 FPS
    
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
        """Handle new packet from receiver"""
        # Forward to data manager
        self._data_manager.on_packet_received(data)
    
    def _on_connection_changed(self, connected: bool):
        """Handle connection state change"""
        self._control_panel.set_connected(connected)
        
        if not connected:
            self._status_bar.showMessage("Disconnected")
    
    def _on_error(self, message: str):
        """Handle error from receiver"""
        self._status_bar.showMessage(f"Error: {message}")
    
    def _on_stats_updated(self, stats: dict):
        """Handle stats update from receiver"""
        self._control_panel.update_stats(stats)
        
        # Update status bar
        rx_kb = stats.get('bytes_received', 0) / 1024.0
        self._status_rx.setText(f"Rx: {rx_kb:.1f} KB")
        self._status_packets.setText(f"Packets: {stats.get('packets_decoded', 0)}")
        self._status_drops.setText(f"Drops: {stats.get('sequence_drops', 0)}")
    
    def _update_gui(self):
        """Update GUI with latest data (60 Hz)"""
        data = self._data_manager.get_latest()
        if data is None:
            return
        
        # Update 3D attitude
        self._attitude_view.update_attitude(
            data.qw, data.qx, data.qy, data.qz,
            data.roll, data.pitch, data.yaw
        )
        
        # Update data table
        self._data_table.update_data(data)
        
        # Update health panel
        self._control_panel.update_health(data)
        
        # Add to plots
        self._plot_panel.add_data(data)
    
    def _update_plots(self):
        """Refresh plots (30 Hz)"""
        self._plot_panel.refresh()
    
    def _clear_data(self):
        """Clear all data"""
        self._data_manager.clear()
        self._plot_panel.clear()
        self._data_table.clear()
    
    def _open_log(self):
        """Open log file for replay"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open Log File", "./logs", "SQLite Database (*.db)"
        )
        if filename:
            self._status_bar.showMessage(f"Log replay not implemented yet: {filename}")
    
    def _show_about(self):
        """Show about dialog"""
        QMessageBox.about(
            self,
            "About UAV GCS",
            "UAV Ground Control Station\n\n"
            "Realtime telemetry visualization tool.\n\n"
            "Features:\n"
            "- Binary packet protocol (128 bytes)\n"
            "- 3D attitude visualization\n"
            "- Realtime sensor plots\n"
            "- SQLite logging\n\n"
            "Version 1.0"
        )
    
    def closeEvent(self, event):
        """Handle window close"""
        # Stop receiver
        self._receiver.disconnect()
        
        # Stop logger
        self._logger.stop_session()
        
        # Stop timers
        self._gui_timer.stop()
        self._plot_timer.stop()
        
        event.accept()
