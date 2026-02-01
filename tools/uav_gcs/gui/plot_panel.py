"""
UAV Ground Control Station - Plot Panel

Realtime plotting của sensor data với PyQtGraph.
Hiệu suất cao, không block GUI.
"""

import numpy as np
from collections import deque
from typing import Dict, List, Tuple

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QComboBox, 
    QLabel, QSplitter, QGroupBox
)
from PySide6.QtCore import Qt

import pyqtgraph as pg


# Plot configurations
PLOT_CONFIGS = {
    'IMU - Gyro': {
        'signals': [
            ('Gyro X', 'gyro_x', 'r'),
            ('Gyro Y', 'gyro_y', 'g'),
            ('Gyro Z', 'gyro_z', 'b'),
        ],
        'ylabel': 'rad/s',
        'yrange': (-5, 5),
    },
    'IMU - Accel': {
        'signals': [
            ('Accel X', 'accel_x', 'r'),
            ('Accel Y', 'accel_y', 'g'),
            ('Accel Z', 'accel_z', 'b'),
        ],
        'ylabel': 'm/s²',
        'yrange': (-20, 20),
    },
    'Attitude - Euler': {
        'signals': [
            ('Roll', 'roll_deg', 'r'),
            ('Pitch', 'pitch_deg', 'g'),
            ('Yaw', 'yaw_deg', 'b'),
        ],
        'ylabel': 'degrees',
        'yrange': (-180, 180),
    },
    'Magnetometer': {
        'signals': [
            ('Mag X', 'mag_x', 'r'),
            ('Mag Y', 'mag_y', 'g'),
            ('Mag Z', 'mag_z', 'b'),
        ],
        'ylabel': 'Gauss',
        'yrange': (-1, 1),
    },
    'Barometer': {
        'signals': [
            ('Altitude', 'baro_alt', 'c'),
        ],
        'ylabel': 'meters',
        'yrange': (-10, 100),
    },
}


class RealtimePlot(pg.PlotWidget):
    """
    Single realtime plot with multiple traces.
    
    Optimized for high update rate.
    """
    
    def __init__(self, title: str, ylabel: str, yrange: Tuple[float, float],
                 signals: List[Tuple[str, str, str]], history_sec: float = 10.0,
                 parent=None):
        super().__init__(parent)
        
        self.setBackground('#1e1e1e')
        self.setTitle(title, color='w')
        self.setLabel('left', ylabel, color='w')
        self.setLabel('bottom', 'Time (s)', color='w')
        self.setYRange(*yrange)
        self.showGrid(x=True, y=True, alpha=0.3)
        
        # Enable antialiasing for better quality
        pg.setConfigOptions(antialias=True)
        
        self._signals = signals
        self._history_sec = history_sec
        self._sample_rate = 100  # Expected sample rate
        self._max_points = int(history_sec * self._sample_rate)
        
        # Data buffers
        self._time_buffer: deque = deque(maxlen=self._max_points)
        self._data_buffers: Dict[str, deque] = {}
        self._curves: Dict[str, pg.PlotDataItem] = {}
        
        # Create curves
        for name, attr, color in signals:
            self._data_buffers[attr] = deque(maxlen=self._max_points)
            curve = self.plot(pen=pg.mkPen(color, width=1), name=name)
            self._curves[attr] = curve
        
        # Add legend
        self.addLegend(offset=(10, 10))
        
        # Reference time (for relative x-axis)
        self._start_time = None
    
    def add_data(self, timestamp_us: int, values: Dict[str, float]):
        """
        Add new data point.
        
        Args:
            timestamp_us: MCU timestamp in microseconds
            values: Dict mapping attribute name to value
        """
        # Convert to seconds
        time_sec = timestamp_us / 1_000_000.0
        
        if self._start_time is None:
            self._start_time = time_sec
        
        relative_time = time_sec - self._start_time
        self._time_buffer.append(relative_time)
        
        # Add each signal value
        for _, attr, _ in self._signals:
            value = values.get(attr, 0.0)
            self._data_buffers[attr].append(value)
    
    def refresh(self):
        """Update plot curves with current data"""
        if len(self._time_buffer) < 2:
            return
        
        time_array = np.array(self._time_buffer)
        
        for _, attr, _ in self._signals:
            if len(self._data_buffers[attr]) == len(time_array):
                data_array = np.array(self._data_buffers[attr])
                self._curves[attr].setData(time_array, data_array)
        
        # Auto-scroll to show latest data
        if len(time_array) > 0:
            latest = time_array[-1]
            self.setXRange(max(0, latest - self._history_sec), latest + 0.5)
    
    def clear_data(self):
        """Clear all data"""
        self._time_buffer.clear()
        for buf in self._data_buffers.values():
            buf.clear()
        self._start_time = None


class PlotPanel(QWidget):
    """
    Panel containing multiple realtime plots.
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self._setup_ui()
        self._plots: List[RealtimePlot] = []
        self._create_plots()
    
    def _setup_ui(self):
        """Setup widget layout"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Controls
        controls = QHBoxLayout()
        
        controls.addWidget(QLabel("Time window:"))
        self._time_combo = QComboBox()
        self._time_combo.addItems(['5s', '10s', '30s', '60s'])
        self._time_combo.setCurrentIndex(1)
        self._time_combo.currentTextChanged.connect(self._on_time_changed)
        controls.addWidget(self._time_combo)
        
        controls.addStretch()
        
        layout.addLayout(controls)
        
        # Plot container with splitter
        self._splitter = QSplitter(Qt.Vertical)
        layout.addWidget(self._splitter)
    
    def _create_plots(self):
        """Create all plot widgets"""
        for config_name, config in PLOT_CONFIGS.items():
            plot = RealtimePlot(
                title=config_name,
                ylabel=config['ylabel'],
                yrange=config['yrange'],
                signals=config['signals'],
                history_sec=10.0
            )
            self._plots.append(plot)
            self._splitter.addWidget(plot)
        
        # Set equal sizes
        sizes = [100] * len(self._plots)
        self._splitter.setSizes(sizes)
    
    def add_data(self, data):
        """
        Add new telemetry data to plots.
        
        Args:
            data: TelemetryData object
        """
        # Extract values
        values = {
            'gyro_x': data.gyro_x,
            'gyro_y': data.gyro_y,
            'gyro_z': data.gyro_z,
            'accel_x': data.accel_x,
            'accel_y': data.accel_y,
            'accel_z': data.accel_z,
            'roll_deg': data.roll_deg,
            'pitch_deg': data.pitch_deg,
            'yaw_deg': data.yaw_deg,
            'mag_x': data.mag_x,
            'mag_y': data.mag_y,
            'mag_z': data.mag_z,
            'baro_alt': data.baro_alt,
        }
        
        for plot in self._plots:
            plot.add_data(data.timestamp_us, values)
    
    def refresh(self):
        """Refresh all plots"""
        for plot in self._plots:
            plot.refresh()
    
    def clear(self):
        """Clear all plot data"""
        for plot in self._plots:
            plot.clear_data()
    
    def _on_time_changed(self, text: str):
        """Handle time window change"""
        seconds = int(text.rstrip('s'))
        for plot in self._plots:
            plot._history_sec = seconds
