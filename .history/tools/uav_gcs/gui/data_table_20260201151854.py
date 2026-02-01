"""
UAV Ground Control Station - Data Table

Realtime data table hiển thị tất cả sensor values.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QBrush


class DataTable(QWidget):
    """
    Realtime data table showing all telemetry values.
    """
    
    # Table configuration: (label, fields, unit, format)
    ROWS = [
        ("Gyro", ["gyro_x", "gyro_y", "gyro_z"], "rad/s", ".4f"),
        ("Accel", ["accel_x", "accel_y", "accel_z"], "m/s²", ".3f"),
        ("Mag", ["mag_x", "mag_y", "mag_z"], "Gauss", ".4f"),
        ("Euler", ["roll_deg", "pitch_deg", "yaw_deg"], "°", ".2f"),
        ("Quaternion", ["qw", "qx", "qy", "qz"], "", ".4f"),
        ("Baro", ["pressure", "baro_alt", None], "Pa / m", ".2f"),
        ("GPS Pos", ["latitude", "longitude", "altitude_msl"], "° / m", ".6f"),
        ("GPS Info", ["ground_speed", "heading", None], "m/s / °", ".2f"),
        ("IMU Temp", ["imu_temp", None, None], "°C", ".1f"),
    ]
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup widget layout"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create table
        self._table = QTableWidget()
        self._table.setRowCount(len(self.ROWS))
        self._table.setColumnCount(6)  # Label, X, Y, Z, Unit, Status
        
        # Headers
        self._table.setHorizontalHeaderLabels(
            ["Sensor", "X / Value 1", "Y / Value 2", "Z / Value 3", "Unit", "Status"]
        )
        
        # Configure
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.setSelectionMode(QAbstractItemView.NoSelection)
        self._table.setAlternatingRowColors(True)
        
        # Set column widths
        header = self._table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
        
        # Initialize rows
        for row, (label, _, unit, _) in enumerate(self.ROWS):
            # Label
            item = QTableWidgetItem(label)
            item.setTextAlignment(Qt.AlignCenter)
            self._table.setItem(row, 0, item)
            
            # Values (X, Y, Z) - empty initially
            for col in range(1, 4):
                item = QTableWidgetItem("--")
                item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                item.setFont(self._get_mono_font())
                self._table.setItem(row, col, item)
            
            # Unit
            item = QTableWidgetItem(unit)
            item.setTextAlignment(Qt.AlignCenter)
            self._table.setItem(row, 4, item)
            
            # Status indicator
            item = QTableWidgetItem("●")
            item.setTextAlignment(Qt.AlignCenter)
            item.setForeground(QBrush(QColor("#404040")))
            self._table.setItem(row, 5, item)
        
        layout.addWidget(self._table)
    
    def _get_mono_font(self):
        """Get monospace font"""
        from PySide6.QtGui import QFont
        font = QFont("Monospace")
        font.setStyleHint(QFont.TypeWriter)
        return font
    
    def update_data(self, data):
        """
        Update table with new telemetry data.
        
        Args:
            data: TelemetryData object
        """
        for row, (_, fields, _, fmt) in enumerate(self.ROWS):
            for col, field in enumerate(fields):
                if field is None:
                    continue
                
                # Get value
                value = getattr(data, field, None)
                if value is not None:
                    text = f"{value:{fmt}}"
                else:
                    text = "--"
                
                self._table.item(row, col + 1).setText(text)
            
            # Update status indicator (green = data received)
            status_item = self._table.item(row, 5)
            status_item.setForeground(QBrush(QColor("#00ff00")))
    
    def clear(self):
        """Clear all data"""
        for row in range(len(self.ROWS)):
            for col in range(1, 4):
                self._table.item(row, col).setText("--")
            
            status_item = self._table.item(row, 5)
            status_item.setForeground(QBrush(QColor("#404040")))
