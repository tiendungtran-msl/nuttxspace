#!/usr/bin/env python3
"""
UAV Ground Control Station - Fast Debug Mode

Optimized for high-speed sensor data display.
No heavy plots, just fast text updates.

Usage:
    python3 run_gcs_fast.py
"""

import sys

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt
from PySide6.QtGui import QPalette, QColor

from uav_gcs.gui import FastMainWindow


def main():
    """Main entry point for fast GCS"""
    # High DPI support
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    
    app = QApplication(sys.argv)
    app.setApplicationName("UAV GCS - Fast Debug")
    app.setOrganizationName("UAV Avionics")
    
    # Dark theme
    app.setStyle("Fusion")
    
    # Dark palette optimized for data display
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(25, 25, 25))
    palette.setColor(QPalette.WindowText, QColor(200, 200, 200))
    palette.setColor(QPalette.Base, QColor(30, 30, 30))
    palette.setColor(QPalette.AlternateBase, QColor(40, 40, 40))
    palette.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
    palette.setColor(QPalette.ToolTipText, QColor(200, 200, 200))
    palette.setColor(QPalette.Text, QColor(200, 200, 200))
    palette.setColor(QPalette.Button, QColor(45, 45, 45))
    palette.setColor(QPalette.ButtonText, QColor(200, 200, 200))
    palette.setColor(QPalette.BrightText, QColor(255, 100, 100))
    palette.setColor(QPalette.Link, QColor(0, 150, 255))
    palette.setColor(QPalette.Highlight, QColor(0, 100, 180))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)
    
    # Create and show fast main window
    window = FastMainWindow()
    window.show()
    
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
