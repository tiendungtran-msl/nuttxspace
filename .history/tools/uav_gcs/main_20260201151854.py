"""
UAV Ground Control Station - Main Entry Point

Usage:
    python -m uav_gcs.main
    
    or
    
    python run_gcs.py
"""

import sys

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

from .gui import MainWindow


def main():
    """Main entry point"""
    # High DPI support
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    
    app = QApplication(sys.argv)
    app.setApplicationName("UAV Ground Control Station")
    app.setOrganizationName("UAV Avionics")
    
    # Dark theme
    app.setStyle("Fusion")
    
    # Dark palette
    from PySide6.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(30, 30, 30))
    palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
    palette.setColor(QPalette.Base, QColor(45, 45, 45))
    palette.setColor(QPalette.AlternateBase, QColor(55, 55, 55))
    palette.setColor(QPalette.ToolTipBase, QColor(30, 30, 30))
    palette.setColor(QPalette.ToolTipText, QColor(220, 220, 220))
    palette.setColor(QPalette.Text, QColor(220, 220, 220))
    palette.setColor(QPalette.Button, QColor(50, 50, 50))
    palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
    palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    palette.setColor(QPalette.Link, QColor(0, 120, 215))
    palette.setColor(QPalette.Highlight, QColor(0, 120, 215))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
