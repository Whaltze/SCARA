import sys

from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication

from .main_window import FiveBarSerialGUI
from ..V_monitor import MonitorWindow


def run_app() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("SCARA_UI")
    app.setOrganizationName("SCARA Course Design")
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei", 9))

    window = FiveBarSerialGUI()
    window.show()

    monitor = MonitorWindow(window.kinematics)
    monitor.show()
    window.velocity_monitor = monitor

    return app.exec()
