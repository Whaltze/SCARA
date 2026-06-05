import re
import time

from PySide6.QtCore import Slot
from PySide6.QtWidgets import QHBoxLayout, QMainWindow, QPushButton, QVBoxLayout, QWidget

try:
    import pyqtgraph as pg
except ImportError:
    pg = None
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure


_FEEDBACK_RE = re.compile(r"X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)")


class MonitorWindow(QMainWindow):
    def __init__(self, kinematics_engine):
        super().__init__()
        self.setWindowTitle("\u7535\u673a\u52a8\u529b\u5b66\u5b9e\u65f6\u76d1\u63a7 (\u771f\u5b9e\u53cd\u9988\u89e3\u7b97)")
        self.resize(1000, 800)

        self.kin = kinematics_engine
        self.is_running = True
        self.window_seconds = 10.0

        self.time_history = []
        self.v1_data, self.v2_data = [], []
        self.a1_data, self.a2_data = [], []

        self.last_pos = None
        self.last_theta = None
        self.last_velocity = None
        self.last_time = None
        self.start_timestamp = time.perf_counter()

        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        btn_layout = QHBoxLayout()
        self.btn_pause = QPushButton("\u5f00\u59cb/\u6682\u505c\u663e\u793a")
        self.btn_clear = QPushButton("\u6e05\u7a7a\u56fe\u50cf")
        btn_layout.addWidget(self.btn_pause)
        btn_layout.addWidget(self.btn_clear)
        layout.addLayout(btn_layout)

        if pg is not None:
            self.view = pg.GraphicsLayoutWidget()
            layout.addWidget(self.view)

            p1_pen = pg.mkPen(color="#00FF00", width=2)
            p2_pen = pg.mkPen(color="#00FFFF", width=2)

            self.p_v1 = self.view.addPlot(title="\u7535\u673a 1 \u771f\u5b9e\u53cd\u9988\u901f\u5ea6 (deg/s)")
            self.cv1 = self.p_v1.plot(pen=p1_pen)
            self.p_v2 = self.view.addPlot(title="\u7535\u673a 2 \u771f\u5b9e\u53cd\u9988\u901f\u5ea6 (deg/s)")
            self.cv2 = self.p_v2.plot(pen=p2_pen)
            self.view.nextRow()
            self.p_a1 = self.view.addPlot(title="\u7535\u673a 1 \u771f\u5b9e\u53cd\u9988\u52a0\u901f\u5ea6 (deg/s\u00b2)")
            self.ca1 = self.p_a1.plot(pen=p1_pen)
            self.p_a2 = self.view.addPlot(title="\u7535\u673a 2 \u771f\u5b9e\u53cd\u9988\u52a0\u901f\u5ea6 (deg/s\u00b2)")
            self.ca2 = self.p_a2.plot(pen=p2_pen)
            for plot in (self.p_v1, self.p_v2, self.p_a1, self.p_a2):
                plot.showGrid(x=True, y=True, alpha=0.2)
        else:
            self.fig = Figure(figsize=(8, 6))
            self.canvas = FigureCanvas(self.fig)
            layout.addWidget(self.canvas)
            self.ax_v1 = self.fig.add_subplot(221)
            self.ax_v2 = self.fig.add_subplot(222)
            self.ax_a1 = self.fig.add_subplot(223)
            self.ax_a2 = self.fig.add_subplot(224)
            self.cv1, = self.ax_v1.plot([], [], color="#00AA00", lw=1.2)
            self.cv2, = self.ax_v2.plot([], [], color="#0088AA", lw=1.2)
            self.ca1, = self.ax_a1.plot([], [], color="#00AA00", lw=1.2)
            self.ca2, = self.ax_a2.plot([], [], color="#0088AA", lw=1.2)
            for ax, title in (
                (self.ax_v1, "\u7535\u673a 1 \u771f\u5b9e\u53cd\u9988\u901f\u5ea6 (deg/s)"),
                (self.ax_v2, "\u7535\u673a 2 \u771f\u5b9e\u53cd\u9988\u901f\u5ea6 (deg/s)"),
                (self.ax_a1, "\u7535\u673a 1 \u771f\u5b9e\u53cd\u9988\u52a0\u901f\u5ea6 (deg/s\u00b2)"),
                (self.ax_a2, "\u7535\u673a 2 \u771f\u5b9e\u53cd\u9988\u52a0\u901f\u5ea6 (deg/s\u00b2)"),
            ):
                ax.set_title(title)
                ax.grid(True, alpha=0.2)

        self.btn_pause.clicked.connect(lambda: setattr(self, "is_running", not self.is_running))
        self.btn_clear.clicked.connect(self.clear_all)

    def clear_all(self):
        self.time_history.clear()
        self.v1_data.clear()
        self.v2_data.clear()
        self.a1_data.clear()
        self.a2_data.clear()
        self.last_pos = None
        self.last_theta = None
        self.last_velocity = None
        self.last_time = None
        self.start_timestamp = time.perf_counter()
        self.update_plots()

    def process_tcp_point(self, x, y, dt):
        """??/???????????? dt ?? TCP ???????????

        ? process_new_data??????????????
        ???????????? dt??? ? ??????????
        ?????????????????????
        """
        if not self.is_running:
            return

        curr_x, curr_y = x, y
        if self.last_pos == (curr_x, curr_y):
            return

        theta = self.kin.inverse(curr_x, curr_y)
        if theta[0] is None:
            return

        if self.last_theta is not None and dt > 1e-9:
            v1 = (theta[0] - self.last_theta[0]) / dt
            v2 = (theta[1] - self.last_theta[1]) / dt

            if self.last_velocity is not None:
                a1 = (v1 - self.last_velocity[0]) / dt
                a2 = (v2 - self.last_velocity[1]) / dt
                t_rel = self.time_history[-1] + dt if self.time_history else 0.0
                self.time_history.append(t_rel)
                self.v1_data.append(v1)
                self.v2_data.append(v2)
                self.a1_data.append(a1)
                self.a2_data.append(a2)
                self._trim_window(t_rel)
                self.update_plots()

            self.last_velocity = (v1, v2)

        self.last_pos = (curr_x, curr_y)
        self.last_theta = (theta[0], theta[1])

    @Slot(str)
    def process_new_data(self, raw_str):
        if not self.is_running:
            return

        match = _FEEDBACK_RE.search(raw_str)
        if not match:
            return

        curr_x, curr_y = float(match.group(1)), float(match.group(2))
        if self.last_pos == (curr_x, curr_y):
            return

        theta = self.kin.inverse(curr_x, curr_y)
        if theta[0] is None:
            return

        now = time.perf_counter()
        if self.last_theta is not None and self.last_time is not None:
            dt = now - self.last_time
            if dt <= 0.0:
                return

            v1 = (theta[0] - self.last_theta[0]) / dt
            v2 = (theta[1] - self.last_theta[1]) / dt

            if self.last_velocity is not None:
                a1 = (v1 - self.last_velocity[0]) / dt
                a2 = (v2 - self.last_velocity[1]) / dt
                t_rel = now - self.start_timestamp
                self.time_history.append(t_rel)
                self.v1_data.append(v1)
                self.v2_data.append(v2)
                self.a1_data.append(a1)
                self.a2_data.append(a2)
                self._trim_window(t_rel)
                self.update_plots()

            self.last_velocity = (v1, v2)

        self.last_pos = (curr_x, curr_y)
        self.last_theta = (theta[0], theta[1])
        self.last_time = now

    def _trim_window(self, latest_time):
        min_time = max(0.0, latest_time - self.window_seconds)
        while self.time_history and self.time_history[0] < min_time:
            self.time_history.pop(0)
            self.v1_data.pop(0)
            self.v2_data.pop(0)
            self.a1_data.pop(0)
            self.a2_data.pop(0)

    def update_plots(self):
        self._set_curve_data(self.cv1, self.time_history, self.v1_data)
        self._set_curve_data(self.cv2, self.time_history, self.v2_data)
        self._set_curve_data(self.ca1, self.time_history, self.a1_data)
        self._set_curve_data(self.ca2, self.time_history, self.a2_data)
        self._apply_time_window()

    def _apply_time_window(self):
        latest = self.time_history[-1] if self.time_history else 0.0
        if latest <= self.window_seconds:
            start, end = 0.0, self.window_seconds
        else:
            start, end = latest - self.window_seconds, latest

        if pg is not None:
            for plot in (self.p_v1, self.p_v2, self.p_a1, self.p_a2):
                plot.setXRange(start, end, padding=0)
        else:
            for ax in (self.ax_v1, self.ax_v2, self.ax_a1, self.ax_a2):
                ax.set_xlim(start, end)
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
            self.canvas.draw_idle()

    def _set_curve_data(self, curve, x_data, y_data):
        if pg is not None:
            curve.setData(x_data, y_data)
        else:
            curve.set_data(x_data, y_data)
