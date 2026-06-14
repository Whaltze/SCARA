import re
import time

from PySide6.QtCore import Qt, Slot
from PySide6.QtWidgets import QHBoxLayout, QMainWindow, QPushButton, QVBoxLayout, QWidget

try:
    import pyqtgraph as pg
except ImportError:
    pg = None
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure


_FEEDBACK_RE = re.compile(r"X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)")
_STATUS_AXIS_RE = re.compile(r"\bA([12]):(\d+),(\d+),([-+]?\d+),([-+]?\d+)")
_STAT_PPS_RE = re.compile(r"\bpps=([-+]?\d+),([-+]?\d+)")
_STAT_TICK_RE = re.compile(r"\bt=(\d+)")
_MIN_DERIVATIVE_DT = 0.02
_ACCEL_EMA_ALPHA = 0.25


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
        self.pos_time_history = []
        self.pos_v1_data, self.pos_v2_data = [], []
        self.pos_a1_data, self.pos_a2_data = [], []
        self.cmd_time_history = []
        self.cmd_v1_data, self.cmd_v2_data = [], []
        self.cmd_a1_data, self.cmd_a2_data = [], []

        self.last_pos = None
        self.last_theta = None
        self.last_velocity = None
        self.last_time = None
        self._pos_accel_ema = None
        self._pps_last_time = None
        self._pps_last_mcu_tick = None
        self._pps_last_velocity = None
        self._pps_accel_ema = None
        self._cmd_last_velocity = None
        self._cmd_accel_ema = None
        self._cmd_elapsed = 0.0
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
            p1_aux_pen = pg.mkPen(color="#88AA88", width=1, style=Qt.DashLine)
            p2_aux_pen = pg.mkPen(color="#88AACC", width=1, style=Qt.DashLine)
            p1_cmd_pen = pg.mkPen(color="#FFAA00", width=1, style=Qt.DotLine)
            p2_cmd_pen = pg.mkPen(color="#FF66FF", width=1, style=Qt.DotLine)

            self.p_v1 = self.view.addPlot(title="\u7535\u673a 1 PPS\u901f\u5ea6 / \u4f4d\u7f6e\u53cd\u63a8 (deg/s)")
            self.cv1 = self.p_v1.plot(pen=p1_pen)
            self.cv1_pos = self.p_v1.plot(pen=p1_aux_pen)
            self.cv1_cmd = self.p_v1.plot(pen=p1_cmd_pen)
            self.p_v2 = self.view.addPlot(title="\u7535\u673a 2 PPS\u901f\u5ea6 / \u4f4d\u7f6e\u53cd\u63a8 (deg/s)")
            self.cv2 = self.p_v2.plot(pen=p2_pen)
            self.cv2_pos = self.p_v2.plot(pen=p2_aux_pen)
            self.cv2_cmd = self.p_v2.plot(pen=p2_cmd_pen)
            self.view.nextRow()
            self.p_a1 = self.view.addPlot(title="\u7535\u673a 1 PPS\u52a0\u901f\u5ea6 / \u4f4d\u7f6e\u53cd\u63a8 (deg/s\u00b2)")
            self.ca1 = self.p_a1.plot(pen=p1_pen)
            self.ca1_pos = self.p_a1.plot(pen=p1_aux_pen)
            self.ca1_cmd = self.p_a1.plot(pen=p1_cmd_pen)
            self.p_a2 = self.view.addPlot(title="\u7535\u673a 2 PPS\u52a0\u901f\u5ea6 / \u4f4d\u7f6e\u53cd\u63a8 (deg/s\u00b2)")
            self.ca2 = self.p_a2.plot(pen=p2_pen)
            self.ca2_pos = self.p_a2.plot(pen=p2_aux_pen)
            self.ca2_cmd = self.p_a2.plot(pen=p2_cmd_pen)
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
            self.cv1_pos, = self.ax_v1.plot([], [], color="#88AA88", lw=0.9, ls="--")
            self.cv2_pos, = self.ax_v2.plot([], [], color="#88AACC", lw=0.9, ls="--")
            self.ca1_pos, = self.ax_a1.plot([], [], color="#88AA88", lw=0.9, ls="--")
            self.ca2_pos, = self.ax_a2.plot([], [], color="#88AACC", lw=0.9, ls="--")
            self.cv1_cmd, = self.ax_v1.plot([], [], color="#FFAA00", lw=0.9, ls=":")
            self.cv2_cmd, = self.ax_v2.plot([], [], color="#CC44CC", lw=0.9, ls=":")
            self.ca1_cmd, = self.ax_a1.plot([], [], color="#FFAA00", lw=0.9, ls=":")
            self.ca2_cmd, = self.ax_a2.plot([], [], color="#CC44CC", lw=0.9, ls=":")
            for ax, title in (
                (self.ax_v1, "\u7535\u673a 1 PPS\u901f\u5ea6 / \u4f4d\u7f6e\u53cd\u63a8 (deg/s)"),
                (self.ax_v2, "\u7535\u673a 2 PPS\u901f\u5ea6 / \u4f4d\u7f6e\u53cd\u63a8 (deg/s)"),
                (self.ax_a1, "\u7535\u673a 1 PPS\u52a0\u901f\u5ea6 / \u4f4d\u7f6e\u53cd\u63a8 (deg/s\u00b2)"),
                (self.ax_a2, "\u7535\u673a 2 PPS\u52a0\u901f\u5ea6 / \u4f4d\u7f6e\u53cd\u63a8 (deg/s\u00b2)"),
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
        self.pos_time_history.clear()
        self.pos_v1_data.clear()
        self.pos_v2_data.clear()
        self.pos_a1_data.clear()
        self.pos_a2_data.clear()
        self.cmd_time_history.clear()
        self.cmd_v1_data.clear()
        self.cmd_v2_data.clear()
        self.cmd_a1_data.clear()
        self.cmd_a2_data.clear()
        self.last_pos = None
        self.last_theta = None
        self.last_velocity = None
        self.last_time = None
        self._pos_accel_ema = None
        self._pps_last_time = None
        self._pps_last_mcu_tick = None
        self._pps_last_velocity = None
        self._pps_accel_ema = None
        self._cmd_last_velocity = None
        self._cmd_accel_ema = None
        self._cmd_elapsed = 0.0
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

        prev_time = self.last_time
        t_rel = self.pos_time_history[-1] + dt if self.pos_time_history else 0.0
        self._process_position_sample(float(x), float(y), max(float(dt), 0.001), t_rel)
        self.last_time = prev_time

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
            if dt < _MIN_DERIVATIVE_DT:
                return

            self._append_position_derivative(theta, dt, now - self.start_timestamp)

        self.last_pos = (curr_x, curr_y)
        self.last_theta = (theta[0], theta[1])
        self.last_time = now

    @Slot(str)
    def process_mcu_status(self, raw_str, ppr=6400):
        if not self.is_running:
            return

        pps = [None, None]
        for match in _STATUS_AXIS_RE.finditer(raw_str):
            axis = int(match.group(1)) - 1
            pps[axis] = int(match.group(4))

        if pps[0] is None or pps[1] is None:
            stat_match = _STAT_PPS_RE.search(raw_str)
            if stat_match:
                pps[0] = int(stat_match.group(1))
                pps[1] = int(stat_match.group(2))

        if pps[0] is None or pps[1] is None:
            return

        try:
            ppr_value = max(1, int(ppr))
        except (TypeError, ValueError):
            ppr_value = 6400

        now = time.perf_counter()
        dt = None
        tick_match = _STAT_TICK_RE.search(raw_str)
        if tick_match:
            tick_ms = int(tick_match.group(1))
            if self._pps_last_mcu_tick is not None:
                tick_dt = (tick_ms - self._pps_last_mcu_tick) / 1000.0
                if _MIN_DERIVATIVE_DT <= tick_dt <= 5.0:
                    dt = tick_dt
            self._pps_last_mcu_tick = tick_ms

        if dt is None and self._pps_last_time is not None:
            perf_dt = now - self._pps_last_time
            if _MIN_DERIVATIVE_DT <= perf_dt <= 5.0:
                dt = perf_dt

        velocity = (pps[0] * 360.0 / ppr_value, pps[1] * 360.0 / ppr_value)
        if dt is not None and self._pps_last_velocity is not None:
            raw_accel = (
                (velocity[0] - self._pps_last_velocity[0]) / dt,
                (velocity[1] - self._pps_last_velocity[1]) / dt,
            )
            if self._pps_accel_ema is None:
                accel = raw_accel
            else:
                accel = (
                    _ACCEL_EMA_ALPHA * raw_accel[0] + (1.0 - _ACCEL_EMA_ALPHA) * self._pps_accel_ema[0],
                    _ACCEL_EMA_ALPHA * raw_accel[1] + (1.0 - _ACCEL_EMA_ALPHA) * self._pps_accel_ema[1],
                )
            self._pps_accel_ema = accel

            t_rel = now - self.start_timestamp
            self.time_history.append(t_rel)
            self.v1_data.append(velocity[0])
            self.v2_data.append(velocity[1])
            self.a1_data.append(accel[0])
            self.a2_data.append(accel[1])
            self._trim_primary_window(t_rel)
            self.update_plots()

        self._pps_last_velocity = velocity
        self._pps_last_time = now

    def process_commanded_pps(self, pps1, pps2, duration_ticks, ppr=6400):
        if not self.is_running:
            return
        try:
            ppr_value = max(1, int(ppr))
            dt = max(0.0001, int(duration_ticks) / 10000.0)
        except (TypeError, ValueError):
            return

        velocity = (float(pps1) * 360.0 / ppr_value, float(pps2) * 360.0 / ppr_value)
        if self._cmd_last_velocity is None:
            accel = (0.0, 0.0)
        else:
            raw_accel = (
                (velocity[0] - self._cmd_last_velocity[0]) / dt,
                (velocity[1] - self._cmd_last_velocity[1]) / dt,
            )
            if self._cmd_accel_ema is None:
                accel = raw_accel
            else:
                accel = (
                    _ACCEL_EMA_ALPHA * raw_accel[0] + (1.0 - _ACCEL_EMA_ALPHA) * self._cmd_accel_ema[0],
                    _ACCEL_EMA_ALPHA * raw_accel[1] + (1.0 - _ACCEL_EMA_ALPHA) * self._cmd_accel_ema[1],
                )
            self._cmd_accel_ema = accel
        self._cmd_elapsed += dt
        self.cmd_time_history.append(self._cmd_elapsed)
        self.cmd_v1_data.append(velocity[0])
        self.cmd_v2_data.append(velocity[1])
        self.cmd_a1_data.append(accel[0])
        self.cmd_a2_data.append(accel[1])
        self._cmd_last_velocity = velocity
        self._trim_command_window(self._cmd_elapsed)
        self.update_plots()

    def _process_position_sample(self, x, y, dt, t_rel):
        if self.last_pos == (x, y):
            return

        theta = self.kin.inverse(x, y)
        if theta[0] is None:
            return

        if self.last_theta is not None and dt >= _MIN_DERIVATIVE_DT:
            self._append_position_derivative(theta, dt, t_rel)

        self.last_pos = (x, y)
        self.last_theta = (theta[0], theta[1])

    def _append_position_derivative(self, theta, dt, t_rel):
        v1 = (theta[0] - self.last_theta[0]) / dt
        v2 = (theta[1] - self.last_theta[1]) / dt

        if self.last_velocity is not None:
            raw_a1 = (v1 - self.last_velocity[0]) / dt
            raw_a2 = (v2 - self.last_velocity[1]) / dt
            if self._pos_accel_ema is None:
                a1, a2 = raw_a1, raw_a2
            else:
                a1 = _ACCEL_EMA_ALPHA * raw_a1 + (1.0 - _ACCEL_EMA_ALPHA) * self._pos_accel_ema[0]
                a2 = _ACCEL_EMA_ALPHA * raw_a2 + (1.0 - _ACCEL_EMA_ALPHA) * self._pos_accel_ema[1]
            self._pos_accel_ema = (a1, a2)
            self.pos_time_history.append(t_rel)
            self.pos_v1_data.append(v1)
            self.pos_v2_data.append(v2)
            self.pos_a1_data.append(a1)
            self.pos_a2_data.append(a2)
            self._trim_position_window(t_rel)
            self.update_plots()

        self.last_velocity = (v1, v2)

    def _trim_series(self, latest_time, x_data, *series):
        min_time = max(0.0, latest_time - self.window_seconds)
        while x_data and x_data[0] < min_time:
            x_data.pop(0)
            for data in series:
                data.pop(0)

    def _trim_primary_window(self, latest_time):
        self._trim_series(latest_time, self.time_history, self.v1_data, self.v2_data, self.a1_data, self.a2_data)

    def _trim_position_window(self, latest_time):
        self._trim_series(
            latest_time,
            self.pos_time_history,
            self.pos_v1_data,
            self.pos_v2_data,
            self.pos_a1_data,
            self.pos_a2_data,
        )

    def _trim_command_window(self, latest_time):
        self._trim_series(
            latest_time,
            self.cmd_time_history,
            self.cmd_v1_data,
            self.cmd_v2_data,
            self.cmd_a1_data,
            self.cmd_a2_data,
        )

    def update_plots(self):
        self._set_curve_data(self.cv1, self.time_history, self.v1_data)
        self._set_curve_data(self.cv2, self.time_history, self.v2_data)
        self._set_curve_data(self.ca1, self.time_history, self.a1_data)
        self._set_curve_data(self.ca2, self.time_history, self.a2_data)
        self._set_curve_data(self.cv1_pos, self.pos_time_history, self.pos_v1_data)
        self._set_curve_data(self.cv2_pos, self.pos_time_history, self.pos_v2_data)
        self._set_curve_data(self.ca1_pos, self.pos_time_history, self.pos_a1_data)
        self._set_curve_data(self.ca2_pos, self.pos_time_history, self.pos_a2_data)
        self._set_curve_data(self.cv1_cmd, self.cmd_time_history, self.cmd_v1_data)
        self._set_curve_data(self.cv2_cmd, self.cmd_time_history, self.cmd_v2_data)
        self._set_curve_data(self.ca1_cmd, self.cmd_time_history, self.cmd_a1_data)
        self._set_curve_data(self.ca2_cmd, self.cmd_time_history, self.cmd_a2_data)
        self._apply_time_window()

    def _apply_time_window(self):
        latest = max(
            self.time_history[-1] if self.time_history else 0.0,
            self.pos_time_history[-1] if self.pos_time_history else 0.0,
            self.cmd_time_history[-1] if self.cmd_time_history else 0.0,
        )
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
