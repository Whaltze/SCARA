import numpy as np
import matplotlib.ticker as ticker


class ScaraPlotMixin:
    def clear_plot_trace(self):
        self.history_x, self.history_y = [self.cur_x], [self.cur_y]
        self.feedback_x, self.feedback_y = [], []
        self.preview_x, self.preview_y = [], []
        self.preview_label = ""
        self.update_plot()

    def log_error(self, m):
        self.log_display.append(f"<font color='red'>[ERR] {m}</font>")

    def precompute_workspace(self):
        xr, yr = np.linspace(75 - 330, 75 + 330, 60), np.linspace(0, 480, 60)
        self.ws_x, self.ws_y = [], []
        for xi in xr:
            for yi in yr:
                if self.inverse_kinematics(xi, yi)[0] is not None:
                    self.ws_x.append(xi)
                    self.ws_y.append(yi)

    def set_planned_preview(self, path, label="规划预览"):
        self.preview_x = [float(p[0]) for p in path]
        self.preview_y = [float(p[1]) for p in path]
        self.preview_label = label
        self.update_plot()

    def append_feedback_point(self, x, y):
        if not hasattr(self, "feedback_x"):
            self.feedback_x, self.feedback_y = [], []
        if self.feedback_x and abs(self.feedback_x[-1] - x) < 0.01 and abs(self.feedback_y[-1] - y) < 0.01:
            return
        self.feedback_x.append(x)
        self.feedback_y.append(y)
        if len(self.feedback_x) > 5000:
            self.feedback_x = self.feedback_x[-5000:]
            self.feedback_y = self.feedback_y[-5000:]

    def setup_plot_interaction(self):
        if getattr(self, "_plot_events_connected", False):
            return
        self._plot_events_connected = True
        self._plot_drag = None
        self.canvas.mpl_connect("scroll_event", self._on_plot_scroll)
        self.canvas.mpl_connect("button_press_event", self._on_plot_press)
        self.canvas.mpl_connect("button_release_event", self._on_plot_release)
        self.canvas.mpl_connect("motion_notify_event", self._on_plot_motion)

    def _init_plot_artists(self):
        if getattr(self, "_plot_ready", False):
            return
        self.setup_plot_interaction()
        self.fig.subplots_adjust(left=0.055, right=0.995, bottom=0.075, top=0.995)
        self.ax.clear()
        self.ws_artist = self.ax.scatter(self.ws_x, self.ws_y, s=1, color="#e5f2ff", label="workspace")
        (self.preview_line,) = self.ax.plot([], [], color="#2ca02c", lw=1.2, label="规划预览")
        (self.sent_line,) = self.ax.plot([], [], color="#1f77b4", lw=1.0, alpha=0.85, label="已发送")
        (self.feedback_line,) = self.ax.plot([], [], color="#ffbf00", lw=1.0, alpha=0.9, label="下位机反馈")
        (self.left_arm_line,) = self.ax.plot([], [], "ro-", lw=3)
        (self.right_arm_line,) = self.ax.plot([], [], "bo-", lw=3)
        (self.current_point_line,) = self.ax.plot([], [], "ko", ms=4)
        self.ax.xaxis.set_major_locator(ticker.MultipleLocator(20))
        self.ax.yaxis.set_major_locator(ticker.MultipleLocator(20))
        self.ax.tick_params(axis="x", labelrotation=90, labelsize=7)
        self.ax.tick_params(axis="y", labelsize=7)
        self.ax.grid(True, alpha=0.15)
        self.ax.margins(0)
        self.ax.set_aspect("equal", adjustable="datalim")
        self._plot_ready = True

    def update_plot(self, q1_deg=None, q2_deg=None):
        if q1_deg is None:
            ik = self.inverse_kinematics(self.cur_x, self.cur_y)
            q1_deg, q2_deg = ik if ik and ik[0] is not None else (90, 90)

        if not hasattr(self, "preview_x"):
            self.preview_x, self.preview_y = [], []
            self.preview_label = ""
        if not hasattr(self, "feedback_x"):
            self.feedback_x, self.feedback_y = [], []

        self._init_plot_artists()
        q1, q2 = np.radians(q1_deg), np.radians(q2_deg)
        c1 = [self.L1 * np.cos(q1), self.L1 * np.sin(q1)]
        c2 = [self.L0 + self.L1 * np.cos(q2), self.L1 * np.sin(q2)]

        self.preview_line.set_data(self.preview_x, self.preview_y)
        self.sent_line.set_data(self.history_x, self.history_y)
        self.feedback_line.set_data(self.feedback_x, self.feedback_y)
        self.left_arm_line.set_data([0, c1[0], self.cur_x], [0, c1[1], self.cur_y])
        self.right_arm_line.set_data([self.L0, c2[0], self.cur_x], [0, c2[1], self.cur_y])
        self.current_point_line.set_data([self.cur_x], [self.cur_y])
        if not getattr(self, "_plot_view_initialized", False):
            self._fit_motion_axis()
            self._plot_view_initialized = True

        self.status_label.setText(f"坐标: X={self.cur_x:.1f}, Y={self.cur_y:.1f}")
        self.canvas.draw_idle()

    def _fit_motion_axis(self):
        if self.preview_x:
            xs = [0.0, self.L0, self.cur_x]
            ys = [0.0, 0.0, self.cur_y]
            for data_x, data_y in (
                (self.preview_x, self.preview_y),
                (self.history_x, self.history_y),
                (self.feedback_x, self.feedback_y),
            ):
                xs.extend(data_x)
                ys.extend(data_y)
        else:
            xs = list(self.ws_x)
            ys = list(self.ws_y)
        if not xs or not ys:
            self.ax.set_xlim(-220, 420)
            self.ax.set_ylim(-20, 480)
            return
        self.ax.set_xlim(min(xs) - 20, max(xs) + 20)
        self.ax.set_ylim(min(ys) - 20, max(ys) + 20)

    def _on_plot_scroll(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        scale = 0.8 if event.button == "up" else 1.25
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        new_w = (x1 - x0) * scale
        new_h = (y1 - y0) * scale
        rx = (event.xdata - x0) / (x1 - x0)
        ry = (event.ydata - y0) / (y1 - y0)
        self.ax.set_xlim(event.xdata - new_w * rx, event.xdata + new_w * (1.0 - rx))
        self.ax.set_ylim(event.ydata - new_h * ry, event.ydata + new_h * (1.0 - ry))
        self._plot_user_view = True
        self.canvas.draw_idle()

    def _on_plot_press(self, event):
        if event.inaxes != self.ax or event.button != 1 or event.xdata is None or event.ydata is None:
            return
        self._plot_drag = (event.xdata, event.ydata, self.ax.get_xlim(), self.ax.get_ylim())
        self._plot_user_view = True

    def _on_plot_release(self, event):
        self._plot_drag = None

    def _on_plot_motion(self, event):
        if not self._plot_drag or event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        sx, sy, xlim, ylim = self._plot_drag
        dx = sx - event.xdata
        dy = sy - event.ydata
        self.ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
        self.ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
        self.canvas.draw_idle()
