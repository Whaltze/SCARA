from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QTextEdit,
    QGridLayout,
    QGroupBox,
    QComboBox,
    QLineEdit,
    QSizePolicy,
    QSlider,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen


class HandwritingPad(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._strokes = []
        self._drawing = False
        self.setMinimumHeight(150)
        self.setStyleSheet("background: #fafafa; border: 1px solid #888;")

    def normalized_strokes(self):
        return [[point for point in stroke] for stroke in self._strokes if len(stroke) >= 2]

    def clear(self):
        self._strokes = []
        self._drawing = False
        self.update()

    def _event_point(self, event):
        pos = event.position()
        w = max(1, self.width())
        h = max(1, self.height())
        x = min(1.0, max(0.0, pos.x() / w))
        y = min(1.0, max(0.0, pos.y() / h))
        return (x, y)

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        self._drawing = True
        self._strokes.append([self._event_point(event)])
        self.update()

    def mouseMoveEvent(self, event):
        if not self._drawing or not self._strokes:
            return
        point = self._event_point(event)
        last = self._strokes[-1][-1]
        if abs(point[0] - last[0]) + abs(point[1] - last[1]) >= 0.004:
            self._strokes[-1].append(point)
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drawing = False
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#fafafa"))
        grid_pen = QPen(QColor("#dddddd"), 1)
        painter.setPen(grid_pen)
        for i in range(1, 4):
            x = int(self.width() * i / 4)
            y = int(self.height() * i / 4)
            painter.drawLine(x, 0, x, self.height())
            painter.drawLine(0, y, self.width(), y)
        painter.setPen(QPen(QColor("#222222"), 2))
        for stroke in self._strokes:
            for p0, p1 in zip(stroke, stroke[1:]):
                painter.drawLine(
                    int(p0[0] * self.width()),
                    int(p0[1] * self.height()),
                    int(p1[0] * self.width()),
                    int(p1[1] * self.height()),
                )
        painter.setPen(QPen(QColor("#888888"), 1))
        painter.drawRect(0, 0, self.width() - 1, self.height() - 1)


class ScaraUiMixin:
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        left_panel = QVBoxLayout()
        main_layout.addLayout(left_panel, 3) 

        # 1. 串口设置
        serial_group = QGroupBox("串口设置")
        s_grid = QGridLayout()
        s_grid.addWidget(QLabel("串口:"), 0, 0)
        self.port_combo = QComboBox()
        s_grid.addWidget(self.port_combo, 0, 1)
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        s_grid.addWidget(self.btn_refresh, 0, 2)
        s_grid.addWidget(QLabel("波特率:"), 1, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200"])
        s_grid.addWidget(self.baud_combo, 1, 1)
        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self.toggle_serial)
        s_grid.addWidget(self.btn_connect, 1, 2)
        self.serial_status = QLabel("未连接")
        self.serial_status.setStyleSheet("color: gray; font-weight: bold;")
        s_grid.addWidget(self.serial_status, 2, 0, 1, 3)
        serial_group.setLayout(s_grid)
        left_panel.addWidget(serial_group)

        # 2. 硬件控制
        hw_group = QGroupBox("硬件控制")
        hw_layout = QGridLayout()
        box_w, box_h = 150, 28
        hw_layout.addWidget(QLabel("脉冲/圈(需与驱动拨码一致):"), 0, 0)
        self.microstep_combo = QComboBox()
        self.microstep_combo.addItems(["400", "1600", "3200", "6400"])
        self.microstep_combo.setCurrentText("1600")
        self.microstep_combo.setFixedSize(box_w, box_h)
        self.microstep_combo.currentTextChanged.connect(self.on_microstep_changed)
        hw_layout.addWidget(self.microstep_combo, 0, 1, Qt.AlignLeft)
        hw_layout.addWidget(QLabel("运行速度(mm/s):"), 1, 0)
        self.hw_speed_input = QLineEdit("20.0")
        self.hw_speed_input.setFixedSize(box_w, box_h)
        hw_layout.addWidget(self.hw_speed_input, 1, 1, Qt.AlignLeft)
        hw_group.setLayout(hw_layout)
        left_panel.addWidget(hw_group)

        # 3. 方向点动
        jog_group = QGroupBox("方向点动")
        jog_grid = QGridLayout()
        self.btns = {"UP": QPushButton("前进"), "DOWN": QPushButton("后退"), "LEFT": QPushButton("左移"), "RIGHT": QPushButton("右移")}
        for b in self.btns.values(): 
            b.setFixedSize(100, 30)
        
        # 电机单独控制按钮（四个角）
        self.motor_btns = {
            "M1_POS": QPushButton("M1+"),  # 电机1正向旋转
            "M1_NEG": QPushButton("M1-"),  # 电机1逆向旋转
            "M2_POS": QPushButton("M2+"),  # 电机2正向旋转
            "M2_NEG": QPushButton("M2-"),  # 电机2逆向旋转
        }
        for b in self.motor_btns.values():
            b.setFixedSize(100, 30)  # 与前后左右方向按钮等大
            b.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold;")
        
        # 布局：四个角放置电机控制按钮，中心放置方向点动按钮
        jog_grid.addWidget(self.motor_btns["M1_POS"], 0, 0)  # 左上角 - 电机1正向
        jog_grid.addWidget(self.motor_btns["M2_POS"], 0, 2)  # 右上角 - 电机2正向
        jog_grid.addWidget(self.motor_btns["M1_NEG"], 2, 0)  # 左下角 - 电机1逆向
        jog_grid.addWidget(self.motor_btns["M2_NEG"], 2, 2)  # 右下角 - 电机2逆向
        
        jog_grid.addWidget(self.btns["UP"], 0, 1)
        jog_grid.addWidget(self.btns["LEFT"], 1, 0)
        jog_grid.addWidget(self.btns["RIGHT"], 1, 2)
        jog_grid.addWidget(self.btns["DOWN"], 2, 1)
        self.jog_speed_input = QLineEdit("30.0")
        self.jog_speed_input.setFixedSize(100, 30)
        self.jog_speed_input.setAlignment(Qt.AlignCenter)
        jog_grid.addWidget(self.jog_speed_input, 1, 1)
        jog_group.setLayout(jog_grid)
        left_panel.addWidget(jog_group)
        
        # 方向点动连接
        self.btns["UP"].clicked.connect(lambda: self.add_jog(0, 10))
        self.btns["DOWN"].clicked.connect(lambda: self.add_jog(0, -10))
        self.btns["LEFT"].clicked.connect(lambda: self.add_jog(-10, 0))
        self.btns["RIGHT"].clicked.connect(lambda: self.add_jog(10, 0))
        
        # 电机单独控制连接（半步/半圈旋转）
        self.motor_btns["M1_POS"].clicked.connect(lambda: self.motor_jog(1, 1))   # 电机1正向
        self.motor_btns["M1_NEG"].clicked.connect(lambda: self.motor_jog(1, -1))  # 电机1逆向
        self.motor_btns["M2_POS"].clicked.connect(lambda: self.motor_jog(2, 1))   # 电机2正向
        self.motor_btns["M2_NEG"].clicked.connect(lambda: self.motor_jog(2, -1))  # 电机2逆向

        # 4. 轨迹规划
        interp_group = QGroupBox("轨迹规划")
        i_grid = QGridLayout()
        i_grid.addWidget(QLabel("模式:"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["G1 直线", "G2 顺圆", "G3 逆圆"])
        self.mode_combo.setFixedSize(box_w, box_h)
        i_grid.addWidget(self.mode_combo, 0, 1)
        i_grid.addWidget(QLabel("目标X:"), 1, 0)
        self.target_x = QLineEdit("150.0")
        self.target_x.setFixedSize(box_w, box_h)
        i_grid.addWidget(self.target_x, 1, 1)
        i_grid.addWidget(QLabel("目标Y:"), 1, 2)
        self.target_y = QLineEdit("250.0")
        self.target_y.setFixedSize(box_w, box_h)
        i_grid.addWidget(self.target_y, 1, 3)
        i_grid.addWidget(QLabel("半径:"), 2, 0)
        self.radius_r = QLineEdit("60.0")
        self.radius_r.setFixedSize(box_w, box_h)
        i_grid.addWidget(self.radius_r, 2, 1)
        btn_start = QPushButton("🚀 启动 轨迹运动")
        btn_start.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold;")
        btn_start.clicked.connect(lambda: self.plan_trajectory(silent=False))
        i_grid.addWidget(btn_start, 3, 0, 1, 4)
        interp_group.setLayout(i_grid)
        left_panel.addWidget(interp_group)

        # 5. 固定轨迹
        ft_group = QGroupBox("固定轨迹")
        ft_grid = QGridLayout()
        ft_grid.addWidget(QLabel("起始点X:"), 0, 0)
        self.car_start_x = QLineEdit("75.0")
        self.car_start_x.setFixedSize(box_w, box_h)
        ft_grid.addWidget(self.car_start_x, 0, 1)
        ft_grid.addWidget(QLabel("起始点Y:"), 1, 0)
        self.car_start_y = QLineEdit("200.0")
        self.car_start_y.setFixedSize(box_w, box_h)
        ft_grid.addWidget(self.car_start_y, 1, 1)
        
        btn_car = QPushButton("小车1")
        btn_car.setStyleSheet("background-color: #3498db; color: white; font-weight: bold;")
        btn_car.clicked.connect(self.plan_car_path)
        ft_grid.addWidget(btn_car, 2, 0)
        
        btn_car2 = QPushButton("小车2")
        btn_car2.setStyleSheet("background-color: #3498db; color: white; font-weight: bold;")
        btn_car2.clicked.connect(self.plan_car2_path)
        ft_grid.addWidget(btn_car2, 2, 1)

        btn_text_fdu = QPushButton("福州大学")
        btn_text_fdu.setStyleSheet("background-color: #16a085; color: white; font-weight: bold;")
        btn_text_fdu.clicked.connect(lambda: self.plan_fixed_text_path("福州大学"))
        ft_grid.addWidget(btn_text_fdu, 3, 0)

        btn_text_fzu = QPushButton("FZU")
        btn_text_fzu.setStyleSheet("background-color: #16a085; color: white; font-weight: bold;")
        btn_text_fzu.clicked.connect(lambda: self.plan_fixed_text_path("FZU"))
        ft_grid.addWidget(btn_text_fzu, 3, 1)
        
        ft_group.setLayout(ft_grid)
        left_panel.addWidget(ft_group)

        handwriting_group = QGroupBox("手写板")
        handwriting_lay = QVBoxLayout()
        self.handwriting_pad = HandwritingPad()
        handwriting_lay.addWidget(self.handwriting_pad)
        handwriting_btns = QHBoxLayout()
        self.btn_hand_clear = QPushButton("清空")
        self.btn_hand_preview = QPushButton("预览")
        self.btn_hand_run = QPushButton("运行手写")
        handwriting_btns.addWidget(self.btn_hand_clear)
        handwriting_btns.addWidget(self.btn_hand_preview)
        handwriting_btns.addWidget(self.btn_hand_run)
        handwriting_lay.addLayout(handwriting_btns)
        handwriting_group.setLayout(handwriting_lay)
        left_panel.addWidget(handwriting_group)
        self.btn_hand_clear.clicked.connect(self.clear_handwriting)
        self.btn_hand_preview.clicked.connect(self.plan_handwriting_preview)
        self.btn_hand_run.clicked.connect(self.plan_handwriting_path)
        left_panel.addStretch()

        # --- 中间面板 (3/10) ---
        mid_panel = QVBoxLayout()
        main_layout.addLayout(mid_panel, 3)
        
        coord_group = QGroupBox("实时坐标")
        c_lay = QVBoxLayout()
        self.status_label = QLabel("坐标: X=75.0, Y=220.0")
        self.status_label.setFont(QFont("Arial", 11, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignCenter)
        c_lay.addWidget(self.status_label)
        self.feedback_pose_label = QLabel("回传末端: X=--, Y=--")
        self.feedback_joint_label = QLabel("回传角度: M1=-- deg, M2=-- deg")
        self.feedback_pulse_label = QLabel("脉冲/PPS: P=--,--  A1=--/--  A2=--/--")
        for label in (self.feedback_pose_label, self.feedback_joint_label, self.feedback_pulse_label):
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("color: #cccccc;")
            c_lay.addWidget(label)
        coord_group.setLayout(c_lay)
        mid_panel.addWidget(coord_group)

        # 下位机健康监控模块
        mcu_status_group = QGroupBox("下位机健康监控")
        mcu_status_lay = QGridLayout()
        mcu_status_lay.setContentsMargins(10, 10, 10, 10)
        mcu_status_lay.setSpacing(8)
        self.lbl_mcu_err = QLabel("错误码: 0")
        self.lbl_mcu_tick = QLabel("MCU时间: 0 ms")
        self.lbl_mcu_gbuf = QLabel("缓冲区占用: 0 / 32")
        self.lbl_mcu_queue = QLabel("队列负载(Q): 0")
        self.lbl_mcu_err.setStyleSheet("font-weight: bold; color: #e06c75;")
        self.lbl_mcu_tick.setStyleSheet("color: #61afef;")
        self.lbl_mcu_gbuf.setStyleSheet("color: #98c379;")
        self.lbl_mcu_queue.setStyleSheet("color: #d19a66;")
        mcu_status_lay.addWidget(self.lbl_mcu_err, 0, 0)
        mcu_status_lay.addWidget(self.lbl_mcu_tick, 0, 1)
        mcu_status_lay.addWidget(self.lbl_mcu_gbuf, 1, 0)
        mcu_status_lay.addWidget(self.lbl_mcu_queue, 1, 1)
        mcu_status_group.setLayout(mcu_status_lay)
        mid_panel.addWidget(mcu_status_group)
        
        teach_group = QGroupBox("示教")
        t_lay = QVBoxLayout()
        self.btn_start_rec = QPushButton("开始轨迹记录")
        self.btn_end_rec = QPushButton("结束轨迹记录")
        self.btn_rec_point = QPushButton("记录轨迹点")
        self.btn_clear_point = QPushButton("清除轨迹点")
        self.btn_replay = QPushButton("开始轨迹复现")
        t_lay.addWidget(self.btn_start_rec)
        t_lay.addWidget(self.btn_end_rec)
        t_lay.addWidget(self.btn_rec_point)
        t_lay.addWidget(self.btn_clear_point)
        t_lay.addWidget(self.btn_replay)
        teach_group.setLayout(t_lay)
        mid_panel.addWidget(teach_group)
        self.btn_start_rec.clicked.connect(self.start_recording)
        self.btn_end_rec.clicked.connect(self.stop_recording)
        self.btn_rec_point.clicked.connect(self.record_single_point)
        self.btn_clear_point.clicked.connect(self.clear_teach_points)
        self.btn_replay.clicked.connect(self.start_playback)

        # 视觉识别组
        vision_group = QGroupBox("视觉识别")
        v_grid = QGridLayout()
        v_grid.addWidget(QLabel("摄像头ID:"), 0, 0)
        self.cam_id_combo = QComboBox()
        self.cam_id_combo.addItems(["0","1","2"])
        v_grid.addWidget(self.cam_id_combo, 0, 1, 1, 5) 
        v_btns_lay = QHBoxLayout()
        self.btn_cam_open = QPushButton("打开摄像头")
        self.btn_cam_close = QPushButton("关闭摄像头")
        v_btns_lay.addWidget(self.btn_cam_open)
        v_btns_lay.addWidget(self.btn_cam_close)
        v_grid.addLayout(v_btns_lay, 1, 0, 1, 6)
        self.btn_color_toggle = QPushButton("颜色识别: ON")
        self.btn_edge_toggle = QPushButton("边缘检测: OFF")
        v_grid.addWidget(self.btn_color_toggle, 2, 0, 1, 6)
        v_grid.addWidget(self.btn_edge_toggle, 3, 0, 1, 6)
        
        h_sel_lay = QHBoxLayout()
        h_sel_lay.addWidget(QLabel("识别颜色:"))
        self.color_sel = QComboBox()
        self.color_sel.addItems(["红色","黄色","绿色","蓝色"])
        h_sel_lay.addWidget(self.color_sel)
        h_sel_lay.addWidget(QLabel("颜色容差:"))
        self.thresh_sel = QComboBox()
        self.thresh_sel.addItems(["30","50","70","100"])
        h_sel_lay.addWidget(self.thresh_sel)
        v_grid.addLayout(h_sel_lay, 4, 0, 1, 6)

        self.sliders = {}
        slider_params = [
            ("Hmin", 179, "Hmax", 179),
            ("Smin", 255, "Smax", 255),
            ("Vmin", 255, "Vmax", 255)
        ]
        for i, (name1, mx1, name2, mx2) in enumerate(slider_params):
            row = 5 + i
            v_grid.addWidget(QLabel(f"{name1}:"), row, 0)
            sl1 = QSlider(Qt.Horizontal)
            sl1.setRange(0, mx1)
            val_lab1 = QLabel("0")
            sl1.valueChanged.connect(lambda v, l=val_lab1: l.setText(str(v)))
            sl1.valueChanged.connect(self.sync_hsv_to_thread)
            v_grid.addWidget(sl1, row, 1)
            v_grid.addWidget(val_lab1, row, 2)
            self.sliders[name1] = sl1
            v_grid.addWidget(QLabel(f"{name2}:"), row, 3)
            sl2 = QSlider(Qt.Horizontal)
            sl2.setRange(0, mx2)
            val_lab2 = QLabel("0")
            sl2.valueChanged.connect(lambda v, l=val_lab2: l.setText(str(v)))
            sl2.valueChanged.connect(self.sync_hsv_to_thread)
            v_grid.addWidget(sl2, row, 4)
            v_grid.addWidget(val_lab2, row, 5)
            self.sliders[name2] = sl2

        self.btn_vision_trace = QPushButton("🎨 启动视觉循迹轨迹")
        self.btn_vision_trace.setStyleSheet("background-color: #9b59b6; color: white; font-weight: bold;")
        v_grid.addWidget(self.btn_vision_trace, 8, 0, 1, 6)
        vision_group.setLayout(v_grid)
        mid_panel.addWidget(vision_group)
        
        self.btn_cam_open.clicked.connect(self.start_cameras)
        self.btn_cam_close.clicked.connect(self.stop_cameras)
        self.btn_color_toggle.clicked.connect(self.toggle_color)
        self.btn_edge_toggle.clicked.connect(self.toggle_edge)
        self.color_sel.currentTextChanged.connect(self.update_v_params)
        self.thresh_sel.currentTextChanged.connect(self.update_v_params)
        self.btn_vision_trace.clicked.connect(self.plan_vision_trajectory)

        task_group = QGroupBox("系统任务与安全")
        task_lay = QVBoxLayout()
        task_lay.setContentsMargins(10, 5, 10, 5) 
        self.btn_reset_home = QPushButton("系统一键复位 (回0点)")
        self.btn_emergency_stop = QPushButton("🛑 紧急停止")
        self.btn_emergency_stop.setText("\u6025\u505c (\u4fdd\u7559\u961f\u5217)")
        self.btn_stop_motion = QPushButton("\u505c\u6b62 (\u6e05\u9664\u961f\u5217)")
        self.btn_stop_motion.setFixedHeight(38)
        self.btn_emergency_stop.setFixedHeight(45)
        self.btn_stop_motion.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold;")
        self.btn_emergency_stop.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
        task_lay.addWidget(self.btn_reset_home)
        task_lay.addWidget(self.btn_stop_motion)
        task_lay.addWidget(self.btn_emergency_stop)
        task_group.setLayout(task_lay)
        mid_panel.addWidget(task_group)
        mid_panel.addStretch()
        self.btn_reset_home.clicked.connect(self.system_reset)
        self.btn_stop_motion.clicked.connect(self.stop_motion)
        self.btn_emergency_stop.clicked.connect(self.emergency_stop)

        # 右侧面板
        right_panel = QVBoxLayout()
        right_panel.setContentsMargins(0, 0, 0, 0)
        right_panel.setSpacing(4)
        main_layout.addLayout(right_panel, 4)
        self.fig = Figure(figsize=(8, 5))
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_panel.addWidget(self.canvas, 6)
        self.ax = self.fig.add_subplot(111)
        
        plot_ctrl_lay = QHBoxLayout()
        self.plot_mode_combo = QComboBox()
        self.plot_mode_combo.addItems(["通讯发送内容", "通讯接收内容"])
        self.plot_mode_combo.setToolTip("选择绘图框轨迹的依据来源")
        plot_ctrl_lay.addWidget(self.plot_mode_combo, 1)
        
        self.btn_clear_plot = QPushButton("清空绘图轨迹")
        self.btn_clear_plot.clicked.connect(self.clear_plot_trace)
        plot_ctrl_lay.addWidget(self.btn_clear_plot, 1)
        right_panel.addLayout(plot_ctrl_lay)
        
        bottom_h = QHBoxLayout()
        self.cam_label = QLabel("摄像头未启动")
        self.cam_label.setStyleSheet("background: black; color: white; border: 1px solid #444;")
        self.cam_label.setAlignment(Qt.AlignCenter)
        self.cam_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        bottom_h.addWidget(self.cam_label, 1)
        
        log_v = QVBoxLayout()
        log_v.addWidget(QLabel("通讯指令记录:"))
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setStyleSheet("background: #1e1e1e; color: #61afef; font-family: Consolas;")
        log_v.addWidget(self.log_display)
        self.btn_clear_log = QPushButton("清空发送数据")
        self.btn_clear_log.clicked.connect(lambda: self.log_display.clear())
        log_v.addWidget(self.btn_clear_log)
        bottom_h.addLayout(log_v, 1)
        right_panel.addLayout(bottom_h, 4)

    # --- 通用工具 ---
