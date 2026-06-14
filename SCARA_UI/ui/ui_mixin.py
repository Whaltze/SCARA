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
    QFontComboBox,
    QLineEdit,
    QSizePolicy,
    QSlider,
    QScrollArea,
    QCheckBox,
    QDoubleSpinBox,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen


class HandwritingPad(QWidget):
    CAPTURE_MIN_DELTA = 0.001

    def __init__(self, parent=None):
        super().__init__(parent)
        self._strokes = []
        self._drawing = False
        self.setMinimumHeight(110)
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
        if self._append_event_point(event):
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._append_event_point(event)
            self._drawing = False
            self.update()

    def _append_event_point(self, event):
        if not self._strokes:
            return False
        point = self._event_point(event)
        last = self._strokes[-1][-1]
        if abs(point[0] - last[0]) + abs(point[1] - last[1]) < self.CAPTURE_MIN_DELTA:
            return False
        self._strokes[-1].append(point)
        return True

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
        central_widget.setStyleSheet(
            "QWidget { font-size: 9pt; } "
            "QGroupBox { margin-top: 8px; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 2px; } "
            "QPushButton { min-height: 22px; padding: 2px 5px; } "
            "QLineEdit, QComboBox { min-height: 22px; }"
        )
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)
        
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setMinimumWidth(200)
        left_scroll.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        left_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_widget = QWidget()
        left_panel = QVBoxLayout(left_widget)
        left_panel.setContentsMargins(4, 4, 4, 4)
        left_panel.setSpacing(5)
        left_scroll.setWidget(left_widget)
        main_layout.addWidget(left_scroll, 1)

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
        # box_w/box_h 控制左侧输入控件尺寸；屏幕放不下时优先调小这里或依靠滚动区。
        box_w, box_h = 120, 24
        hw_layout.addWidget(QLabel("脉冲/圈(需与驱动拨码一致):"), 0, 0)
        self.microstep_combo = QComboBox()
        self.microstep_combo.addItems(["400", "1600", "3200", "6400", "12800"])
        self.microstep_combo.setEditable(True)
        # Default 6400 PPR matches firmware and the physical driver DIP switches.
        self.microstep_combo.setCurrentText("6400")
        self.microstep_combo.setFixedSize(box_w, box_h)
        self.microstep_combo.activated.connect(lambda *_: self.on_microstep_changed())
        if self.microstep_combo.lineEdit() is not None:
            self.microstep_combo.lineEdit().editingFinished.connect(self.on_microstep_changed)
        hw_layout.addWidget(self.microstep_combo, 0, 1, Qt.AlignLeft)
        hw_layout.addWidget(QLabel("运行速度(mm/s):"), 1, 0)
        self.hw_speed_input = QLineEdit("20.0")
        self.hw_speed_input.setFixedSize(box_w, box_h)
        hw_layout.addWidget(self.hw_speed_input, 1, 1, Qt.AlignLeft)
        hw_layout.addWidget(QLabel("运行加速度(mm/s²):"), 2, 0)
        self.hw_accel_input = QLineEdit("100.0")
        self.hw_accel_input.setFixedSize(box_w, box_h)
        hw_layout.addWidget(self.hw_accel_input, 2, 1, Qt.AlignLeft)
        self.jog_pps_label = QLabel("目标PPS --  峰值PPS --  周期 --")
        self.jog_pps_label.setStyleSheet("color: #aaaaaa;")
        hw_layout.addWidget(self.jog_pps_label, 3, 0, 1, 2)
        self.hw_speed_input.textChanged.connect(lambda *_: self.update_jog_pps_preview())
        self.hw_accel_input.textChanged.connect(lambda *_: self.update_jog_pps_preview())
        hw_group.setLayout(hw_layout)
        left_panel.addWidget(hw_group)

        # 3. 方向点动
        jog_group = QGroupBox("方向点动")
        jog_grid = QGridLayout()
        jog_grid.setSpacing(4)  # ?????
        self.btns = {"UP": QPushButton("前进"), "DOWN": QPushButton("后退"), "LEFT": QPushButton("左移"), "RIGHT": QPushButton("右移")}
        for b in self.btns.values(): 
            b.setFixedSize(82, 24)
        
        # 电机单独控制按钮（四个角）
        self.motor_btns = {
            "M1_POS": QPushButton("M1+"),  # 电机1正向旋转
            "M1_NEG": QPushButton("M1-"),  # 电机1逆向旋转
            "M2_POS": QPushButton("M2+"),  # 电机2正向旋转
            "M2_NEG": QPushButton("M2-"),  # 电机2逆向旋转
        }
        for b in self.motor_btns.values():
            b.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold;")
            b.setFixedSize(82, 24)  # 与前后左右方向按钮等大
        
        # 布局：四个角放置电机控制按钮，中心放置方向点动按钮
        jog_grid.addWidget(self.motor_btns["M1_POS"], 0, 0)  # 左上角 - 电机1正向
        jog_grid.addWidget(self.motor_btns["M2_POS"], 0, 2)  # 右上角 - 电机2正向
        jog_grid.addWidget(self.motor_btns["M1_NEG"], 2, 0)  # 左下角 - 电机1逆向
        jog_grid.addWidget(self.motor_btns["M2_NEG"], 2, 2)  # 右下角 - 电机2逆向
        
        jog_grid.addWidget(self.btns["UP"], 0, 1)
        jog_grid.addWidget(self.btns["LEFT"], 1, 0)
        jog_grid.addWidget(self.btns["RIGHT"], 1, 2)
        jog_grid.addWidget(self.btns["DOWN"], 2, 1)
        self.jog_step_input = QLineEdit("10.0")
        self.jog_step_input.setFixedSize(82, 24)
        self.jog_step_input.setAlignment(Qt.AlignCenter)
        self.jog_step_input.textChanged.connect(lambda *_: self.update_jog_pps_preview())
        jog_grid.addWidget(self.jog_step_input, 1, 1)
        jog_group.setLayout(jog_grid)
        left_panel.addWidget(jog_group)
        
        # 方向点动连接
        self.btns["UP"].clicked.connect(lambda: self.add_jog_step(0, 1))
        self.btns["DOWN"].clicked.connect(lambda: self.add_jog_step(0, -1))
        self.btns["LEFT"].clicked.connect(lambda: self.add_jog_step(-1, 0))
        self.btns["RIGHT"].clicked.connect(lambda: self.add_jog_step(1, 0))
        
        # 电机单独控制连接（半步/半圈旋转）
        self.motor_btns["M1_POS"].clicked.connect(lambda: self.motor_jog_direct(1, 1))   # 电机1正向
        self.motor_btns["M1_NEG"].clicked.connect(lambda: self.motor_jog_direct(1, -1))  # 电机1逆向
        self.motor_btns["M2_POS"].clicked.connect(lambda: self.motor_jog_direct(2, 1))   # 电机2正向
        self.motor_btns["M2_NEG"].clicked.connect(lambda: self.motor_jog_direct(2, -1))  # 电机2逆向
        self.update_jog_pps_preview()

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

        # 自定义轨迹：下拉选择预设轨迹 + 运行按钮（便于后续扩展画画等图案）
        ft_grid.addWidget(QLabel("自定义轨迹:"), 3, 0)
        self.preset_traj_combo = QComboBox()
        self.preset_traj_combo.setFixedSize(box_w, box_h)
        self.preset_traj_combo.addItems(self.preset_trajectory_names())
        ft_grid.addWidget(self.preset_traj_combo, 3, 1)

        btn_preset_run = QPushButton("运行轨迹")
        btn_preset_run.setStyleSheet("background-color: #16a085; color: white; font-weight: bold;")
        btn_preset_run.clicked.connect(self.run_selected_preset_trajectory)
        # 跨两列铺满，避免单独占用第三列而向右突出，与本组其它行保持同一右边界。
        ft_grid.addWidget(btn_preset_run, 4, 0, 1, 2)

        ft_grid.addWidget(QLabel("空心字:"), 5, 0)
        self.text_outline_input = QLineEdit("福州大学")
        self.text_outline_input.setFixedHeight(box_h)
        ft_grid.addWidget(self.text_outline_input, 5, 1)

        ft_grid.addWidget(QLabel("字体:"), 6, 0)
        self.text_font_combo = QFontComboBox()
        self.text_font_combo.setFixedHeight(box_h)
        self.text_font_combo.setCurrentFont(QFont("Microsoft YaHei", 9))
        ft_grid.addWidget(self.text_font_combo, 6, 1)

        ft_grid.addWidget(QLabel("高度(mm):"), 7, 0)
        self.text_height_input = QLineEdit("80.0")
        self.text_height_input.setFixedSize(box_w, box_h)
        ft_grid.addWidget(self.text_height_input, 7, 1)

        btn_text_preview = QPushButton("预览空心字")
        btn_text_preview.setStyleSheet("background-color: #8e44ad; color: white; font-weight: bold;")
        btn_text_preview.clicked.connect(self.preview_text_outline_path)
        ft_grid.addWidget(btn_text_preview, 8, 0)

        btn_text_run = QPushButton("运行空心字")
        btn_text_run.setStyleSheet("background-color: #16a085; color: white; font-weight: bold;")
        btn_text_run.clicked.connect(self.plan_text_outline_path)
        ft_grid.addWidget(btn_text_run, 8, 1)
        
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

        # --- ???? (3/10) --- ?? QScrollArea ???????????
        mid_scroll = QScrollArea()
        mid_scroll.setWidgetResizable(True)
        mid_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        mid_scroll.setMinimumWidth(200)
        mid_scroll.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        mid_scroll.setStyleSheet("QScrollArea { border: none; }")
        mid_widget = QWidget()
        mid_panel = QVBoxLayout(mid_widget)
        mid_panel.setContentsMargins(4, 4, 4, 4)
        mid_panel.setSpacing(6)
        mid_scroll.setWidget(mid_widget)
        main_layout.addWidget(mid_scroll, 1)
        
        coord_group = QGroupBox("实时坐标")
        c_lay = QVBoxLayout()
        self.status_label = QLabel("坐标: X=75.0, Y=220.0")
        self.status_label.setFont(QFont("Arial", 11, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignCenter)
        c_lay.addWidget(self.status_label)
        self.feedback_pose_label = QLabel("回传末端: X=--, Y=--")
        self.feedback_joint_label = QLabel("回传角度: M1=-- deg, M2=-- deg")
        self.feedback_pulse_label = QLabel("脉冲/PPS: P=--,--  A1=--/--  A2=--/--")
        self.feedback_error_label = QLabel("XY误差: 当前 dX=--, dY=--, |e|=-- mm\nMaxX/MaxY --/-- mm  RMSX/RMSY --/-- mm")
        for label in (self.feedback_pose_label, self.feedback_joint_label, self.feedback_pulse_label, self.feedback_error_label):
            label.setAlignment(Qt.AlignCenter)
            label.setWordWrap(True)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
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
        self.lbl_mcu_gbuf = QLabel("缓冲区占用: 0 / 48")
        self.lbl_mcu_queue = QLabel("队列负载(Q): 0")
        self.lbl_mcu_interp = QLabel("插补: --  已执行 0/0  队列 0")
        self.lbl_mcu_hz = QLabel("控制频率: -- Hz")
        self.lbl_sender_mode = QLabel("Sender: idle")
        for label in (self.lbl_mcu_err, self.lbl_mcu_tick, self.lbl_mcu_gbuf, self.lbl_mcu_queue, self.lbl_mcu_interp, self.lbl_mcu_hz, self.lbl_sender_mode):
            label.setWordWrap(True)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.lbl_mcu_err.setStyleSheet("font-weight: bold; color: #e06c75;")
        self.lbl_mcu_tick.setStyleSheet("color: #61afef;")
        self.lbl_mcu_gbuf.setStyleSheet("color: #98c379;")
        self.lbl_mcu_queue.setStyleSheet("color: #d19a66;")
        self.lbl_mcu_interp.setStyleSheet("color: #c678dd;")
        self.lbl_mcu_hz.setStyleSheet("color: #56b6c2;")
        self.lbl_sender_mode.setStyleSheet("color: #e5c07b;")
        mcu_status_lay.addWidget(self.lbl_mcu_err, 0, 0)
        mcu_status_lay.addWidget(self.lbl_mcu_tick, 0, 1)
        mcu_status_lay.addWidget(self.lbl_mcu_gbuf, 1, 0)
        mcu_status_lay.addWidget(self.lbl_mcu_queue, 1, 1)
        mcu_status_lay.addWidget(self.lbl_mcu_interp, 2, 0)
        mcu_status_lay.addWidget(self.lbl_mcu_hz, 2, 1)
        mcu_status_lay.addWidget(self.lbl_sender_mode, 3, 0, 1, 2)
        mcu_status_group.setLayout(mcu_status_lay)
        mid_panel.addWidget(mcu_status_group)
        
        teach_group = QGroupBox("示教模式")
        t_grid = QGridLayout()
        t_grid.addWidget(QLabel("运动模式:"), 0, 0)
        self.teach_mode_combo = QComboBox()
        self.teach_mode_combo.addItems(["直线模式", "圆弧模式"])
        t_grid.addWidget(self.teach_mode_combo, 0, 1)

        t_grid.addWidget(QLabel("运行方式:"), 1, 0)
        self.teach_run_mode_combo = QComboBox()
        self.teach_run_mode_combo.addItems(["连续", "单步"])
        t_grid.addWidget(self.teach_run_mode_combo, 1, 1)

        t_grid.addWidget(QLabel("目标 X:"), 2, 0)
        self.teach_target_x = QLineEdit(f"{self.cur_x:.2f}")
        t_grid.addWidget(self.teach_target_x, 2, 1)
        t_grid.addWidget(QLabel("目标 Y:"), 3, 0)
        self.teach_target_y = QLineEdit(f"{self.cur_y:.2f}")
        t_grid.addWidget(self.teach_target_y, 3, 1)

        self.teach_points_label = QLabel("已记录 0 个目标点")
        self.teach_points_label.setWordWrap(True)
        self.teach_points_label.setStyleSheet("color: #61afef;")
        t_grid.addWidget(self.teach_points_label, 4, 0, 1, 2)

        self.btn_add_teach_target = QPushButton("添加目标点")
        self.btn_rec_point = QPushButton("记录当前位置")
        self.btn_remove_teach_point = QPushButton("撤销最后点")
        self.btn_clear_point = QPushButton("清空目标点")
        self.btn_preview_teach = QPushButton("预览示教轨迹")
        self.btn_replay = QPushButton("执行示教轨迹")
        self.btn_replay.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold;")
        t_grid.addWidget(self.btn_add_teach_target, 5, 0)
        t_grid.addWidget(self.btn_rec_point, 5, 1)
        t_grid.addWidget(self.btn_remove_teach_point, 6, 0)
        t_grid.addWidget(self.btn_clear_point, 6, 1)
        t_grid.addWidget(self.btn_preview_teach, 7, 0)
        t_grid.addWidget(self.btn_replay, 7, 1)
        teach_group.setLayout(t_grid)
        mid_panel.addWidget(teach_group)
        self.teach_mode_combo.currentTextChanged.connect(self.on_teach_mode_changed)
        self.teach_run_mode_combo.currentTextChanged.connect(self.on_teach_run_mode_changed)
        self.btn_add_teach_target.clicked.connect(self.add_teach_target_point)
        self.btn_rec_point.clicked.connect(self.record_single_point)
        self.btn_remove_teach_point.clicked.connect(self.remove_last_teach_point)
        self.btn_clear_point.clicked.connect(self.clear_teach_points)
        self.btn_preview_teach.clicked.connect(self.preview_teach_path)
        self.btn_replay.clicked.connect(self.start_playback)
        self.on_teach_mode_changed()

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
        self.btn_capture_click = QPushButton("🎯 画面点击捕获")
        self.btn_capture_click.setCheckable(True)
        self.btn_capture_click.setStyleSheet(
            "QPushButton { background-color: #555; color: #ccc; font-weight: bold; padding: 4px; border-radius: 3px; }"
            "QPushButton:checked { background-color: #4CAF50; color: white; }"
        )
        self.btn_capture_click.setToolTip("开启后在摄像头画面上点击来设定目标点")
        v_grid.addWidget(self.btn_capture_click, 9, 0, 1, 6)
        vision_group.setLayout(v_grid)
        mid_panel.addWidget(vision_group)
        
        self.btn_cam_open.clicked.connect(self.start_cameras)
        self.btn_cam_close.clicked.connect(self.stop_cameras)
        self.btn_color_toggle.clicked.connect(self.toggle_color)
        self.btn_edge_toggle.clicked.connect(self.toggle_edge)
        self.color_sel.currentTextChanged.connect(self.update_v_params)
        self.thresh_sel.currentTextChanged.connect(self.update_v_params)
        self.btn_vision_trace.clicked.connect(self.plan_vision_trajectory)
        self.btn_capture_click.toggled.connect(self.on_capture_mode_toggled)

        laser_group = QGroupBox("激光加工")
        laser_grid = QGridLayout()
        self.laser_enable_toggle = QPushButton("激光开启")
        self.laser_enable_toggle.setCheckable(True)
        self.laser_enable_toggle.setChecked(False)
        self.laser_enable_toggle.setToolTip("开启后下一次轨迹或点动允许落笔出光；完成、停止、急停、回零或断连后自动关闭")
        self.laser_enable_toggle.setStyleSheet("background-color: #34495e; color: white; font-weight: bold;")
        self.laser_enable_toggle.toggled.connect(self.on_laser_enable_toggled)
        laser_grid.addWidget(self.laser_enable_toggle, 0, 0, 1, 2)
        laser_grid.addWidget(QLabel("功率:"), 1, 0)
        self.laser_power_input = QDoubleSpinBox()
        self.laser_power_input.setRange(0.1, 5.0)
        self.laser_power_input.setSingleStep(0.1)
        self.laser_power_input.setDecimals(1)
        self.laser_power_input.setValue(1.0)
        self.laser_power_input.setSuffix("%")
        self.laser_power_input.valueChanged.connect(self.on_laser_power_changed)
        laser_grid.addWidget(self.laser_power_input, 1, 1)
        self.laser_status_label = QLabel("下位机状态: 断开")
        self.laser_status_label.setStyleSheet("color: #aaaaaa; font-weight: bold;")
        laser_grid.addWidget(self.laser_status_label, 2, 0, 1, 2)
        laser_group.setLayout(laser_grid)
        mid_panel.addWidget(laser_group)

        task_group = QGroupBox("系统任务与安全")
        task_lay = QVBoxLayout()
        task_lay.setContentsMargins(10, 5, 10, 5) 
        self.btn_reset_home = QPushButton("系统一键复位 (回0点)")
        self.btn_emergency_stop = QPushButton("🛑 紧急停止")
        self.btn_emergency_stop.setText("\u6025\u505c (\u4fdd\u7559\u961f\u5217)")
        self.btn_stop_motion = QPushButton("\u505c\u6b62 (\u6e05\u9664\u961f\u5217)")
        self.btn_stop_motion.setFixedHeight(32)
        self.btn_emergency_stop.setFixedHeight(34)
        self.btn_reset_home.setText("仿真回零")
        self.btn_home_real = QPushButton("实机回零")
        self.btn_reset_home.setFixedHeight(30)
        self.btn_home_real.setFixedHeight(30)
        self.btn_reset_home.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
        self.btn_home_real.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold;")
        self.btn_stop_motion.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold;")
        self.btn_emergency_stop.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
        home_lay = QHBoxLayout()
        home_lay.addWidget(self.btn_reset_home)
        home_lay.addWidget(self.btn_home_real)
        task_lay.addLayout(home_lay)
        task_lay.addWidget(self.btn_stop_motion)
        task_lay.addWidget(self.btn_emergency_stop)
        task_group.setLayout(task_lay)
        mid_panel.addWidget(task_group)
        mid_panel.addStretch()
        self.btn_reset_home.clicked.connect(self.system_reset_simulated)
        self.btn_home_real.clicked.connect(self.system_reset_real)
        self.btn_stop_motion.clicked.connect(self.stop_motion)
        self.btn_emergency_stop.clicked.connect(self.emergency_stop)

        # 右侧面板
        right_widget = QWidget()
        # 右侧仿真列固定宽度；窗口拉伸时只让左/中控制列变化。
        # 若显示器较窄，可把 720 调小到 640；若希望更大仿真图，可调到 800。
        right_widget.setFixedWidth(720)
        right_widget.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        right_panel = QVBoxLayout(right_widget)
        right_panel.setContentsMargins(0, 0, 0, 0)
        right_panel.setSpacing(4)
        main_layout.addWidget(right_widget, 0)
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
        self.log_display.document().setMaximumBlockCount(1200)
        self.log_display.setStyleSheet("background: #1e1e1e; color: #61afef; font-family: Consolas;")
        log_v.addWidget(self.log_display)
        self.btn_clear_log = QPushButton("清空发送数据")
        self.btn_clear_log.clicked.connect(lambda: self.log_display.clear())
        log_v.addWidget(self.btn_clear_log)
        bottom_h.addLayout(log_v, 1)
        right_panel.addLayout(bottom_h, 4)

    # --- 通用工具 ---
