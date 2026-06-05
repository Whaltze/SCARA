# E:\v1\SCARA-main\SCARA_UI\motor_monitor\test_monitor.py
import time
import re
import numpy as np
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PySide6.QtCore import Slot
import pyqtgraph as pg

class MonitorWindow(QMainWindow):
    def __init__(self, kinematics_engine):
        super().__init__()
        self.setWindowTitle("电机动力学实时监控 (已开启平滑滤波)")
        self.resize(1000, 800)
        
        self.kin = kinematics_engine
        self.is_running = True
        
        # 数据缓存
        self.time_history = []
        self.v1_data, self.v2_data = [], []
        self.a1_data, self.a2_data = [], []
        
        # 滤波状态变量 (指数移动平均滤波)
        self.alpha_v = 0.2  # 速度平滑系数 (0-1, 越小越平滑)
        self.alpha_a = 0.1  # 加速度平滑系数
        self.last_v_filt = [0.0, 0.0]
        self.last_a_filt = [0.0, 0.0]
        
        # 运动学记录
        self.last_pos = None # (x, y) 用于检测重复数据
        self.last_theta = None # (t1, t2)
        self.last_time = time.perf_counter()
        self.start_timestamp = time.perf_counter()

        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        btn_layout = QHBoxLayout()
        self.btn_pause = QPushButton("开始/暂停显示")
        self.btn_clear = QPushButton("清空图像")
        btn_layout.addWidget(self.btn_pause); btn_layout.addWidget(self.btn_clear)
        layout.addLayout(btn_layout)

        self.view = pg.GraphicsLayoutWidget()
        layout.addWidget(self.view)
        
        # 改善配色
        p1_pen = pg.mkPen(color='#00FF00', width=2)
        p2_pen = pg.mkPen(color='#00FFFF', width=2)

        self.p_v1 = self.view.addPlot(title="电机 1 速度 (deg/s)")
        self.cv1 = self.p_v1.plot(pen=p1_pen)
        self.p_v2 = self.view.addPlot(title="电机 2 速度 (deg/s)")
        self.cv2 = self.p_v2.plot(pen=p2_pen)
        self.view.nextRow()
        self.p_a1 = self.view.addPlot(title="电机 1 加速度 (deg/s²)")
        self.ca1 = self.p_a1.plot(pen=p1_pen)
        self.p_a2 = self.view.addPlot(title="电机 2 加速度 (deg/s²)")
        self.ca2 = self.p_a2.plot(pen=p2_pen)

        self.btn_pause.clicked.connect(lambda: setattr(self, 'is_running', not self.is_running))
        self.btn_clear.clicked.connect(self.clear_all)

    def clear_all(self):
        self.time_history.clear()
        self.v1_data.clear(); self.v2_data.clear()
        self.a1_data.clear(); self.a2_data.clear()
        self.cv1.setData([], []); self.cv2.setData([], [])
        self.ca1.setData([], []); self.ca2.setData([], [])

    @Slot(str)
    def process_new_data(self, raw_str):
        if not self.is_running: return
        try:
            x_m = re.search(r"X([-+]?\d*\.\d+|\d+)", raw_str)
            y_m = re.search(r"Y([-+]?\d*\.\d+|\d+)", raw_str)
            
            if x_m and y_m:
                curr_x, curr_y = float(x_m.group(1)), float(y_m.group(1))
                
                # --- 核心修复 1: 重复数据拦截 ---
                # 如果坐标和上一次完全一样，说明是重复解析同一行，直接拦截，防止速度跌落至0
                if self.last_pos == (curr_x, curr_y):
                    return
                
                # 逆解
                res = self.kin.inverse(curr_x, curr_y)
                if res[0] is None: return
                t1, t2 = res[0], res[1]

                now = time.perf_counter()
                dt = now - self.last_time
                # 限制 dt 防止抖动 (最小 20ms)
                if dt < 0.02: return 
                
                if self.last_theta is not None:
                    # 1. 计算原始速度
                    raw_v1 = (t1 - self.last_theta[0]) / dt
                    raw_v2 = (t2 - self.last_theta[1]) / dt
                    
                    # --- 核心修复 2: 速度低通滤波 ---
                    v1_f = self.alpha_v * raw_v1 + (1 - self.alpha_v) * self.last_v_filt[0]
                    v2_f = self.alpha_v * raw_v2 + (1 - self.alpha_v) * self.last_v_filt[1]
                    
                    # 2. 计算原始加速度
                    raw_a1 = (v1_f - self.last_v_filt[0]) / dt
                    raw_a2 = (v2_f - self.last_v_filt[1]) / dt
                    
                    # --- 核心修复 3: 加速度低通滤波 ---
                    a1_f = self.alpha_a * raw_a1 + (1 - self.alpha_a) * self.last_a_filt[0]
                    a2_f = self.alpha_a * raw_a2 + (1 - self.alpha_a) * self.last_a_filt[1]
                    
                    # 存储平滑后的数据
                    self.time_history.append(now - self.start_timestamp)
                    self.v1_data.append(v1_f); self.v2_data.append(v2_f)
                    self.a1_data.append(a1_f); self.a2_data.append(a2_f)
                    
                    # 更新状态
                    self.last_v_filt = [v1_f, v2_f]
                    self.last_a_filt = [a1_f, a2_f]
                    
                    if len(self.time_history) > 500:
                        for arr in [self.time_history, self.v1_data, self.v2_data, self.a1_data, self.a2_data]: arr.pop(0)
                    self.update_plots()

                self.last_pos = (curr_x, curr_y)
                self.last_theta = (t1, t2)
                self.last_time = now
        except: pass

    def update_plots(self):
        self.cv1.setData(self.time_history, self.v1_data)
        self.cv2.setData(self.time_history, self.v2_data)
        self.ca1.setData(self.time_history, self.a1_data)
        self.ca2.setData(self.time_history, self.a2_data)