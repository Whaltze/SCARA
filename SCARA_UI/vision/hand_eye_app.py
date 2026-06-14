"""九点标定工具"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
from PySide6.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, 
                               QWidget, QPushButton, QHBoxLayout, QLineEdit, 
                               QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox, QGroupBox)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QImage, QPixmap

# 路径修复：确保能引用到同目录下的 core 文件
from vision.camera_core import CameraProcessor
from vision.coordinate_core import CoordinateProcessor

class HandEyeCalibrationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("九点标定工具 (Hand-Eye Calibration) - 1280x720版")
        self.setGeometry(100, 100, 1280, 800)

        # 1. 初始化核心逻辑
        # 自动加载同目录下的 json 文件
        self.cam_processor = CameraProcessor() 
        self.coord_processor = CoordinateProcessor()
        
        # 数据存储
        self.calibration_pairs = [] # 存储 [{'u':..., 'v':..., 'x':..., 'y':...}]
        self.current_click_pos = None # 当前鼠标点击的像素位置

        # --- UI 布局 ---
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # 左侧：视频显示区域
        self.image_label = QLabel("摄像头启动中...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(960, 540) # 16:9 比例
        self.image_label.setStyleSheet("border: 2px solid #555; background-color: #222;")
        self.image_label.mousePressEvent = self.on_image_click
        
        # 右侧：控制与数据区域
        control_panel = QVBoxLayout()
        
        # 1. 坐标输入区
        input_group = QGroupBox("1. 数据采集 (请点击画面特征点)")
        input_layout = QVBoxLayout()
        
        self.lbl_pixel = QLabel("当前点击像素: 未选择")
        self.lbl_pixel.setStyleSheet("font-weight: bold; color: blue; font-size: 14px;")
        
        coord_layout = QHBoxLayout()
        self.input_robot_x = QLineEdit()
        self.input_robot_x.setPlaceholderText("机械臂 X (mm)")
        self.input_robot_y = QLineEdit()
        self.input_robot_y.setPlaceholderText("机械臂 Y (mm)")
        coord_layout.addWidget(QLabel("X:"))
        coord_layout.addWidget(self.input_robot_x)
        coord_layout.addWidget(QLabel("Y:"))
        coord_layout.addWidget(self.input_robot_y)
        
        self.btn_add_point = QPushButton("记录该点 (Add)")
        self.btn_add_point.clicked.connect(self.add_point)
        self.btn_add_point.setStyleSheet("background-color: #e0f7fa; height: 35px; font-weight: bold;")

        input_layout.addWidget(self.lbl_pixel)
        input_layout.addLayout(coord_layout)
        input_layout.addWidget(self.btn_add_point)
        input_group.setLayout(input_layout)

        # 2. 数据列表
        self.table = QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Pix U", "Pix V", "Robot X", "Robot Y"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        # 3. 操作按钮
        action_layout = QHBoxLayout()
        self.btn_calculate = QPushButton("计算并保存矩阵")
        self.btn_calculate.setStyleSheet("background-color: #d1c4e9; font-weight: bold; height: 45px;")
        self.btn_calculate.clicked.connect(self.calculate_matrix)
        
        self.btn_clear = QPushButton("清空数据")
        self.btn_clear.clicked.connect(self.clear_data)
        
        action_layout.addWidget(self.btn_calculate)
        action_layout.addWidget(self.btn_clear)

        # 4. 测试区域
        test_group = QGroupBox("2. 验证模式")
        test_layout = QVBoxLayout()
        self.test_mode_checkbox = QPushButton("开启实时验证: OFF")
        self.test_mode_checkbox.setCheckable(True)
        self.test_mode_checkbox.toggled.connect(self.toggle_test_mode)
        self.lbl_test_result = QLabel("点击画面任意位置，查看预测坐标")
        
        test_layout.addWidget(self.test_mode_checkbox)
        test_layout.addWidget(self.lbl_test_result)
        test_group.setLayout(test_layout)

        # 组装右侧
        control_panel.addWidget(input_group)
        control_panel.addWidget(self.table)
        control_panel.addLayout(action_layout)
        control_panel.addWidget(test_group)
        
        main_layout.addWidget(self.image_label, stretch=3)
        main_layout.addLayout(control_panel, stretch=1)
        
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # --- 摄像头初始化 (关键：使用高清分辨率匹配标定) ---
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # === 关键步骤：先除畸变 ===
            # 如果 cam_processor 加载参数成功，这里返回的一定是直的图
            if self.cam_processor.is_calibrated:
                self.current_undistorted = self.cam_processor.undistort_image(frame)
                status_text = "畸变矫正: ON"
                color = (0, 255, 0)
            else:
                self.current_undistorted = frame
                status_text = "畸变矫正: OFF (未加载参数)"
                color = (0, 0, 255)
            
            # 拷贝一份用于显示绘制
            display_img = self.current_undistorted.copy()
            
            # 在左上角显示状态
            cv2.putText(display_img, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            # 绘制当前点击位置
            if self.current_click_pos:
                u, v = self.current_click_pos
                # 画个十字
                cv2.drawMarker(display_img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                # 显示坐标数值
                cv2.putText(display_img, f"({u},{v})", (u+10, v-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # 如果在测试模式，显示预测的物理坐标
                if self.test_mode_checkbox.isChecked() and self.coord_processor.is_calibrated:
                    rx, ry = self.coord_processor.pixel_to_robot(u, v)
                    cv2.putText(display_img, f"Robot: X{rx:.1f} Y{ry:.1f}", (u+10, v+20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            self.display_image(display_img)

    def display_image(self, frame):
        # 转换颜色 BGR -> RGB
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        # 缩放以适应 Label 大小 (保持比例)
        scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        self.image_label.setPixmap(scaled_pixmap)
        
        # 保存缩放比例，以便将鼠标点击坐标还原回真实图像坐标
        self.scale_factor_x = w / scaled_pixmap.width()
        self.scale_factor_y = h / scaled_pixmap.height()
        
        # 计算图片在 label 中的偏移量（因为 KeepAspectRatio 可能会留黑边）
        self.offset_x = (self.image_label.width() - scaled_pixmap.width()) / 2
        self.offset_y = (self.image_label.height() - scaled_pixmap.height()) / 2

    def on_image_click(self, event):
        # 获取相对于 Label 的坐标
        click_x = event.pos().x()
        click_y = event.pos().y()
        
        # 还原到真实图像坐标 (1280x720)
        # 1. 减去黑边偏移
        real_x_display = click_x - self.offset_x
        real_y_display = click_y - self.offset_y
        
        # 2. 乘以缩放比例
        if hasattr(self, 'scale_factor_x'):
            final_u = int(real_x_display * self.scale_factor_x)
            final_v = int(real_y_display * self.scale_factor_y)
            
            # 边界检查
            h, w = self.current_undistorted.shape[:2]
            final_u = max(0, min(w-1, final_u))
            final_v = max(0, min(h-1, final_v))
            
            self.current_click_pos = (final_u, final_v)
            self.lbl_pixel.setText(f"当前点击像素: U={final_u}, V={final_v}")
            
            # 如果是验证模式，直接在界面更新预测值
            if self.test_mode_checkbox.isChecked() and self.coord_processor.is_calibrated:
                rx, ry = self.coord_processor.pixel_to_robot(final_u, final_v)
                self.lbl_test_result.setText(f"预测坐标: X={rx:.2f}, Y={ry:.2f}")

    def add_point(self):
        if not self.current_click_pos:
            QMessageBox.warning(self, "提示", "请先在左侧图像上点击一个点")
            return
            
        try:
            rx = float(self.input_robot_x.text())
            ry = float(self.input_robot_y.text())
        except ValueError:
            QMessageBox.warning(self, "错误", "请输入有效的数字坐标")
            return

        u, v = self.current_click_pos
        self.calibration_pairs.append({'u': u, 'v': v, 'x': rx, 'y': ry})
        
        # 更新表格
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(str(u)))
        self.table.setItem(row, 1, QTableWidgetItem(str(v)))
        self.table.setItem(row, 2, QTableWidgetItem(str(rx)))
        self.table.setItem(row, 3, QTableWidgetItem(str(ry)))
        
        # 清空输入框，准备下一次
        self.input_robot_x.clear()
        self.input_robot_y.clear()
        # 焦点回到图片，方便下次点击
        self.image_label.setFocus()

    def calculate_matrix(self):
        if len(self.calibration_pairs) < 4:
            QMessageBox.warning(self, "警告", "透视变换至少需要4个点，建议采集9个点以获得更高精度")
            return
            
        img_pts = [[p['u'], p['v']] for p in self.calibration_pairs]
        rob_pts = [[p['x'], p['y']] for p in self.calibration_pairs]
        
        # 使用新的 coordinate_core (透视变换)
        success, mat = self.coord_processor.calibrate_affine(img_pts, rob_pts)
        
        if success:
            QMessageBox.information(self, "成功", "标定成功！矩阵已保存。\n已自动开启验证模式。")
            self.test_mode_checkbox.setChecked(True)
        else:
            QMessageBox.critical(self, "失败", "计算矩阵失败")

    def clear_data(self):
        self.calibration_pairs = []
        self.table.setRowCount(0)
        self.current_click_pos = None

    def toggle_test_mode(self, checked):
        if checked:
            # 尝试重新加载矩阵，确保用的是最新的
            self.coord_processor.load_matrix()
            if not self.coord_processor.is_calibrated:
                QMessageBox.warning(self, "提示", "请先进行标定或确保矩阵文件存在")
                self.test_mode_checkbox.setChecked(False)
                return
            self.test_mode_checkbox.setText("开启实时验证: ON")
            self.test_mode_checkbox.setStyleSheet("background-color: #a5d6a7; font-weight: bold;")
        else:
            self.test_mode_checkbox.setText("开启实时验证: OFF")
            self.test_mode_checkbox.setStyleSheet("")

    def closeEvent(self, event):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HandEyeCalibrationApp()
    window.show()
    sys.exit(app.exec())
