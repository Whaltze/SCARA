import cv2
import numpy as np

from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap

from .threads import CameraThread


class ScaraVisionMixin:
    def update_v_params(self):
        color = self.color_sel.currentText()
        presets = {
            '红色': (0, 10, 100, 255, 100, 255),
            '黄色': (20, 30, 100, 255, 100, 255),
            '绿色': (35, 85, 100, 255, 100, 255),
            '蓝色': (100, 124, 100, 255, 100, 255)
        }
        p = presets.get(color, (0, 10, 0, 255, 0, 255))
        self.sliders["Hmin"].setValue(p[0])
        self.sliders["Hmax"].setValue(p[1])
        self.sliders["Smin"].setValue(p[2])
        self.sliders["Smax"].setValue(p[3])
        self.sliders["Vmin"].setValue(p[4])
        self.sliders["Vmax"].setValue(p[5])
        self.img_proc_thread.color_to_detect = color

    def sync_hsv_to_thread(self):
        self.img_proc_thread.set_hsv_thresholds(
            self.sliders["Hmin"].value(), self.sliders["Hmax"].value(),
            self.sliders["Smin"].value(), self.sliders["Smax"].value(),
            self.sliders["Vmin"].value(), self.sliders["Vmax"].value()
        )

    def start_cameras(self):
        if self.cam_thread and self.cam_thread.isRunning():
            return
        self.cam_thread = CameraThread(int(self.cam_id_combo.currentText()))
        self.cam_thread.frame_ready.connect(self.update_latest_frame)
        self.cam_thread.frame_ready.connect(self.img_proc_thread.update_frame)
        self.cam_thread.start()
        self.log_display.append("<font color='cyan'>摄像头已启动</font>")

    def update_latest_frame(self, frame):
        self.latest_raw_frame = frame

    def stop_cameras(self):
        if self.cam_thread:
            self.cam_thread.stop()
            self.cam_thread = None
        self.cam_label.clear()
        self.cam_label.setText("摄像头已关闭")
        self.log_display.append("<font color='yellow'>摄像头已关闭</font>")

    def display_camera_frame(self, frame):
        if self.cam_thread is None:
            return
        h, w, ch = frame.shape
        q_img = QImage(frame.data, w, h, w * ch, QImage.Format_RGB888)
        pix = QPixmap.fromImage(q_img)
        if not self.cam_label.size().isEmpty():
            self.cam_label.setPixmap(pix.scaled(self.cam_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def toggle_color(self):
        self.img_proc_thread.color_detection = not self.img_proc_thread.color_detection
        self.btn_color_toggle.setText(f"颜色识别: {'ON' if self.img_proc_thread.color_detection else 'OFF'}")

    def toggle_edge(self):
        self.img_proc_thread.edge_detection = not self.img_proc_thread.edge_detection
        self.btn_edge_toggle.setText(f"边缘检测: {'ON' if self.img_proc_thread.edge_detection else 'OFF'}")

    def pixel_to_robot(self, u, v):
        if self.coord_proc and self.coord_proc.is_calibrated:
            return self.coord_proc.pixel_to_robot(u, v)
        scale = 0.5
        return (u - 320) * scale + 75.0, (240 - v) * scale + 200.0

    def plan_vision_trajectory(self):
        if self.latest_raw_frame is None:
            self.log_error("无画面")
            return
        mask = self.img_proc_thread.get_color_mask(self.latest_raw_frame)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((20, 20), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objs = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 300:
                continue
            (u, v), r = cv2.minEnclosingCircle(cnt)
            objs.append({'centroid': (u, v), 'radius': r})
        
        if not objs:
            self.log_error("未发现目标")
            return
        
        objs.sort(key=lambda o: np.hypot(o['centroid'][0]-320, o['centroid'][1]-480))
        
        route_points = [(self.cur_x, self.cur_y)]
        spd = float(self.hw_speed_input.text())
        
        for obj in objs:
            rx, ry = self.pixel_to_robot(*obj['centroid'])
            route_points.append((rx, ry))

        path = self.generate_polyline_path(route_points, spd, silent_first=True)
        self.load_motion_queue(path)

    # --- 核心算法 ---
