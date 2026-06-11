import cv2
import numpy as np
from collections import deque

from PySide6.QtCore import Qt, QEvent
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
        self._ensure_click_capture_setup()

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
        # 点击捕获模式下绘制标记
        if getattr(self, "_capture_mode", False) and getattr(self, "_click_points", None):
            frame = self.draw_click_markers(frame)
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
        send_path = self.generate_binary_send_from_path(path, spd)
        self.load_motion_queue(path, send_path=send_path)


    # -------------------------------------------------------
    #  画面点击捕获
    # -------------------------------------------------------

    def _ensure_click_capture_setup(self):
        """确保 cam_label 已安装事件过滤器。"""
        if getattr(self, "_click_capture_ready", False):
            return
        if not hasattr(self, "_capture_mode"):
            self._capture_mode = False
        if not hasattr(self, "_click_points"):
            self._click_points = deque(maxlen=5)
        self.cam_label.installEventFilter(self)
        self.cam_label.setMouseTracking(True)
        self._click_capture_ready = True

    def eventFilter(self, obj, event):
        """拦截 cam_label 鼠标点击事件。"""
        if obj is self.cam_label and event.type() == QEvent.MouseButtonPress:
            if event.button() == Qt.LeftButton and getattr(self, "_capture_mode", False):
                label_x, label_y = event.position().x(), event.position().y()
                frame_x, frame_y = self._label_to_frame(label_x, label_y)
                if frame_x < 0 or frame_y < 0:
                    return super().eventFilter(obj, event)

                rx, ry = self.pixel_to_robot(frame_x, frame_y)

                # 工作空间校验
                if not self.check_workspace_safety(rx, ry):
                    self.log_display.append(
                        "<font color='red'>⚠ 超出工作空间: "
                        f"像素=({frame_x},{frame_y})  "
                        f"机械臂=({rx:.1f},{ry:.1f}) — 未记录未执行</font>"
                    )
                    return super().eventFilter(obj, event)

                # 有效点：记录并立即执行单点移动
                self._click_points.append((frame_x, frame_y))
                self.log_display.append(
                    "<font color='#4CAF50'>🎯 目标点: "
                    f"像素=({frame_x},{frame_y})  "
                    f"机械臂=({rx:.1f},{ry:.1f})  → 执行移动</font>"
                )
                self._move_to_point(rx, ry)
        return super().eventFilter(obj, event)

    def _label_to_frame(self, label_x, label_y):
        """将 QLabel 上的像素坐标转换为摄像头原始帧坐标。

        处理 KeepAspectRatio 居中显示的 letterbox 情况。
        返回 (-1, -1) 表示点击在黑边区域。
        """
        raw = getattr(self, "latest_raw_frame", None)
        if raw is None:
            return (-1, -1)
        fh, fw = raw.shape[:2]
        lw = self.cam_label.width()
        lh = self.cam_label.height()
        if lw <= 0 or lh <= 0:
            return (-1, -1)

        # KeepAspectRatio letterbox 换算
        scale = min(lw / fw, lh / fh)
        disp_w = int(fw * scale)
        disp_h = int(fh * scale)
        offset_x = (lw - disp_w) // 2
        offset_y = (lh - disp_h) // 2

        rel_x = label_x - offset_x
        rel_y = label_y - offset_y
        if rel_x < 0 or rel_y < 0 or rel_x >= disp_w or rel_y >= disp_h:
            return (-1, -1)

        fx = int(rel_x / scale)
        fy = int(rel_y / scale)
        return (max(0, min(fx, fw - 1)), max(0, min(fy, fh - 1)))

    def on_capture_mode_toggled(self, checked):
        """由 ui_mixin 按钮切换时调用。"""
        self._capture_mode = checked
        if checked:
            self._click_points.clear()
            self.cam_label.setCursor(Qt.CrossCursor)
        else:
            self.cam_label.setCursor(Qt.ArrowCursor)

    def _move_to_point(self, rx, ry):
        """单点即移：从当前位置生成轨迹并装入运动队列。"""
        route_points = [(self.cur_x, self.cur_y), (rx, ry)]
        spd = float(self.hw_speed_input.text()) if hasattr(self, "hw_speed_input") else 80.0
        path = self.generate_polyline_path(route_points, spd)
        if not path:
            self.log_error("单点轨迹生成失败")
            return
        send_path = self.generate_binary_send_from_path(path, spd)
        self.load_motion_queue(path, send_path=send_path)

    def plan_click_trajectory(self):
        """根据已捕获的点击点规划运动轨迹。

        在相邻点击点之间线性插值，生成平滑轨迹。
        返回: [(x1,y1), (x2,y2), ...] 机械臂坐标列表
        """
        pts = list(self._click_points)
        if not pts:
            return []

        # 转换为机械臂坐标
        robot_pts = [self.pixel_to_robot(x, y) for (x, y) in pts]

        if len(robot_pts) == 1:
            return robot_pts

        spd = float(self.hw_speed_input.text()) if hasattr(self, "hw_speed_input") else 80.0

        trajectory = [robot_pts[0]]
        for i in range(1, len(robot_pts)):
            x0, y0 = robot_pts[i - 1]
            x1, y1 = robot_pts[i]
            dist = np.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(dist / 1.5))
            for t in range(1, steps + 1):
                alpha = t / steps
                xi = x0 + (x1 - x0) * alpha
                yi = y0 + (y1 - y0) * alpha
                trajectory.append((xi, yi))

        self.log_display.append(
            "<font color='#FFA500'>🧮 点击轨迹已规划: "
            f"{len(robot_pts)}个点 → {len(trajectory)}步</font>"
        )

        # 自动装载轨迹到运动队列
        route_points = [(self.cur_x, self.cur_y)] + robot_pts
        path = self.generate_polyline_path(route_points, spd, silent_first=True)
        send_path = self.generate_binary_send_from_path(path, spd)
        self.load_motion_queue(path, send_path=send_path)

        return trajectory

    def draw_click_markers(self, frame):
        """在帧上绘制点击标记和轨迹线。

        参数:
            frame: BGR numpy 数组
        返回:
            绘制后的帧副本
        """
        out = frame.copy()
        pts = list(self._click_points)

        # 绘制点击标记圆圈
        for x, y in pts:
            cv2.circle(out, (x, y), 8, (0, 255, 0), -1)
            cv2.circle(out, (x, y), 10, (255, 255, 255), 1)

        # 绘制轨迹线
        if len(pts) >= 2:
            for i in range(1, len(pts)):
                cv2.line(out, pts[i - 1], pts[i], (255, 165, 0), 2)

        # 标注序号
        for idx, (x, y) in enumerate(pts):
            cv2.putText(out, str(idx + 1), (x + 12, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)

        return out

    # --- 核心算法 ---
