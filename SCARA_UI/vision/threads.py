import numpy as np
import cv2

from PySide6.QtCore import QThread, Signal


class CameraThread(QThread):
    frame_ready = Signal(object)
    
    def __init__(self, camera_id=0):
        super().__init__()
        self.camera_id = camera_id
        self.running = False
        self.cap = None

    def run(self):
        self.running = True
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame_ready.emit(frame)
            else:
                self.msleep(100)
            self.msleep(30)
        if self.cap:
            self.cap.release()

    def stop(self):
        self.running = False
        self.wait()

class ImageProcessingThread(QThread):
    processed_frame_ready = Signal(object)
    
    def __init__(self, cam_proc):
        super().__init__()
        self.cam_proc = cam_proc
        self.current_frame = None
        self.running = True
        self.color_detection = True
        self.edge_detection = False
        self.color_to_detect = '红色'
        
        self.h_min, self.s_min, self.v_min = 0, 100, 100
        self.h_max, self.s_max, self.v_max = 10, 255, 255

    def run(self):
        while self.running:
            if self.current_frame is not None:
                frame = self.current_frame.copy()
                processed = self.process_image(frame)
                self.processed_frame_ready.emit(processed)
                self.current_frame = None
            self.msleep(10)

    def update_frame(self, frame):
        self.current_frame = frame

    def set_hsv_thresholds(self, hmin, hmax, smin, smax, vmin, vmax):
        self.h_min, self.h_max = hmin, hmax
        self.s_min, self.s_max = smin, smax
        self.v_min, self.v_max = vmin, vmax

    def get_color_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, self.s_max, self.v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        if self.color_to_detect == '红色' and self.h_min < 10:
            mask2 = cv2.inRange(hsv, np.array([160, self.s_min, self.v_min]), np.array([180, self.s_max, self.v_max]))
            mask = cv2.bitwise_or(mask, mask2)
        return mask

    def process_image(self, frame):
        if self.cam_proc and self.cam_proc.is_calibrated:
            frame = self.cam_proc.undistort_image(frame)
        frame_rgb = cv2.cvtColor(frame, cv2.BGR2RGB if hasattr(cv2, 'BGR2RGB') else cv2.COLOR_BGR2RGB)
        if self.color_detection:
            mask = self.get_color_mask(frame)
            cts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cts:
                area = cv2.contourArea(c)
                if area > 300:
                    cv2.drawContours(frame_rgb, [c], -1, (0, 255, 0), 2)
                    # 几何中心
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.drawMarker(frame_rgb, (cx, cy), (0, 0, 255),
                                       cv2.MARKER_CROSS, 10, 2)
                        cv2.putText(frame_rgb, f"({cx},{cy})", (cx + 10, cy - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                    (255, 255, 255), 1, cv2.LINE_AA)
        if self.edge_detection:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 200)
            edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
            frame_rgb = cv2.addWeighted(frame_rgb, 0.7, edges_rgb, 0.3, 0)
        return frame_rgb

# ========================================================================================
#   主程序类
# ========================================================================================
