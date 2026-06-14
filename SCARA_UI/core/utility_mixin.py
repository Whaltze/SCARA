import time

import numpy as np


class ScaraUtilityMixin:
    def get_timestamp(self):
        return time.strftime("%H:%M:%S") + f".{int(time.time()*1000)%1000:03d}"

    def send_ascii_line(self, line, tag="manual"):
        if not self.ser or not self.ser.is_open:
            self.log_error("串口未连接")
            return False
        text = line.strip()
        if not text:
            return False
        if text in ("?", "!", "~"):
            self.ser.write(text.encode("ascii"))
            return True
        if hasattr(self, "_line_requires_homing") and self._line_requires_homing(text):
            self.log_error("Manual motion G-code is blocked; use a formal motion action with a verified preview path.")
            return False
        if text[0].upper() in ("G", "M", "$") and hasattr(self, "load_gcode_job"):
            append = bool(getattr(self, "waiting_for_ack", False) or getattr(self, "point_queue", None))
            return self.load_gcode_job([text], append=append)
        self.ser.write((text + "\n").encode('ascii', errors='ignore'))
        self.log_display.append(
            f"<font color='#ffffff'>TX {self.get_timestamp()} [{tag}] line={text}</font>"
        )
        return True

    # --- 五连杆正运动学 ---
    def forward_kinematics(self, q1_deg, q2_deg):
        try:
            q1 = np.radians(q1_deg)
            q2 = np.radians(q2_deg)
            c1x = self.L1 * np.cos(q1)
            c1y = self.L1 * np.sin(q1)
            c2x = self.L0 + self.L1 * np.cos(q2)
            c2y = self.L1 * np.sin(q2)
            dx = c2x - c1x
            dy = c2y - c1y
            d = np.sqrt(dx**2 + dy**2)
            if d > 2 * self.L2 or d < 0.001:
                return None, None
            a = d / 2.0
            h = np.sqrt(max(0, self.L2**2 - a**2))
            x0 = c1x + a * dx / d
            y0 = c1y + a * dy / d
            rx = -dy * (h / d)
            ry = dx * (h / d)
            return x0 + rx, y0 + ry
        except:
            return None, None

    # --- 视觉逻辑 ---
