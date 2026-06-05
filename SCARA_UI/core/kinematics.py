"""五连杆并联 SCARA 运动学模型。

本文件只放机械臂几何计算，不放 UI 和串口逻辑，方便后期按真实机械尺寸修改。
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass
class FiveBarConfig:
    """五连杆机构参数，单位默认 mm / degree。"""

    base_distance: float = 150.0
    active_link: float = 160.0
    passive_link: float = 200.0
    min_y: float = 10.0
    min_anchor_dist: float = 40.0
    max_anchor_dist: float = 360.0
    min_elbow_angle_deg: float = 45.0


class FiveBarKinematics:
    """五连杆并联 SCARA 的正逆运动学。

    当前课程设计阶段，上位机负责高开销几何判断和轨迹遍历，单片机只负责接收 G-code 点流。
    """

    def __init__(self, config: Optional[FiveBarConfig] = None):
        self.config = config or FiveBarConfig()

    def inverse(self, x: float, y: float) -> Tuple[Optional[float], Optional[float]]:
        """由末端 XY 求两个主动臂角度，失败时返回 (None, None)。"""
        c = self.config
        if y < c.min_y:
            return None, None

        d1 = math.hypot(x, y)
        d2 = math.hypot(x - c.base_distance, y)
        if (
            d1 > c.max_anchor_dist
            or d1 < c.min_anchor_dist
            or d2 > c.max_anchor_dist
            or d2 < c.min_anchor_dist
        ):
            return None, None

        try:
            cos_a1 = self._clamp(
                (c.active_link**2 + c.passive_link**2 - d1**2)
                / (2.0 * c.active_link * c.passive_link),
                -1.0,
                1.0,
            )
            cos_a2 = self._clamp(
                (c.active_link**2 + c.passive_link**2 - d2**2)
                / (2.0 * c.active_link * c.passive_link),
                -1.0,
                1.0,
            )

            if (
                math.degrees(math.acos(cos_a1)) < c.min_elbow_angle_deg
                or math.degrees(math.acos(cos_a2)) < c.min_elbow_angle_deg
            ):
                return None, None

            q1 = math.atan2(y, x) + math.acos(
                self._clamp(
                    (c.active_link**2 + d1**2 - c.passive_link**2)
                    / (2.0 * c.active_link * d1),
                    -1.0,
                    1.0,
                )
            )
            q2 = math.atan2(y, x - c.base_distance) - math.acos(
                self._clamp(
                    (c.active_link**2 + d2**2 - c.passive_link**2)
                    / (2.0 * c.active_link * d2),
                    -1.0,
                    1.0,
                )
            )
            return math.degrees(q1), math.degrees(q2)
        except (ValueError, ZeroDivisionError):
            return None, None

    def forward(self, q1_deg: float, q2_deg: float) -> Tuple[Optional[float], Optional[float]]:
        """由两个主动臂角度求末端 XY，主要用于仿真显示和后期标定检查。"""
        c = self.config
        q1 = math.radians(q1_deg)
        q2 = math.radians(q2_deg)
        c1x = c.active_link * math.cos(q1)
        c1y = c.active_link * math.sin(q1)
        c2x = c.base_distance + c.active_link * math.cos(q2)
        c2y = c.active_link * math.sin(q2)

        dx = c2x - c1x
        dy = c2y - c1y
        d = math.hypot(dx, dy)
        if d <= 1e-6 or d > 2.0 * c.passive_link:
            return None, None

        mid_x = (c1x + c2x) * 0.5
        mid_y = (c1y + c2y) * 0.5
        h = math.sqrt(max(0.0, c.passive_link**2 - (d * 0.5) ** 2))
        ux = -dy / d
        uy = dx / d
        ix1 = mid_x + ux * h
        iy1 = mid_y + uy * h
        ix2 = mid_x - ux * h
        iy2 = mid_y - uy * h
        # 与下位机保持一致：五连杆末端取两个圆交点中 Y 更高的一支。
        return (ix1, iy1) if iy1 >= iy2 else (ix2, iy2)

    def is_reachable(self, x: float, y: float, margin: float = 5.0) -> bool:
        """带安全边界的工作空间判断。"""
        c = self.config
        d1 = math.hypot(x, y)
        d2 = math.hypot(x - c.base_distance, y)
        if (
            y < c.min_y + margin
            or d1 > c.max_anchor_dist - margin
            or d2 > c.max_anchor_dist - margin
            or d1 < c.min_anchor_dist + margin
            or d2 < c.min_anchor_dist + margin
        ):
            return False
        return self.inverse(x, y)[0] is not None

    def find_safe_home(self, preferred: Tuple[float, float]) -> Tuple[float, float]:
        """如果配置的 HOME 点不可达，自动寻找一个靠近中间区域的安全点。"""
        if self.has_jog_neighborhood(preferred[0], preferred[1], jog_mm=10.0):
            return preferred

        candidates = []
        for y in range(260, 60, -10):
            for x in range(-40, int(self.config.base_distance + 41), 10):
                if self.has_jog_neighborhood(float(x), float(y), jog_mm=10.0):
                    score = abs(x - self.config.base_distance * 0.5) + abs(y - 220)
                    candidates.append((score, float(x), float(y)))

        if not candidates:
            return preferred
        _, x, y = min(candidates, key=lambda item: item[0])
        return x, y

    def has_jog_neighborhood(self, x: float, y: float, jog_mm: float = 10.0) -> bool:
        """检查当前位置及四方向点动目标是否都在工作空间内。"""
        checks = [
            (x, y),
            (x, y + jog_mm),
            (x, y - jog_mm),
            (x - jog_mm, y),
            (x + jog_mm, y),
        ]
        return all(self.is_reachable(px, py, margin=5.0) for px, py in checks)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))
