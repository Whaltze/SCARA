"""上位机轻量 GRBL 风格轨迹规划器。

借鉴 GRBL planner 的思想：先把线段放进规划块，再做 junction 限速、反向减速传播、
正向加速传播。这里没有复制 GRBL 源码，只在 Python 上位机中实现适合课程设计的版本。
"""

from dataclasses import dataclass
import math
from typing import List, Tuple


@dataclass
class PlannerPoint:
    """发送给下位机的一条 G1 目标点。"""

    x: float
    y: float
    feed_mm_min: float
    silent: bool = False


@dataclass
class PlannerBlock:
    """规划缓冲中的一段直线。"""

    start: Tuple[float, float]
    end: Tuple[float, float]
    feed_mm_s: float
    unit: Tuple[float, float]
    length: float
    max_entry_speed: float = 0.0
    entry_speed: float = 0.0
    exit_speed: float = 0.0


class LookAheadPlanner:
    """小型 look-ahead 规划器，输出带速度 F 的 G1 点流。"""

    def __init__(self, accel_mm_s2: float = 100.0, junction_deviation: float = 0.02, sample_dt: float = 0.04):
        self.accel_mm_s2 = max(1.0, accel_mm_s2)
        self.junction_deviation = max(0.001, junction_deviation)
        self.sample_dt = max(0.005, sample_dt)

    def plan_polyline(
        self,
        points: List[Tuple[float, float]],
        feed_mm_s: float,
        start_speed: float = 0.0,
        end_speed: float = 0.0,
        silent_first: bool = False,
    ) -> List[PlannerPoint]:
        """规划一条折线，返回离散 G1 点流。

        points 至少包含起点和终点；feed_mm_s 为上位机期望末端速度。
        """
        blocks = self._build_blocks(points, feed_mm_s)
        if not blocks:
            return []

        self._compute_junction_limits(blocks, start_speed, end_speed)
        self._reverse_pass(blocks, end_speed)
        self._forward_pass(blocks, start_speed)

        planned = []  # type: List[PlannerPoint]
        for index, block in enumerate(blocks):
            block.exit_speed = blocks[index + 1].entry_speed if index + 1 < len(blocks) else end_speed
            planned.extend(self._sample_block(block, silent=(silent_first and index == 0)))
        return planned

    def plan_line(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
        feed_mm_s: float,
        start_speed: float = 0.0,
        end_speed: float = 0.0,
        silent: bool = False,
    ) -> List[PlannerPoint]:
        """规划单条直线。"""
        return self.plan_polyline([start, end], feed_mm_s, start_speed, end_speed, silent_first=silent)

    def _build_blocks(self, points: List[Tuple[float, float]], feed_mm_s: float) -> List[PlannerBlock]:
        blocks = []  # type: List[PlannerBlock]
        for p0, p1 in zip(points, points[1:]):
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            length = math.hypot(dx, dy)
            if length < 0.001:
                continue
            blocks.append(
                PlannerBlock(
                    start=p0,
                    end=p1,
                    feed_mm_s=max(0.1, feed_mm_s),
                    unit=(dx / length, dy / length),
                    length=length,
                )
            )
        return blocks

    def _compute_junction_limits(self, blocks: List[PlannerBlock], start_speed: float, end_speed: float) -> None:
        blocks[0].max_entry_speed = min(blocks[0].feed_mm_s, start_speed)
        for index in range(1, len(blocks)):
            prev_block = blocks[index - 1]
            block = blocks[index]
            dot = self._clamp(prev_block.unit[0] * block.unit[0] + prev_block.unit[1] * block.unit[1], -0.999999, 0.999999)
            # GRBL 风格 junction_deviation：方向越接近，允许越高；急转角自动降速。
            sin_theta_half = math.sqrt(max(0.0, 0.5 * (1.0 - dot)))
            if sin_theta_half < 1e-6:
                junction_speed = min(prev_block.feed_mm_s, block.feed_mm_s)
            else:
                junction_speed = math.sqrt(
                    max(
                        0.0,
                        self.accel_mm_s2 * self.junction_deviation * sin_theta_half / max(1e-6, 1.0 - sin_theta_half),
                    )
                )
            block.max_entry_speed = min(prev_block.feed_mm_s, block.feed_mm_s, junction_speed)
        blocks[-1].max_entry_speed = min(blocks[-1].max_entry_speed, blocks[-1].feed_mm_s)
        blocks[-1].exit_speed = min(end_speed, blocks[-1].feed_mm_s)

    def _reverse_pass(self, blocks: List[PlannerBlock], end_speed: float) -> None:
        next_entry = min(end_speed, blocks[-1].feed_mm_s)
        for block in reversed(blocks):
            allowed = math.sqrt(max(0.0, next_entry * next_entry + 2.0 * self.accel_mm_s2 * block.length))
            block.entry_speed = min(block.max_entry_speed, block.feed_mm_s, allowed)
            next_entry = block.entry_speed

    def _forward_pass(self, blocks: List[PlannerBlock], start_speed: float) -> None:
        prev_entry = min(start_speed, blocks[0].feed_mm_s)
        blocks[0].entry_speed = min(blocks[0].entry_speed, prev_entry)
        for index in range(1, len(blocks)):
            prev = blocks[index - 1]
            block = blocks[index]
            allowed = math.sqrt(max(0.0, prev.entry_speed * prev.entry_speed + 2.0 * self.accel_mm_s2 * prev.length))
            block.entry_speed = min(block.entry_speed, allowed)

    def _sample_block(self, block: PlannerBlock, silent: bool) -> List[PlannerPoint]:
        path = []  # type: List[PlannerPoint]
        distance = 0.0
        while distance < block.length:
            speed = self._speed_at_distance(block, distance)
            step = max(0.002, speed * self.sample_dt)
            distance = min(block.length, distance + step)
            ratio = distance / block.length
            x = block.start[0] + (block.end[0] - block.start[0]) * ratio
            y = block.start[1] + (block.end[1] - block.start[1]) * ratio
            path.append(PlannerPoint(x=x, y=y, feed_mm_min=max(1.0, speed * 1000.0), silent=silent))
            if step <= 0.002 and len(path) > 20000:
                break
        return path

    def _speed_at_distance(self, block: PlannerBlock, distance: float) -> float:
        accel_speed = math.sqrt(max(0.0, block.entry_speed**2 + 2.0 * self.accel_mm_s2 * distance))
        decel_speed = math.sqrt(max(0.0, block.exit_speed**2 + 2.0 * self.accel_mm_s2 * (block.length - distance)))
        return max(0.1, min(block.feed_mm_s, accel_speed, decel_speed))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))
