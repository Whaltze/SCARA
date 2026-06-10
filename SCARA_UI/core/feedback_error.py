"""Expected-vs-feedback trajectory error metrics in millimeters."""

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple


Point2D = Tuple[float, float]


@dataclass
class FeedbackErrorSample:
    feedback_x: float
    feedback_y: float
    expected_x: float
    expected_y: float
    err_x: float
    err_y: float
    err_norm: float
    segment_index: int


@dataclass
class FeedbackErrorStats:
    count: int = 0
    max_abs_x: float = 0.0
    max_abs_y: float = 0.0
    max_norm: float = 0.0
    rms_x: float = 0.0
    rms_y: float = 0.0
    rms_norm: float = 0.0
    last: Optional[FeedbackErrorSample] = None


class FeedbackErrorTracker:
    SEARCH_BACKTRACK_SEGMENTS = 8
    SEARCH_FORWARD_SEGMENTS = 160

    def __init__(self):
        self.expected: List[Point2D] = []
        self.samples: List[FeedbackErrorSample] = []
        self._sum_x2 = 0.0
        self._sum_y2 = 0.0
        self._sum_norm2 = 0.0
        self._last_segment_hint = 0

    def set_expected_path(self, path: Iterable[Sequence[float]]) -> None:
        self.expected = [(float(p[0]), float(p[1])) for p in path]
        self.clear_samples()

    def clear_samples(self) -> None:
        self.samples = []
        self._sum_x2 = 0.0
        self._sum_y2 = 0.0
        self._sum_norm2 = 0.0
        self._last_segment_hint = 0

    def add_feedback(self, x: float, y: float) -> Optional[FeedbackErrorSample]:
        if not self.expected:
            return None

        expected_x, expected_y, segment_index = self._nearest_expected_point(float(x), float(y))
        err_x = float(x) - expected_x
        err_y = float(y) - expected_y
        err_norm = math.hypot(err_x, err_y)
        sample = FeedbackErrorSample(
            feedback_x=float(x),
            feedback_y=float(y),
            expected_x=expected_x,
            expected_y=expected_y,
            err_x=err_x,
            err_y=err_y,
            err_norm=err_norm,
            segment_index=segment_index,
        )
        self.samples.append(sample)
        self._sum_x2 += err_x * err_x
        self._sum_y2 += err_y * err_y
        self._sum_norm2 += err_norm * err_norm
        if len(self.samples) > 10000:
            self._rebuild_tail()
        return sample

    def stats(self) -> FeedbackErrorStats:
        if not self.samples:
            return FeedbackErrorStats()
        count = len(self.samples)
        max_abs_x = max(abs(sample.err_x) for sample in self.samples)
        max_abs_y = max(abs(sample.err_y) for sample in self.samples)
        max_norm = max(sample.err_norm for sample in self.samples)
        return FeedbackErrorStats(
            count=count,
            max_abs_x=max_abs_x,
            max_abs_y=max_abs_y,
            max_norm=max_norm,
            rms_x=math.sqrt(self._sum_x2 / count),
            rms_y=math.sqrt(self._sum_y2 / count),
            rms_norm=math.sqrt(self._sum_norm2 / count),
            last=self.samples[-1],
        )

    def _nearest_expected_point(self, x: float, y: float) -> Tuple[float, float, int]:
        if len(self.expected) == 1:
            return self.expected[0][0], self.expected[0][1], 0

        best = None
        segment_count = len(self.expected) - 1
        start = max(0, self._last_segment_hint - self.SEARCH_BACKTRACK_SEGMENTS)
        end = min(segment_count, self._last_segment_hint + self.SEARCH_FORWARD_SEGMENTS)

        for index in range(start, end + 1):
            ax, ay = self.expected[index]
            bx, by = self.expected[min(index + 1, len(self.expected) - 1)]
            px, py = _project_point_to_segment(x, y, ax, ay, bx, by)
            dist2 = (x - px) * (x - px) + (y - py) * (y - py)
            if best is None or dist2 < best[0]:
                best = (dist2, px, py, index)

        if best is None:
            ax, ay = self.expected[-1]
            return ax, ay, len(self.expected) - 1
        self._last_segment_hint = best[3]
        return best[1], best[2], best[3]

    def _rebuild_tail(self) -> None:
        self.samples = self.samples[-5000:]
        self._sum_x2 = sum(sample.err_x * sample.err_x for sample in self.samples)
        self._sum_y2 = sum(sample.err_y * sample.err_y for sample in self.samples)
        self._sum_norm2 = sum(sample.err_norm * sample.err_norm for sample in self.samples)


def _project_point_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> Point2D:
    vx = bx - ax
    vy = by - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return ax, ay
    t = ((px - ax) * vx + (py - ay) * vy) / denom
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    return ax + vx * t, ay + vy * t
