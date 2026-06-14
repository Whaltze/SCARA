"""Static check: car trajectories emit an exact-stop G4 dwell after every
line/arc, while the line/arc commands themselves stay identical and all other
callers (default exact_stop_dwell_ms=0.0) are unchanged.

Run with the project's Python (needs numpy; no Qt needed):
  <python> _tmp_verify_car_exactstop.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from SCARA_UI.motion.motion_mixin import ScaraMotionMixin
from SCARA_UI.trajectory.look_ahead import LookAheadPlanner


class Stub(ScaraMotionMixin):
    """Minimal harness: only the send-path geometry, no Qt widgets."""

    def __init__(self):
        self.path_planner = LookAheadPlanner(accel_mm_s2=100.0, junction_deviation=0.02)
        self.cur_x = 75.0
        self.cur_y = 200.0

    def log_error(self, msg):
        raise AssertionError(f"unexpected log_error: {msg}")
    # no ui_to_mcu_xy -> mcu_xy() falls back to raw coords (fine for structure)


def check(shape_builder, label):
    s = Stub()
    segs = shape_builder(s, 75.0, 200.0)
    dwell = f"G4 P{s.CAR_EXACT_STOP_DWELL_MS / 1000.0:.3f}"
    g_with = s.generate_geometry_gcode(segs, 20.0, start=(75.0, 200.0),
                                       exact_stop_dwell_ms=s.CAR_EXACT_STOP_DWELL_MS)
    g_without = s.generate_geometry_gcode(segs, 20.0, start=(75.0, 200.0))

    # 1) default path never inserts the exact-stop dwell
    assert not any(c == dwell for c in g_without), f"{label}: default must not insert {dwell}"

    # 2) every motion command is immediately followed by the exact-stop dwell
    for idx, c in enumerate(g_with):
        if c[:2] in ("G1", "G2", "G3"):
            nxt = g_with[idx + 1] if idx + 1 < len(g_with) else ""
            assert nxt == dwell, f"{label}: line {idx} {c!r} not followed by {dwell}, got {nxt!r}"

    # 3) removing the appended dwells reproduces the unchanged command stream
    stripped = [c for c in g_with if c != dwell]
    assert stripped == g_without, f"{label}: non-dwell lines diverged from baseline"

    n_motion = sum(1 for c in g_without if c[:2] in ("G1", "G2", "G3"))
    n_dwell = sum(1 for c in g_with if c == dwell)
    assert n_dwell == n_motion, f"{label}: dwell count {n_dwell} != motion count {n_motion}"
    print(f"{label}: segments={len(segs)} motion_cmds={n_motion} dwells={n_dwell} "
          f"with={len(g_with)} without={len(g_without)}  OK")
    return g_with


g = check(lambda s, x, y: s.build_car1_segments(x, y), "car1")
check(lambda s, x, y: s.build_car2_segments(x, y), "car2")

print("\ncar1 first 10 commands:")
for c in g[:10]:
    print("  ", c)
print("\nALL CHECKS PASSED")
