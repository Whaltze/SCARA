import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


kinematics_mod = load_module("kinematics", "SCARA_UI/core/kinematics.py")
planner_mod = load_module("look_ahead", "SCARA_UI/trajectory/look_ahead.py")
motion_mod = load_module("motion_mixin", "SCARA_UI/motion/motion_mixin.py")
protocol_mod = load_module("serial_protocol", "SCARA_UI/communication/serial_protocol.py")


class DummyLog:
    def __init__(self):
        self.items = []

    def append(self, item):
        self.items.append(item)


class DummyUi(motion_mod.ScaraMotionMixin):
    def __init__(self):
        self.L0, self.L1, self.L2 = 150.0, 160.0, 200.0
        self.HOME_X, self.HOME_Y = 75.0, 220.0
        self.cur_x, self.cur_y = 75.0, 220.0
        self.kinematics = kinematics_mod.FiveBarKinematics()
        self.path_planner = planner_mod.LookAheadPlanner(accel_mm_s2=100.0, junction_deviation=0.02, sample_dt=0.02)
        self.errors = []
        self.log_display = DummyLog()

    def log_error(self, msg):
        self.errors.append(str(msg))


def assert_true(condition, message):
    if not condition:
        raise AssertionError(message)


def speed_mm_s(path):
    return [float(item[2]) / 60.0 for item in path]


def assert_accel_limited(path, accel_mm_s2, name):
    speeds = speed_mm_s(path)
    for index in range(1, len(path)):
        dx = float(path[index][0]) - float(path[index - 1][0])
        dy = float(path[index][1]) - float(path[index - 1][1])
        ds = math.hypot(dx, dy)
        dv2 = abs(speeds[index] ** 2 - speeds[index - 1] ** 2)
        allowed = 2.0 * accel_mm_s2 * ds + 0.75
        assert_true(dv2 <= allowed, f"{name} accel jump at {index}: dv2={dv2:.3f} allowed={allowed:.3f}")


def assert_no_mid_speed_drop(path, name):
    speeds = speed_mm_s(path)
    assert_true(len(speeds) > 20, f"{name} produced too few planner points")
    vmax = max(speeds)
    start = len(speeds) // 5
    end = len(speeds) * 4 // 5
    mid_min = min(speeds[start:end])
    assert_true(mid_min >= vmax * 0.80, f"{name} mid-curve speed drop: min={mid_min:.3f} max={vmax:.3f}")


def assert_path_safe(ui, path, name):
    assert_true(path, f"{name} generated no points")
    assert_true(ui.validate_trajectory_points(path, name), f"{name} failed five-bar limit validation")


def assert_bounds(path, expected, name):
    xs = [float(item[0]) for item in path]
    ys = [float(item[1]) for item in path]
    actual = (min(xs), max(xs), min(ys), max(ys))
    for got, want, label in zip(actual, expected, ("min_x", "max_x", "min_y", "max_y")):
        assert_true(abs(got - want) <= 0.05, f"{name} {label}: got {got:.3f}, want {want:.3f}")


def assert_has_arc(segments, name):
    assert_true(any(segment.kind == "arc" for segment in segments), f"{name} did not include rounded arc segments")


def assert_has_line_between(segments, start, end, name):
    for segment in segments:
        if segment.kind != "line":
            continue
        if (
            abs(segment.start[0] - start[0]) <= 0.05
            and abs(segment.start[1] - start[1]) <= 0.05
            and abs(segment.end[0] - end[0]) <= 0.05
            and abs(segment.end[1] - end[1]) <= 0.05
        ):
            return
    raise AssertionError(f"{name} missing preserved line {start}->{end}")


def main():
    ui = DummyUi()
    line = ui.generate_linear_path(75.0, 220.0, 150.0, 250.0, 20.0)
    cw = ui.generate_arc_path(75.0, 220.0, 150.0, 250.0, 60.0, True, 20.0)
    ccw = ui.generate_arc_path(150.0, 250.0, 75.0, 220.0, 60.0, False, 20.0)
    car1 = ui.generate_geometry_path(ui.build_car1_segments(75.0, 200.0), 20.0, label="小车轨迹1")
    car2 = ui.generate_geometry_path(ui.build_car2_segments(75.0, 200.0), 20.0, label="小车轨迹2")
    car1_segments = ui.build_car1_segments(75.0, 200.0)
    rounded_segments = ui.path_planner.rounded_polyline_segments(
        [(75.0, 220.0), (115.0, 220.0), (115.0, 260.0), (150.0, 260.0)],
        corner_radius_mm=3.0,
    )
    rounded = ui.generate_polyline_path(
        [(75.0, 220.0), (115.0, 220.0), (115.0, 260.0), (150.0, 260.0)],
        20.0,
    )
    handwriting_strokes = ui.handwriting_strokes_to_robot([[(0.1, 0.8), (0.3, 0.2), (0.7, 0.6)]])
    handwriting = ui.generate_stroke_path(handwriting_strokes, 20.0, label="handwriting")

    assert_has_arc(rounded_segments, "rounded polyline")
    assert_has_line_between(car1_segments, (183.0, 248.0), (195.0, 236.0), "car1 upper-right fold")

    for name, path in (
        ("G1", line),
        ("G2", cw),
        ("G3", ccw),
        ("car1", car1),
        ("car2", car2),
        ("rounded", rounded),
        ("handwriting", handwriting),
    ):
        assert_path_safe(ui, path, name)
        assert_accel_limited(path, ui.path_planner.accel_mm_s2, name)

    assert_no_mid_speed_drop(line, "G1")
    assert_no_mid_speed_drop(cw, "G2")
    assert_no_mid_speed_drop(ccw, "G3")
    assert_bounds(car1, (75.0, 195.0, 188.0, 248.0), "小车轨迹1")
    assert_bounds(car2, (75.0, 235.0, 188.0, 240.0), "小车轨迹2")
    assert_true(protocol_mod.build_ppr_line(3200) == "PPR 3200 3200", "single-value PPR command mismatch")
    assert_true(protocol_mod.build_ppr_line(3200, 6400) == "PPR 3200 6400", "dual-value PPR command mismatch")

    try:
        strokes = ui.build_text_outline_strokes("FZU")
        text_path = ui.generate_stroke_path(strokes, 20.0, label="FZU")
        assert_path_safe(ui, text_path, "FZU")
    except ModuleNotFoundError:
        print("SKIP text outline check: PySide6 is not installed in this Python runtime")

    print("TRAJECTORY_PLANNER_CHECK PASS")
    print(f"G1 points={len(line)} G2 points={len(cw)} G3 points={len(ccw)} car1={len(car1)} car2={len(car2)} rounded={len(rounded)} handwriting={len(handwriting)}")


if __name__ == "__main__":
    main()
