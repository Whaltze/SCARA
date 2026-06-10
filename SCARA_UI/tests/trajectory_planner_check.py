import importlib.util
import inspect
import math
from dataclasses import dataclass
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

MRAD_PER_REV = motion_mod.ScaraMotionMixin.BINARY_MRAD_PER_REV
DEFAULT_ZERO_MRAD = motion_mod.ScaraMotionMixin.BINARY_ZERO_MRAD
FLAG_EXACT_STOP = motion_mod.ScaraMotionMixin.BINARY_FLAG_EXACT_STOP
FLAG_CARTESIAN_LINE = motion_mod.ScaraMotionMixin.BINARY_FLAG_CARTESIAN_LINE
FLAG_HOST_SEGMENT = motion_mod.ScaraMotionMixin.BINARY_FLAG_HOST_SEGMENT

if "generate_binary_line_targets" not in inspect.getsource(motion_mod.ScaraMotionMixin.plan_trajectory):
    raise AssertionError("G1 UI button is not using the binary geometric line path")


@dataclass
class JointPoint:
    p1_abs: int
    p2_abs: int
    v_dom_pps: int
    flags: int


def joint_deg_to_pulse(theta1_deg, theta2_deg, ppr, zero_mrad=DEFAULT_ZERO_MRAD):
    theta1_mrad = math.radians(float(theta1_deg)) * 1000.0
    theta2_mrad = math.radians(float(theta2_deg)) * 1000.0
    p1 = int(round(((theta1_mrad - zero_mrad[0]) * int(ppr)) / MRAD_PER_REV))
    p2 = int(round(((theta2_mrad - zero_mrad[1]) * int(ppr)) / MRAD_PER_REV))
    return p1, p2


def path_to_joint_points(path, kinematics, ppr, start_xy=None):
    result = []
    start_xy = start_xy or (float(path[0][0]), float(path[0][1]))
    prev = joint_deg_to_pulse(*kinematics.inverse(*start_xy), ppr)
    for point in path:
        q = kinematics.inverse(float(point[0]), float(point[1]))
        pulse = joint_deg_to_pulse(*q, ppr)
        flags = int(point[4]) if len(point) > 4 else 0
        dom = max(abs(pulse[0] - prev[0]), abs(pulse[1] - prev[1]))
        feed = max(1.0, float(point[2]) / 60.0) if len(point) > 2 else 1.0
        v_dom = max(16, min(10000, int(round(dom * feed))))
        result.append(JointPoint(pulse[0], pulse[1], v_dom, flags))
        prev = pulse
    return result


class DummyLog:
    def __init__(self):
        self.items = []

    def append(self, item):
        self.items.append(item)


class DummyUi(motion_mod.ScaraMotionMixin):
    def __init__(self):
        self.L0, self.L1, self.L2 = 150.0, 160.0, 200.0
        self.current_ppr = 3200
        self.kinematics = kinematics_mod.FiveBarKinematics()
        self.HOME_X, self.HOME_Y = self.kinematics.forward(90.0, 90.0)
        self.cur_x, self.cur_y = self.HOME_X, self.HOME_Y
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


def assert_arc_endpoints_consistent(segments, name):
    for index, segment in enumerate(segments):
        if segment.kind != "arc":
            continue
        end = segment.point_at(segment.length)
        error = math.hypot(end[0] - segment.end[0], end[1] - segment.end[1])
        assert_true(error <= 1e-6, f"{name} arc {index} endpoint mismatch: {error:.6f}mm")


def point_line_error(px, py, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    den = vx * vx + vy * vy
    if den <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * vx + (py - ay) * vy) / den
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + vx * t), py - (ay + vy * t))


def max_joint_linearized_line_error(ui, start, targets, ppr=3200):
    prev_pulse = joint_deg_to_pulse(*ui.kinematics.inverse(*start), ppr)
    max_error = 0.0
    for target in targets:
        x, y = target[0], target[1]
        pulse = joint_deg_to_pulse(*ui.kinematics.inverse(x, y), ppr)
        steps = max(abs(pulse[0] - prev_pulse[0]), abs(pulse[1] - prev_pulse[1]), 1)
        for index in range(1, steps + 1):
            t = index / steps
            p1 = round(prev_pulse[0] + (pulse[0] - prev_pulse[0]) * t)
            p2 = round(prev_pulse[1] + (pulse[1] - prev_pulse[1]) * t)
            q1 = math.degrees(((p1 * MRAD_PER_REV / ppr) + DEFAULT_ZERO_MRAD[0]) / 1000.0)
            q2 = math.degrees(((p2 * MRAD_PER_REV / ppr) + DEFAULT_ZERO_MRAD[1]) / 1000.0)
            xy = ui.kinematics.forward(q1, q2)
            max_error = max(max_error, point_line_error(xy[0], xy[1], start[0], start[1], targets[-1][0], targets[-1][1]))
        prev_pulse = pulse
    return max_error


def max_joint_path_error(ui, start, targets, expected_path, ppr=3200):
    prev_pulse = joint_deg_to_pulse(*ui.kinematics.inverse(*start), ppr)
    expected = [(float(p[0]), float(p[1])) for p in expected_path]
    expanded = []
    cursor = (float(start[0]), float(start[1]))
    for target in targets:
        x, y = float(target[0]), float(target[1])
        flags = int(target[4]) if len(target) > 4 else 0
        expanded.append((x, y))
        cursor = (x, y)
    max_error = 0.0
    for x, y in expanded:
        pulse = joint_deg_to_pulse(*ui.kinematics.inverse(x, y), ppr)
        steps = max(abs(pulse[0] - prev_pulse[0]), abs(pulse[1] - prev_pulse[1]), 1)
        for index in range(1, steps + 1):
            t = index / steps
            p1 = round(prev_pulse[0] + (pulse[0] - prev_pulse[0]) * t)
            p2 = round(prev_pulse[1] + (pulse[1] - prev_pulse[1]) * t)
            q1 = math.degrees(((p1 * MRAD_PER_REV / ppr) + DEFAULT_ZERO_MRAD[0]) / 1000.0)
            q2 = math.degrees(((p2 * MRAD_PER_REV / ppr) + DEFAULT_ZERO_MRAD[1]) / 1000.0)
            xy = ui.kinematics.forward(q1, q2)
            nearest = min(
                point_line_error(xy[0], xy[1], expected[i][0], expected[i][1], expected[i + 1][0], expected[i + 1][1])
                for i in range(len(expected) - 1)
            )
            max_error = max(max_error, nearest)
        prev_pulse = pulse
    return max_error


def assert_closed_strokes(ui, strokes, name):
    assert_true(strokes, f"{name} generated no text contours")
    for index, stroke in enumerate(strokes):
        assert_true(ui._is_closed_stroke(stroke, threshold=0.5), f"{name} contour {index} is not closed")


def assert_host_segments_valid(ui, path, start_xy, name):
    segments = ui.build_host_segments_from_path(path, start_xy=start_xy, label=name)
    assert_true(segments, f"{name} produced no HOST_SEGMENT output")
    q1, q2 = ui.inverse_kinematics(start_xy[0], start_xy[1])
    assert_true(q1 is not None and q2 is not None, f"{name} invalid host start")
    last = ui._joint_deg_to_pulse(q1, q2)
    for index, segment in enumerate(segments):
        assert_true(segment.flags & FLAG_HOST_SEGMENT, f"{name} segment {index} lost HOST flag")
        duration = int(segment.duration_ticks)
        dominant = max(abs(int(segment.p1_abs) - last[0]), abs(int(segment.p2_abs) - last[1]))
        assert_true(duration >= 1, f"{name} segment {index} has zero duration")
        assert_true(duration <= 65535, f"{name} segment {index} exceeds uint16 duration")
        assert_true(dominant <= duration, f"{name} segment {index} requests multi-pulse tick: d={dominant} ticks={duration}")
        last = (int(segment.p1_abs), int(segment.p2_abs))
    assert_true(segments[-1].flags & FLAG_EXACT_STOP, f"{name} final host segment is not exact-stop")
    return segments


def main():
    ui = DummyUi()
    line = ui.generate_linear_path(75.0, 220.0, 150.0, 250.0, 20.0)
    cw = ui.generate_arc_path(75.0, 220.0, 150.0, 250.0, 60.0, True, 20.0)
    ccw = ui.generate_arc_path(150.0, 250.0, 75.0, 220.0, 60.0, False, 20.0)
    car1 = ui.generate_geometry_path(ui.build_car1_segments(75.0, 200.0), 20.0, label="小车轨迹1")
    car2 = ui.generate_geometry_path(ui.build_car2_segments(75.0, 200.0), 20.0, label="小车轨迹2")
    car1_segments = ui.build_car1_segments(75.0, 200.0)
    car1_preview, car1_send = ui.generate_geometry_motion(car1_segments, 20.0, label="car1")
    car2_preview, car2_send = ui.generate_geometry_motion(ui.build_car2_segments(75.0, 200.0), 20.0, label="car2")
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
    assert_has_arc(car1_segments, "car1")
    assert_has_arc(ui.build_car2_segments(75.0, 200.0), "car2")
    assert_arc_endpoints_consistent(car1_segments, "car1")
    assert_arc_endpoints_consistent(ui.build_car2_segments(75.0, 200.0), "car2")
    assert_arc_endpoints_consistent(rounded_segments, "rounded polyline")

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
    line_send = ui.generate_binary_line_targets((75.0, 220.0), (150.0, 250.0), 20.0)
    arc_send = ui.generate_binary_arc_targets((75.0, 220.0), (150.0, 250.0), 60.0, True, 20.0)
    assert_true(len(line_send) == 1, f"G1 binary line should upload endpoint only: send={len(line_send)}")
    assert_true(line_send[0][0] == 150.0 and line_send[0][1] == 250.0, "G1 binary endpoint changed")
    assert_true((line_send[0][4] & ui.BINARY_FLAG_EXACT_STOP) != 0, "G1 endpoint is not exact-stop")
    assert_true(8 <= len(arc_send) < len(cw) // 2, f"G2 binary arc keypoints not compact: send={len(arc_send)} preview={len(cw)}")
    joint_points = path_to_joint_points(line_send, ui.kinematics, 3200, start_xy=(75.0, 220.0))
    assert_true(len(joint_points) == len(line_send), "G1 binary joint conversion changed keypoint count")
    assert_true((joint_points[0].flags & FLAG_EXACT_STOP) != 0, "G1 exact-stop flag lost during joint conversion")
    assert_true(all(16 <= point.v_dom_pps <= 10000 for point in joint_points), "G1 v_dom out of range")
    assert_true(car1_preview and car1_send, "car1 geometry motion did not produce binary send path")
    assert_true(car2_preview and car2_send, "car2 geometry motion did not produce binary send path")
    assert_true(car1_send[0][3] is True and car1_send[0][1] < 220.0, "car1 missing silent connector to shape start")
    assert_true(car2_send[0][3] is True and car2_send[0][1] < 220.0, "car2 missing silent connector to shape start")
    assert_true(any((point[4] & FLAG_CARTESIAN_LINE) != 0 for point in car1_send), "car1 has no endpoint-style line blocks")
    assert_true(any((point[4] & FLAG_CARTESIAN_LINE) != 0 for point in car2_send), "car2 has no endpoint-style line blocks")
    assert_true(all((point[4] & ui.BINARY_FLAG_EXACT_STOP) != 0 for point in car1_send if point[4] & ui.BINARY_FLAG_CARTESIAN_LINE), "car1 line endpoints are not exact-stop")
    assert_true(all((point[4] & ui.BINARY_FLAG_EXACT_STOP) != 0 for point in car2_send if point[4] & ui.BINARY_FLAG_CARTESIAN_LINE), "car2 line endpoints are not exact-stop")
    assert_true(len(car1_send) < len(car1_preview) // 5, f"car1 binary path was not simplified: send={len(car1_send)} preview={len(car1_preview)}")
    assert_true(len(car2_send) < len(car2_preview) // 5, f"car2 binary path was not simplified: send={len(car2_send)} preview={len(car2_preview)}")
    assert_true(len(path_to_joint_points(car1_send, ui.kinematics, 3200, start_xy=(75.0, 220.0))) <= 3000, "car1 binary upload exceeds stress target size")
    assert_true(len(path_to_joint_points(car2_send, ui.kinematics, 3200, start_xy=(75.0, 220.0))) <= 3000, "car2 binary upload exceeds stress target size")
    handwriting_send = ui.generate_binary_send_from_path(handwriting, 20.0)
    assert_true(len(handwriting_send) < len(handwriting), f"handwriting binary path was not simplified: send={len(handwriting_send)} preview={len(handwriting)}")
    line_host = assert_host_segments_valid(ui, line, (75.0, 220.0), "G1 host")
    arc_host = assert_host_segments_valid(ui, cw, (75.0, 220.0), "G2 host")
    car1_host = assert_host_segments_valid(ui, car1_preview, (ui.cur_x, ui.cur_y), "car1 host")
    car2_host = assert_host_segments_valid(ui, car2_preview, (ui.cur_x, ui.cur_y), "car2 host")
    handwriting_host = assert_host_segments_valid(ui, handwriting, (ui.cur_x, ui.cur_y), "handwriting host")
    assert_true(len(line_host) >= len(line) // 2, "G1 host output unexpectedly sparse")
    assert_true(len(arc_host) >= len(cw) // 2, "G2 host output unexpectedly sparse")
    assert_true(len(car1_host) > 100 and len(car2_host) > 100 and len(handwriting_host) > 100, "complex host output too small")
    assert_bounds(car1, (75.0, 195.0, 188.0, 248.0), "小车轨迹1")
    assert_bounds(car2, (75.0, 235.0, 188.0, 240.0), "小车轨迹2")
    assert_true(protocol_mod.build_ppr_line(3200) == "$100=3200 $101=3200", "single-value PPR command mismatch")
    assert_true(protocol_mod.build_ppr_line(3200, 6400) == "$100=3200 $101=6400", "dual-value PPR command mismatch")
    assert_true(protocol_mod.build_g1_timed_line(10, -20, 300, 7) == "G1 A10 B-20 T300 ;ID=7 HOST=1", "G1 A/B/T command mismatch")

    square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)]
    rotated = ui._rotate_closed_stroke_near_current(square, (9.8, 9.7))
    assert_true(ui._is_closed_stroke(rotated), "rotated text contour is not closed")
    assert_true(math.hypot(rotated[0][0] - 10.0, rotated[0][1] - 10.0) <= 0.5, "closed contour did not choose nearest start")

    try:
        for sample in ("福州大学", "FZU", "SCARA2026", "FZU福大2026"):
            strokes = ui.build_text_outline_strokes(sample, height_mm=70.0)
            assert_closed_strokes(ui, strokes, sample)
            text_path = ui.generate_stroke_path(
                strokes,
                20.0,
                label=sample,
                simplify_tolerance=ui.TEXT_SIMPLIFY_TOLERANCE_MM,
                min_spacing=ui.TEXT_MIN_POINT_SPACING_MM,
                corner_radius_mm=ui.TEXT_CORNER_RADIUS_MM,
                optimize_closed_start=True,
            )
            assert_path_safe(ui, text_path, sample)
            assert_accel_limited(text_path, ui.path_planner.accel_mm_s2, sample)
        fzu_strokes = ui.build_text_outline_strokes("FZU", height_mm=70.0)
        first_min_x, _, _, _ = ui._stroke_bounds(fzu_strokes[0])
        last_min_x, _, _, _ = ui._stroke_bounds(fzu_strokes[-1])
        assert_true(first_min_x <= last_min_x, "FZU contours are not ordered left-to-right")
    except (ModuleNotFoundError, ImportError):
        print("SKIP text outline check: PySide6 is not installed in this Python runtime")

    print("TRAJECTORY_PLANNER_CHECK PASS")
    print(f"G1 points={len(line)} G2 points={len(cw)} G3 points={len(ccw)} car1={len(car1)} car2={len(car2)} rounded={len(rounded)} handwriting={len(handwriting)}")


if __name__ == "__main__":
    main()
