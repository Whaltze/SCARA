from pathlib import Path
import math
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from SCARA_UI.motion.motion_mixin import ScaraMotionMixin
from SCARA_UI.trajectory.look_ahead import LookAheadPlanner
from SCARA_UI.ui.plotting import ScaraPlotMixin
from SCARA_UI.ui.ui_mixin import HandwritingPad


class GeometryOwner(ScaraMotionMixin):
    def __init__(self):
        self.cur_x = 75.0
        self.cur_y = 220.0
        self.path_planner = LookAheadPlanner(accel_mm_s2=100.0, junction_deviation=0.02)
        self.junction_dev = 0.02
        self.errors = []

    def ui_to_mcu_xy(self, x, y):
        return x - 75.0, y

    def _read_run_accel_mm_s2(self):
        return 80.0

    def validate_trajectory_points(self, _points, label="trajectory"):
        return True

    def log_error(self, message):
        self.errors.append(str(message))


def check_real_geometry_gcode():
    owner = GeometryOwner()
    line = owner.path_planner.line_segment((75.0, 220.0), (85.0, 220.0))
    arc = owner.path_planner.arc_segment((85.0, 220.0), (95.0, 230.0), 10.0, clockwise=False)
    commands = owner.generate_geometry_gcode([line, arc], 20.0, start=(75.0, 220.0))
    assert commands[0].startswith("G1 ")
    assert commands[1].startswith("G3 ")
    assert " I" in commands[1] and " J" in commands[1]
    assert len(commands) == 2

    small_arc = owner.path_planner.arc_segment((75.0, 220.0), (77.0, 222.0), 2.0, clockwise=False)
    small_commands = owner.generate_geometry_gcode([small_arc], 20.0, start=(75.0, 220.0))
    assert len(small_commands) == 1
    assert small_commands[0].startswith("G3 ")

    car_segments = owner.build_car1_segments(75.0, 200.0)
    body_points = [
        (75.0, 200.0),
        (75.0, 224.0),
        (147.0, 224.0),
        (147.0, 248.0),
        (183.0, 248.0),
        (195.0, 236.0),
        (195.0, 200.0),
        (183.0, 200.0),
    ]
    car_endpoints = [car_segments[0].start] + [segment.end for segment in car_segments]
    assert all(point in car_endpoints for point in body_points)
    assert all(segment.kind == "line" for segment in car_segments[:7])

    car = owner.generate_geometry_gcode(car_segments, 20.0, start=(75.0, 220.0))
    assert sum(line.startswith(("G2 ", "G3 ")) for line in car) == 2
    assert car.count("G4 P0.001") == 1
    for x, y in body_points:
        expected = f"X{x - 75.0:.3f} Y{y:.3f}"
        assert any(expected in line for line in car), expected


def check_look_ahead():
    planner = LookAheadPlanner(accel_mm_s2=100.0, junction_deviation=0.02)
    straight = [
        planner.line_segment((0.0, 0.0), (30.0, 0.0)),
        planner.line_segment((30.0, 0.0), (60.0, 0.0)),
    ]
    corner = [
        planner.line_segment((0.0, 0.0), (30.0, 0.0)),
        planner.line_segment((30.0, 0.0), (30.0, 30.0)),
    ]
    planner.plan_segments(straight, 20.0)
    planner.plan_segments(corner, 20.0)
    assert math.isclose(straight[1].entry_speed, 20.0, rel_tol=0.05)
    assert corner[1].entry_speed > 0.0
    assert corner[1].entry_speed < straight[1].entry_speed

    sharp = [
        planner.line_segment((0.0, 0.0), (30.0, 0.0)),
        planner.line_segment((30.0, 0.0), (4.019, 15.0)),
    ]
    planner.plan_segments(sharp, 20.0)
    assert sharp[1].entry_speed < corner[1].entry_speed

    reversal = [
        planner.line_segment((0.0, 0.0), (30.0, 0.0)),
        planner.line_segment((30.0, 0.0), (0.0, 0.0)),
    ]
    planner.plan_segments(reversal, 20.0)
    assert reversal[1].entry_speed == 0.0


def check_stroke_geometry_stream():
    owner = GeometryOwner()
    strokes = [
        [(75.0, 220.0), (85.0, 220.0), (85.0, 230.0)],
        [(95.0, 230.0), (105.0, 230.0)],
    ]
    groups = owner._stroke_geometry_groups(strokes, label="test writing")
    assert all(segment.kind == "line" for group in groups for segment in group)
    first_group = groups[0]
    assert first_group[0].end == (85.0, 220.0)
    assert first_group[1].start == (85.0, 220.0)

    preview, command_source, command_count = owner.generate_stroke_motion(strokes, 20.0, label="test writing")
    commands = list(command_source)
    geometry = [line for line in commands if line.startswith(("G1 ", "G2 ", "G3 "))]
    assert preview
    assert geometry
    assert len(geometry) < len(preview)
    assert all("F1200" in line for line in geometry)
    assert not any(line.startswith(("G2 ", "G3 ")) for line in geometry)
    assert any("X10.000 Y220.000" in line for line in geometry)
    assert commands.count("G4 P0.001") >= 2
    assert any(line.startswith("G0 ") for line in commands)
    assert command_count == len(commands)
    assert math.isclose(owner.path_planner.accel_mm_s2, 80.0)


def check_every_writing_input_point_is_retained():
    owner = GeometryOwner()
    stroke = [
        (75.0, 220.0),
        (75.05, 220.02),
        (75.10, 220.0),
        (75.15, 220.08),
        (75.20, 220.0),
    ]
    groups = owner._stroke_geometry_groups([stroke], label="exact writing")
    assert len(groups) == 1
    endpoints = [groups[0][0].start] + [segment.end for segment in groups[0]]
    assert endpoints == stroke

    preview, command_source, command_count = owner.generate_stroke_motion([stroke], 20.0, label="exact writing")
    commands = list(command_source)
    geometry = [line for line in commands if line.startswith("G1 ")]
    assert len(geometry) == len(stroke) - 1
    for x, y in stroke[1:]:
        assert any(f"X{x - 75.0:.3f} Y{y:.3f}" in line for line in geometry)
    assert command_count == len(commands)
    flagged = [(point[0], point[1]) for point in preview if len(point) > 4 and point[4]]
    assert flagged == stroke[1:]


def check_preview_decimation_preserves_key_points():
    class ExpectedPath:
        def set_expected_path(self, path):
            self.path = list(path)

    class PreviewOwner(ScaraPlotMixin):
        MAX_PREVIEW_POINTS = 4

        def __init__(self):
            self.feedback_error_tracker = ExpectedPath()

        def update_plot(self, force=False):
            return

        def _update_feedback_error_label(self):
            return

    owner = PreviewOwner()
    path = [(float(index), 0.0, 100.0, False, index in (3, 7)) for index in range(10)]
    owner.set_planned_preview(path)
    assert owner.preview_x == [0.0, 3.0, 7.0, 9.0]
    assert len(owner.feedback_error_tracker.path) == len(path)


def check_handwriting_release_endpoint_is_recorded():
    class Pad:
        CAPTURE_MIN_DELTA = HandwritingPad.CAPTURE_MIN_DELTA
        _append_event_point = HandwritingPad._append_event_point

        def __init__(self):
            self._strokes = [[(0.1, 0.1)]]

        def _event_point(self, event):
            return event

    pad = Pad()
    assert pad._append_event_point((0.2, 0.2))
    assert pad._strokes[-1][-1] == (0.2, 0.2)
    assert not pad._append_event_point((0.2001, 0.2001))


def check_large_stroke_simplification_is_iterative():
    owner = GeometryOwner()
    points = [(75.0 + index * 0.02, 220.0 + (0.25 if index % 2 else 0.0)) for index in range(5000)]
    simplified = owner._rdp_points(points, 0.01)
    assert simplified[0] == points[0]
    assert simplified[-1] == points[-1]
    assert len(simplified) > 1000


def check_semantic_font_path_preserves_line_corner():
    owner = GeometryOwner()

    class Element:
        def __init__(self, kind, x, y):
            self.kind = kind
            self.x = x
            self.y = y

        def isMoveTo(self):
            return self.kind == "move"

        def isLineTo(self):
            return self.kind == "line"

        def isCurveTo(self):
            return self.kind == "curve"

    class Path:
        elements = [
            Element("move", 0.0, 0.0),
            Element("line", 10.0, 0.0),
            Element("line", 10.0, 10.0),
            Element("curve", 12.0, 12.0),
            Element("data", 18.0, 12.0),
            Element("data", 20.0, 10.0),
        ]

        def elementCount(self):
            return len(self.elements)

        def elementAt(self, index):
            return self.elements[index]

    contours = owner._qt_path_contours(Path(), lambda x, y: (x, y))
    assert len(contours) == 1
    assert contours[0][1] == (10.0, 0.0)
    assert contours[0][2] == (10.0, 10.0)
    assert contours[0][-1] == (20.0, 10.0)
    assert len(contours[0]) > 4


def check_teach_chaining():
    owner = GeometryOwner()
    planner = owner.path_planner

    def on_arc(seg, p):
        if abs(math.hypot(p[0] - seg.center[0], p[1] - seg.center[1]) - seg.radius) > 1e-4:
            return False
        ang = math.atan2(p[1] - seg.center[1], p[0] - seg.center[0])
        off = (ang - seg.start_angle) % (2.0 * math.pi)
        d = seg.delta_angle
        if d >= 0.0:
            return -1e-6 <= off <= d + 1e-6
        return d - 1e-6 <= off - 2.0 * math.pi <= 1e-6

    # 3-point arc: unit circle, CCW upper semicircle through (1,0)->(0,1)->(-1,0).
    seg = planner.arc_segment_3pt((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0))
    assert seg.kind == "arc"
    assert abs(seg.radius - 1.0) < 1e-6
    assert abs(seg.center[0]) < 1e-6 and abs(seg.center[1]) < 1e-6
    assert seg.delta_angle > 0.0  # CCW because the middle point is above
    assert math.hypot(seg.point_at(0.0)[0] - 1.0, seg.point_at(0.0)[1]) < 1e-9
    assert math.hypot(seg.point_at(seg.length)[0] + 1.0, seg.point_at(seg.length)[1]) < 1e-9
    assert on_arc(seg, (0.0, 1.0))  # passes through the middle point
    # A clockwise version (middle point below) flips the sweep sign.
    seg_cw = planner.arc_segment_3pt((1.0, 0.0), (0.0, -1.0), (-1.0, 0.0))
    assert seg_cw.delta_angle < 0.0

    # Collinear / coincident points cannot form an arc.
    for bad in (((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)), ((0.0, 0.0), (0.0, 0.0), (1.0, 1.0))):
        try:
            planner.arc_segment_3pt(*bad)
            raise AssertionError("collinear/coincident points must raise")
        except ValueError:
            pass

    # Line chain: N points -> N-1 shared-endpoint line segments.
    line_pts = [(0.0, 200.0), (10.0, 200.0), (10.0, 210.0), (0.0, 210.0)]
    line_segs = owner._teach_primitive_segments("直线模式", line_pts)
    assert len(line_segs) == len(line_pts) - 1
    assert all(s.kind == "line" for s in line_segs)
    for a, b in zip(line_segs, line_segs[1:]):
        assert math.hypot(a.end[0] - b.start[0], a.end[1] - b.start[1]) < 1e-9
    assert owner._teach_primitive_count("直线模式", len(line_pts)) == len(line_pts) - 1

    # Arc chain: shared endpoints, N//2 arcs, last teach point always covered (wrap when needed).
    def circle_points(n):
        cx, cy, r = 75.0, 200.0, 40.0
        return [(cx + r * math.cos(i * 0.6), cy + r * math.sin(i * 0.6)) for i in range(n)]

    for n in (3, 4, 5, 6, 7, 8):
        pts = circle_points(n)
        segs = owner._teach_arc_chain(pts)
        assert len(segs) == n // 2, f"N={n}: expected {n // 2} arcs, got {len(segs)}"
        assert owner._teach_primitive_count("圆弧模式", n) == n // 2
        assert all(s.kind == "arc" for s in segs)
        # consecutive arcs share an endpoint (continuous path)
        for a, b in zip(segs, segs[1:]):
            assert math.hypot(a.end[0] - b.start[0], a.end[1] - b.start[1]) < 1e-6
        # arc1 starts exactly at the first teach point
        assert math.hypot(segs[0].start[0] - pts[0][0], segs[0].start[1] - pts[0][1]) < 1e-9
        # the last teach point lies on one of the arcs
        assert any(on_arc(s, pts[-1]) for s in segs), f"N={n}: last teach point not covered"

    # Too-few points raise clearly.
    for mode, pts in (("直线模式", [(0.0, 200.0)]), ("圆弧模式", [(0.0, 200.0), (10.0, 200.0)])):
        try:
            owner._teach_primitive_segments(mode, pts)
            raise AssertionError("insufficient points must raise")
        except ValueError:
            pass


def main():
    check_real_geometry_gcode()
    check_look_ahead()
    check_stroke_geometry_stream()
    check_every_writing_input_point_is_retained()
    check_preview_decimation_preserves_key_points()
    check_handwriting_release_endpoint_is_recorded()
    check_large_stroke_simplification_is_iterative()
    check_semantic_font_path_preserves_line_corner()
    check_teach_chaining()
    print("TRAJECTORY_PLANNER_CHECK PASS")


if __name__ == "__main__":
    main()
