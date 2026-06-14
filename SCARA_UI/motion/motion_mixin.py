import math
import os

import numpy as np


class ScaraMotionMixin:
    JOINT_LIMITS_DEG = ((-180.0, 180.0), (0.0, 360.0))
    # Preview sampling is independent from the compact geometry G-code stream.
    ARC_SEGMENT_MM = 2.0
    PREVIEW_ARC_SEGMENT_MM = 0.75
    RAD_PER_REV = 2.0 * math.pi
    JOINT_ZERO_RAD = (2.251, 0.890)
    DEFAULT_CORNER_RADIUS_MM = 0.0
    CAR_CORNER_RADIUS_MM = 0.0
    # 小车要求“每段到点停稳再转向”：在每条直线/圆弧后插入该时长的 G4 暂停，
    # 复用固件 exact-stop 屏障(相邻 dwell 块把 junction 速度强制为 0)，
    # 让开环步进减速到 0、停稳片刻让杆件振荡衰减后再转向，最大化落点精度。
    # 仅小车轨迹启用；其余轨迹保持默认 0(不插停顿，行为不变)。
    CAR_EXACT_STOP_DWELL_MS = 30.0
    # Writing is a true polyline. Every input control point is a real G1
    # endpoint; GRBL junction deviation slows at the sharp corner without
    # replacing it with a fillet arc or deleting a nearby point.
    HANDWRITING_CORNER_RADIUS_MM = 0.0
    PATH_SIMPLIFY_TOLERANCE_MM = 0.0
    PATH_MIN_POINT_SPACING_MM = 0.001
    # Font paths retain true LineTo corners. Only CurveTo elements are
    # adaptively flattened to this Cartesian tolerance.
    TEXT_SIMPLIFY_TOLERANCE_MM = 0.0
    TEXT_MIN_POINT_SPACING_MM = 0.001
    TEXT_CURVE_TOLERANCE_MM = 0.10
    TEXT_CORNER_RADIUS_MM = 0.0
    DRAW_CENTER_X = 75.0
    DRAW_CENTER_Y = 220.0
    JOG_MIN_SMOOTH_PPS = 50.0
    DEFAULT_RUN_ACCEL_MM_S2 = 100.0
    # 自定义图案（SVG 线条 / 图片点阵）共用的缩放安全框，单位 mm；与空心字一致。
    PATTERN_MAX_WIDTH_MM = 135.0
    PATTERN_MAX_HEIGHT_MM = 85.0
    # 图片激光点阵参数：网格点间距、驻点出光停留、灰度阈值（< 阈值视为偏暗→打点）、点阵安全框。
    HALFTONE_SPACING_MM = 2.0
    HALFTONE_DWELL_MS = 30.0
    HALFTONE_THRESHOLD = 128
    HALFTONE_MAX_WIDTH_MM = 120.0
    HALFTONE_MAX_HEIGHT_MM = 80.0

    def _read_jog_step_mm(self):
        widget = getattr(self, "jog_step_input", None)
        if widget is None:
            widget = getattr(self, "jog_speed_input", None)
        return self._read_float(widget, "点动步长", positive=True)

    def _read_run_speed_mm_s(self):
        return self._read_float(self.hw_speed_input, "运行速度", positive=True)

    def _read_run_accel_mm_s2(self):
        widget = getattr(self, "hw_accel_input", None)
        if widget is None:
            return self.DEFAULT_RUN_ACCEL_MM_S2
        return self._read_float(widget, "运行加速度", positive=True)

    def inverse_kinematics(self, x, y):
        return self.kinematics.inverse(x, y)

    def _sync_preview_planner_profile(self):
        """Keep preview geometry limits identical to the next MCU task."""
        accel = self._read_run_accel_mm_s2()
        junction = float(getattr(self, "junction_dev", 0.02))
        self.path_planner.accel_mm_s2 = max(1.0, float(accel))
        self.path_planner.junction_deviation = max(0.001, junction)

    def check_workspace_safety(self, x, y):
        return self.kinematics.is_reachable(x, y, margin=5.0)

    def _read_float(self, widget, name, positive=False):
        text = widget.text().strip()
        if not text:
            raise ValueError(f"{name}不能为空")
        try:
            value = float(text)
        except ValueError as exc:
            raise ValueError(f"{name}必须是数字，当前为 {text!r}") from exc
        if positive and value <= 0.0:
            raise ValueError(f"{name}必须大于 0，当前为 {value:g}")
        return value

    def _limit_violations_at(self, x, y, index):
        c = self.kinematics.config
        violations = []
        if not math.isfinite(x) or not math.isfinite(y):
            return [f"Point {index}({x},{y}): coordinate is not finite; motion cannot be verified."]
        d1 = math.hypot(x, y)
        d2 = math.hypot(x - c.base_distance, y)

        if y < c.min_y:
            excess = c.min_y - y
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): Y 低于下限 {c.min_y:.1f}mm，超出 {excess:.2f}mm；"
                f"请将 Y 至少增大 {excess:.2f}mm 到 {c.min_y:.2f}mm"
            )
        if d1 < c.min_anchor_dist:
            excess = c.min_anchor_dist - d1
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): 左基座距离低于下限 {c.min_anchor_dist:.1f}mm，超出 {excess:.2f}mm；"
                f"请将该点远离左基座至少 {excess:.2f}mm"
            )
        if d1 > c.max_anchor_dist:
            excess = d1 - c.max_anchor_dist
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): 左基座距离超过上限 {c.max_anchor_dist:.1f}mm，超出 {excess:.2f}mm；"
                f"请将该点向左基座靠近至少 {excess:.2f}mm"
            )
        if d2 < c.min_anchor_dist:
            excess = c.min_anchor_dist - d2
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): 右基座距离低于下限 {c.min_anchor_dist:.1f}mm，超出 {excess:.2f}mm；"
                f"请将该点远离右基座至少 {excess:.2f}mm"
            )
        if d2 > c.max_anchor_dist:
            excess = d2 - c.max_anchor_dist
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): 右基座距离超过上限 {c.max_anchor_dist:.1f}mm，超出 {excess:.2f}mm；"
                f"请将该点向右基座靠近至少 {excess:.2f}mm"
            )

        q1, q2 = self.inverse_kinematics(x, y)
        if q1 is None or q2 is None:
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): 五连杆无逆解；请将目标点向工作空间中央移动后重新预览"
            )
            return violations

        for axis, angle in ((1, q1), (2, q2)):
            low, high = self.JOINT_LIMITS_DEG[axis - 1]
            if angle < low:
                excess = low - angle
                violations.append(
                    f"点{index}({x:.2f},{y:.2f}): M{axis} 低于下限 {low:.1f}deg，超出 {excess:.2f}deg；"
                    f"请调整目标点，使 M{axis} 至少增大 {excess:.2f}deg"
                )
            if angle > high:
                excess = angle - high
                violations.append(
                    f"点{index}({x:.2f},{y:.2f}): M{axis} 超过上限 {high:.1f}deg，超出 {excess:.2f}deg；"
                    f"请调整目标点，使 M{axis} 至少减小 {excess:.2f}deg"
                )

        q1_rad = math.radians(q1)
        q2_rad = math.radians(q2)
        left_elbow = (
            c.active_link * math.cos(q1_rad),
            c.active_link * math.sin(q1_rad),
        )
        right_elbow = (
            c.base_distance + c.active_link * math.cos(q2_rad),
            c.active_link * math.sin(q2_rad),
        )
        if left_elbow[0] >= right_elbow[0]:
            excess = left_elbow[0] - right_elbow[0]
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): 左右主动臂交叉，交叉量 {excess:.2f}mm；"
                "请调整目标点，使左臂肘点位于右臂肘点左侧"
            )
        if left_elbow[1] < 0.0:
            excess = -left_elbow[1]
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): 左主动臂低于基座线，超出 {excess:.2f}mm；"
                "请提高目标点 Y 或向工作空间中央移动"
            )
        if right_elbow[1] < 0.0:
            excess = -right_elbow[1]
            violations.append(
                f"点{index}({x:.2f},{y:.2f}): 右主动臂低于基座线，超出 {excess:.2f}mm；"
                "请提高目标点 Y 或向工作空间中央移动"
            )

        return violations

    def validate_trajectory_points(self, points, label="轨迹"):
        violations = []
        total = 0
        for index, point in enumerate(points, start=1):
            x, y = float(point[0]), float(point[1])
            point_violations = self._limit_violations_at(x, y, index)
            total += len(point_violations)
            if point_violations and len(violations) < 5:
                violations.extend(point_violations[: 5 - len(violations)])

        if total:
            self.log_error(f"{label}预检查失败，共 {total} 项超限，已拦截发送；请按以下建议修改后重新预览")
            for item in violations:
                self.log_error(item)
            if total > len(violations):
                self.log_error(f"其余 {total - len(violations)} 项超限已省略")
            return False
        return True

    def generate_linear_path(self, x1, y1, x2, y2, speed_max, v_start=0.0, v_end=0.0, silent=False):
        self._sync_preview_planner_profile()
        planned = self.path_planner.plan_line(
            (x1, y1),
            (x2, y2),
            feed_mm_s=speed_max,
            start_speed=v_start,
            end_speed=v_end,
            silent=silent,
        )
        path = [(p.x, p.y, p.feed_mm_min, p.silent, p.key_point) for p in planned]
        if path and not self.validate_trajectory_points(path, "直线路径"):
            return []
        return path

    def generate_arc_path(self, x1, y1, x2, y2, radius, clockwise, speed_max, silent=False):
        self._sync_preview_planner_profile()
        planned = self.path_planner.plan_arc(
            (x1, y1),
            (x2, y2),
            radius,
            clockwise=clockwise,
            feed_mm_s=speed_max,
            start_speed=0.0,
            end_speed=0.0,
            silent=silent,
        )
        path = [(p.x, p.y, p.feed_mm_min, p.silent, p.key_point) for p in planned]
        if path and not self.validate_trajectory_points(path, "圆弧路径"):
            return []
        return path

    def generate_polyline_path(self, points, speed_max, silent_first=False):
        self._sync_preview_planner_profile()
        # 上位机按真实弧长预先规划速度；下位机只接收带 F 的 G1 点流。
        clean_points = self.preprocess_control_points(points)
        if len(clean_points) < 2:
            self.log_error("轨迹点过少，至少需要起点和终点")
            return []
        if not self.validate_trajectory_points(clean_points, "轨迹控制点"):
            return []
        planned = self.path_planner.plan_rounded_polyline(
            clean_points,
            feed_mm_s=speed_max,
            corner_radius_mm=self.DEFAULT_CORNER_RADIUS_MM,
            silent_first=silent_first,
        )
        path = [(p.x, p.y, p.feed_mm_min, p.silent, p.key_point) for p in planned]
        if path and not self.validate_trajectory_points(path, "轨迹采样点"):
            return []
        return path

    def preprocess_control_points(self, points, simplify_tolerance=None, min_spacing=None):
        simplify_tolerance = self.PATH_SIMPLIFY_TOLERANCE_MM if simplify_tolerance is None else simplify_tolerance
        min_spacing = self.PATH_MIN_POINT_SPACING_MM if min_spacing is None else min_spacing
        clean = []
        for x, y in points:
            p = (float(x), float(y))
            if not clean or np.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) >= min_spacing:
                clean.append(p)
        if len(clean) > 2 and simplify_tolerance > 0:
            clean = self._rdp_points(clean, simplify_tolerance)
        return clean

    def _rdp_points(self, points, epsilon):
        if len(points) <= 2:
            return list(points)
        # Iterative RDP avoids Python recursion failures on long handwriting and
        # font-outline strokes while preserving every geometrically significant
        # corner as an actual polyline endpoint.
        keep = {0, len(points) - 1}
        pending = [(0, len(points) - 1)]
        while pending:
            start_index, end_index = pending.pop()
            start = points[start_index]
            end = points[end_index]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            max_dist = -1.0
            split_index = -1
            for index in range(start_index + 1, end_index):
                px, py = points[index]
                if length <= 1e-9:
                    dist = math.hypot(px - start[0], py - start[1])
                else:
                    dist = abs(dy * px - dx * py + end[0] * start[1] - end[1] * start[0]) / length
                if dist > max_dist:
                    max_dist = dist
                    split_index = index
            if split_index >= 0 and max_dist > epsilon:
                keep.add(split_index)
                pending.append((start_index, split_index))
                pending.append((split_index, end_index))
        return [points[index] for index in sorted(keep)]

    def generate_geometry_path(self, segments, speed_max, silent_first=False, label="固定轨迹"):
        self._sync_preview_planner_profile()
        control_points = []
        for segment in segments:
            if not control_points:
                control_points.append(segment.start)
            control_points.append(segment.end)
        if len(control_points) < 2:
            self.log_error(f"{label}点过少，至少需要起点和终点")
            return []
        if not self.validate_trajectory_points(control_points, f"{label}控制点"):
            return []
        planned = self.path_planner.plan_segments(segments, feed_mm_s=speed_max, silent_first=silent_first)
        path = [(p.x, p.y, p.feed_mm_min, p.silent, p.key_point) for p in planned]
        if path and not self.validate_trajectory_points(path, f"{label}采样点"):
            return []
        return path

    def generate_geometry_motion(self, segments, speed_max, label="固定轨迹", exact_stop_dwell_ms=0.0):
        """Generate a sampled preview and compact geometry G-code."""
        if not segments:
            self.log_error(f"{label}没有有效几何段")
            return [], []
        preview = []
        start = (self.cur_x, self.cur_y)
        first = segments[0].start
        if math.hypot(first[0] - start[0], first[1] - start[1]) > 0.01:
            connector = self.generate_linear_path(start[0], start[1], first[0], first[1], speed_max, silent=True)
            if not connector:
                self.log_error(f"{label}连接到起点失败: 当前({start[0]:.1f},{start[1]:.1f}) -> 起点({first[0]:.1f},{first[1]:.1f})")
                return [], []
            preview.extend(connector)
        body = self.generate_geometry_path(segments, speed_max, silent_first=True, label=label)
        if not body:
            return [], []
        preview.extend(body)
        return preview, self.generate_geometry_gcode(
            segments, speed_max, start=start, exact_stop_dwell_ms=exact_stop_dwell_ms
        )

    def generate_geometry_gcode(self, segments, speed_max, start=None, exact_stop_dwell_ms=0.0):
        """Generate compact G0/G1/G2/G3 commands without rewriting true geometry."""
        return list(
            self._iter_geometry_gcode(
                segments, speed_max, start=start, exact_stop_dwell_ms=exact_stop_dwell_ms
            )
        )

    def _iter_geometry_gcode(self, segments, speed_max, start=None, exact_stop_dwell_ms=0.0):
        """Yield compact geometry commands without sampling the path.

        When ``exact_stop_dwell_ms > 0`` a ``G4`` dwell is appended after every
        line/arc so the controller treats each segment as an exact-stop barrier
        (junction speed forced to 0) and the arm settles at every commanded
        vertex before turning. The line/arc commands themselves are unchanged.
        """
        cursor = tuple(start or (self.cur_x, self.cur_y))
        dwell_cmd = f"G4 P{exact_stop_dwell_ms / 1000.0:.3f}" if exact_stop_dwell_ms > 0.0 else None

        def mcu_xy(point):
            if hasattr(self, "ui_to_mcu_xy"):
                return self.ui_to_mcu_xy(float(point[0]), float(point[1]))
            return float(point[0]), float(point[1])

        first = tuple(segments[0].start) if segments else cursor
        if math.hypot(first[0] - cursor[0], first[1] - cursor[1]) > 0.01:
            x, y = mcu_xy(first)
            yield f"G0 X{x:.3f} Y{y:.3f}"
            yield "G4 P0.001"
            cursor = first
        for segment in segments:
            if math.hypot(segment.start[0] - cursor[0], segment.start[1] - cursor[1]) > 0.01:
                x, y = mcu_xy(segment.start)
                yield f"G0 X{x:.3f} Y{y:.3f}"
            feed = max(1, int(round(float(speed_max) * 60.0)))
            x, y = mcu_xy(segment.end)
            if segment.kind == "arc":
                code = "G2" if segment.delta_angle < 0.0 else "G3"
                i = float(segment.center[0]) - float(segment.start[0])
                j = float(segment.center[1]) - float(segment.start[1])
                yield f"{code} X{x:.3f} Y{y:.3f} I{i:.3f} J{j:.3f} F{feed}"
            else:
                yield f"G1 X{x:.3f} Y{y:.3f} F{feed}"
            if dwell_cmd is not None:
                yield dwell_cmd
            cursor = tuple(segment.end)

    def generate_arc_control_points(self, start, end, radius, clockwise):
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        chord = math.hypot(dx, dy)
        if chord < 0.001:
            raise ValueError("圆弧起点和终点重合；请将任一目标点移动至少 0.001mm 后重试")
        min_radius = chord * 0.5
        if radius < min_radius:
            missing = min_radius - radius
            chord_excess = chord - 2.0 * radius
            raise ValueError(
                f"圆弧半径不足 {missing:.2f}mm：两点距离为 {chord:.2f}mm，半径至少需要 {min_radius:.2f}mm，"
                f"当前为 {radius:.2f}mm；请将半径增大到至少 {min_radius:.2f}mm，"
                f"或将两点距离缩短至少 {chord_excess:.2f}mm"
            )

        mx = (sx + ex) * 0.5
        my = (sy + ey) * 0.5
        h = math.sqrt(max(0.0, radius * radius - min_radius * min_radius))
        nx = -dy / chord
        ny = dx / chord
        centers = ((mx + h * nx, my + h * ny), (mx - h * nx, my - h * ny))

        candidates = []
        for cx, cy in centers:
            a0 = math.atan2(sy - cy, sx - cx)
            a1 = math.atan2(ey - cy, ex - cx)
            if clockwise:
                delta = -((a0 - a1) % (2.0 * math.pi))
            else:
                delta = (a1 - a0) % (2.0 * math.pi)
            if abs(delta) < 1e-9:
                delta = -2.0 * math.pi if clockwise else 2.0 * math.pi
            candidates.append((abs(delta), cx, cy, a0, delta))

        _, cx, cy, a0, delta = min(candidates, key=lambda item: item[0])
        arc_len = abs(delta) * radius
        segments = max(8, int(math.ceil(arc_len / self.PREVIEW_ARC_SEGMENT_MM)))
        segments = min(segments, 5000)
        points = []
        for i in range(segments + 1):
            t = i / segments
            a = a0 + delta * t
            points.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
        points[0] = start
        points[-1] = end
        return points

    def _xy_from_pulse(self, p1, p2, ppr):
        q1 = math.degrees(float(p1) * self.RAD_PER_REV / float(ppr) + self.JOINT_ZERO_RAD[0])
        q2 = math.degrees(float(p2) * self.RAD_PER_REV / float(ppr) + self.JOINT_ZERO_RAD[1])
        return self.kinematics.forward(q1, q2)

    def start_recording(self):
        self.teach_data = []
        self.is_recording = True
        self.log_display.append("<font color='green'>录制开始</font>")

    def stop_recording(self):
        self.is_recording = False
        self.log_display.append(f"录制结束")

    def _teach_mode(self):
        widget = getattr(self, "teach_mode_combo", None)
        return widget.currentText() if widget is not None else "直线模式"

    def _append_teach_point(self, point, source):
        point = (float(point[0]), float(point[1]))
        if not self.validate_trajectory_points([point], f"{source}"):
            return False
        if self.teach_points and math.hypot(point[0] - self.teach_points[-1][0], point[1] - self.teach_points[-1][1]) < 0.001:
            self.log_error(f"{source}与上一个目标点重合；请修改 X 或 Y 后再添加")
            return False
        self.teach_points.append(point)
        self.teach_step_index = 0
        self.log_display.append(
            f"<font color='cyan'>{source} P{len(self.teach_points)}: ({point[0]:.2f}, {point[1]:.2f})</font>"
        )
        self.update_teach_status()
        return True

    def add_teach_target_point(self):
        try:
            x = self._read_float(self.teach_target_x, "示教目标 X")
            y = self._read_float(self.teach_target_y, "示教目标 Y")
            self._append_teach_point((x, y), "添加目标点")
        except ValueError as exc:
            self.log_error(f"示教目标点参数错误: {exc}")

    def record_single_point(self):
        self._append_teach_point((self.cur_x, self.cur_y), "记录当前位置")

    def remove_last_teach_point(self):
        if not self.teach_points:
            self.log_error("没有可撤销的示教目标点")
            return
        point = self.teach_points.pop()
        self.teach_step_index = 0
        self.log_display.append(f"已撤销目标点: ({point[0]:.2f}, {point[1]:.2f})")
        self.update_teach_status()

    def clear_teach_points(self):
        self.teach_points = []
        self.teach_data = []
        self.teach_step_index = 0
        self.log_display.append("示教目标点已清空")
        self.update_teach_status()

    def _teach_run_mode(self):
        widget = getattr(self, "teach_run_mode_combo", None)
        return widget.currentText() if widget is not None else "连续"

    def _teach_primitive_count(self, mode, n):
        """示教段数：直线=N-1 段；圆弧三点共端点链式=N//2 段(N 达到最小点数时)。"""
        is_arc = str(mode).startswith("圆弧")
        need = 3 if is_arc else 2
        if n < need:
            return 0
        return n // 2 if is_arc else n - 1

    def update_teach_status(self, *_):
        label = getattr(self, "teach_points_label", None)
        if label is None:
            return
        mode = self._teach_mode()
        run_mode = self._teach_run_mode()
        is_arc = mode.startswith("圆弧")
        need = 3 if is_arc else 2
        points = list(self.teach_points)
        n = len(points)
        summary_points = points[:4]
        summary = " -> ".join(
            f"P{index}({point[0]:.1f},{point[1]:.1f})"
            for index, point in enumerate(summary_points, start=1)
        )
        if n > 4:
            summary += f" -> ... -> P{n}({points[-1][0]:.1f},{points[-1][1]:.1f})"
        total = self._teach_primitive_count(mode, n)
        requirement = "圆弧需≥3点(每段3点,共端点串联)" if is_arc else "直线需≥2点(每段2点)"
        text = f"{mode} / {run_mode}: 已记录 {n} 点，{requirement}"
        if total:
            text += f"，共 {total} 段"
            if run_mode.startswith("单步"):
                cur = (self.teach_step_index % total) + 1
                text += f"，下一步执行 第 {cur}/{total} 段"
        elif n > 0:
            text += f"（点数不足，至少 {need} 点）"
        if summary:
            text += f"\n{summary}"
        label.setText(text)

    def on_teach_mode_changed(self, *_):
        self.teach_step_index = 0
        self.update_teach_status()

    def on_teach_run_mode_changed(self, *_):
        self.teach_step_index = 0
        self.update_teach_status()

    def _teach_line_chain(self, points):
        if len(points) < 2:
            raise ValueError(f"直线示教至少需要 2 个点，当前为 {len(points)} 个")
        segments = []
        for index, (start, end) in enumerate(zip(points, points[1:]), start=1):
            segment = self._make_line_segment(start, end)
            if segment is None:
                raise ValueError(f"直线 P{index} 与 P{index + 1} 重合；请删除或修改其中一个点")
            segments.append(segment)
        return segments

    def _teach_arc_chain(self, points):
        """三点圆弧共端点链式：弧1=(P1,P2,P3),弧2=(P3,P4,P5)...；
        点数凑不齐最后一段时循环借用开头的点，确保最后一个示教点被经过。"""
        n = len(points)
        if n < 3:
            raise ValueError(f"圆弧示教至少需要 3 个点，当前为 {n} 个")
        segments = []
        s = 0
        guard = 0
        while True:
            a = points[s % n]
            b = points[(s + 1) % n]
            c = points[(s + 2) % n]
            try:
                segments.append(self.path_planner.arc_segment_3pt(a, b, c))
            except ValueError as exc:
                raise ValueError(
                    f"第 {len(segments) + 1} 段圆弧(P{s % n + 1},P{(s + 1) % n + 1},P{(s + 2) % n + 1}): {exc}"
                )
            if s + 2 >= n - 1:  # 本段窗口已覆盖最后一个示教点
                break
            s += 2
            guard += 1
            if guard > n:  # 安全兜底，避免异常情况下死循环
                break
        return segments

    def _teach_primitive_segments(self, mode, points):
        """按运动模式构造有序基元段：直线=逐段折线；圆弧=三点共端点链式(不足循环补齐)。"""
        if str(mode).startswith("圆弧"):
            return self._teach_arc_chain(points)
        return self._teach_line_chain(points)

    def build_teach_motion(self, mode, run_mode, points, speed_max, step_index=0):
        if float(speed_max) <= 0.0:
            raise ValueError(f"运行速度必须大于 0，当前为 {float(speed_max):g}")
        clean_points = [(float(point[0]), float(point[1])) for point in points]
        primitives = self._teach_primitive_segments(mode, clean_points)
        if not primitives:
            raise ValueError("没有可执行的示教段")
        base = "示教圆弧" if str(mode).startswith("圆弧") else "示教直线"
        total = len(primitives)
        if str(run_mode).startswith("单步"):
            idx = step_index % total
            segments = [primitives[idx]]
            label = f"{base}·单步 第{idx + 1}/{total}段"
        else:
            segments = primitives
            label = f"{base}·连续 共{total}段"
        path, send_path = self.generate_geometry_motion(segments, float(speed_max), label=label)
        if not path or not send_path:
            raise ValueError(f"{label}未生成可发送轨迹；请根据终端超限提示修改目标点")
        return path, send_path, label, total

    def _read_teach_motion_config(self):
        mode = self._teach_mode()
        run_mode = self._teach_run_mode()
        speed = self._read_float(self.hw_speed_input, "运行速度", positive=True)
        return mode, run_mode, speed

    def preview_teach_path(self):
        try:
            mode, run_mode, speed = self._read_teach_motion_config()
            # 预览始终展开整条示教轨迹(连续)，便于查看完整形状。
            path, send_path, label, total = self.build_teach_motion(mode, "连续", self.teach_points, speed)
            self.preview_planned_path(path, f"{label}(预览)")
            self.log_display.append(
                f"<font color='cyan'>示教预览完成: {len(self.teach_points)} 点 / {total} 段，"
                f"预览 {len(path)} 采样点</font>"
            )
        except ValueError as exc:
            self.log_error(f"示教轨迹参数错误: {exc}")
        except Exception as exc:
            self.log_error(f"示教轨迹预览错误: {exc}")

    def start_playback(self):
        try:
            mode, run_mode, speed = self._read_teach_motion_config()
            single = run_mode.startswith("单步")
            idx = self.teach_step_index if single else 0
            path, send_path, label, total = self.build_teach_motion(
                mode, run_mode, self.teach_points, speed, step_index=idx
            )
            self.preview_planned_path(path, label)
            loaded = self.load_motion_gcode_job(send_path, preview_path=path)
            if not loaded:
                return
            if single:
                ran = idx % total
                self.teach_step_index = (ran + 1) % total
                tail = "（已到最后一段，下次从第 1 段重新开始）" if self.teach_step_index == 0 else ""
                self.log_display.append(f"<font color='cyan'>{label} 已装载{tail}</font>")
                self.update_teach_status()
            else:
                self.log_display.append(
                    f"<font color='cyan'>{label}已装载: 依次走完 {total} 段并停止</font>"
                )
        except ValueError as exc:
            self.log_error(f"示教轨迹参数错误: {exc}")
        except Exception as exc:
            self.log_error(f"示教轨迹执行错误: {exc}")

    def preview_planned_path(self, path, label):
        if hasattr(self, "set_planned_preview"):
            self.set_planned_preview(path, label)

    def plan_trajectory(self, silent=False):
        try:
            tx = self._read_float(self.target_x, "目标X")
            ty = self._read_float(self.target_y, "目标Y")
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            mode = self.mode_combo.currentText()
            start = (self.cur_x, self.cur_y)
            end = (tx, ty)

            if mode.startswith("G1"):
                path = self.generate_linear_path(self.cur_x, self.cur_y, tx, ty, v, silent=silent)
                send_path = self.generate_geometry_gcode([self._make_line_segment(start, end)], v, start=start)
                label = "G1 直线"
            elif mode.startswith("G2") or mode.startswith("G3"):
                radius = self._read_float(self.radius_r, "圆弧半径", positive=True)
                clockwise = mode.startswith("G2")
                path = self.generate_arc_path(
                    start[0],
                    start[1],
                    end[0],
                    end[1],
                    radius,
                    clockwise=clockwise,
                    speed_max=v,
                    silent=silent,
                )
                send_path = self.generate_geometry_gcode(
                    [self._make_arc_segment(start, end, radius, clockwise)],
                    v,
                    start=start,
                )
                label = "G2 顺圆" if mode.startswith("G2") else "G3 逆圆"
            else:
                raise ValueError(f"不支持的轨迹模式: {mode}")

            if not path or not send_path:
                self.log_error(f"{label}未生成可发送轨迹")
                return
            self.preview_planned_path(path, label)
            self.load_motion_gcode_job(send_path, preview_path=path)
            self.log_display.append(
                f"<font color='cyan'>{label}规划完成: 预览 {len(path)} 点，下发关键点 {len(send_path)} 点，目标 X={tx:.1f}, Y={ty:.1f}</font>"
            )
        except ValueError as e:
            self.log_error(f"轨迹参数错误: {e}")
        except Exception as e:
            self.log_error(f"轨迹规划错误: {e}")

    def _make_line_segment(self, p0, p1):
        return self.path_planner.line_segment(p0, p1)

    def _make_arc_segment(self, p0, p1, radius, clockwise):
        return self.path_planner.arc_segment(p0, p1, radius, clockwise=clockwise)

    def _offset_point(self, origin, point):
        return (origin[0] + point[0], origin[1] + point[1])

    def _line(self, origin, p0, p1):
        return self._make_line_segment(self._offset_point(origin, p0), self._offset_point(origin, p1))

    def _arc(self, origin, p0, p1, radius, clockwise):
        return self._make_arc_segment(self._offset_point(origin, p0), self._offset_point(origin, p1), radius, clockwise)

    def _rounded_chain(self, origin, points, radius=None, keep_corners=None):
        radius = self.DEFAULT_CORNER_RADIUS_MM if radius is None else radius
        keep_corners = set(keep_corners or [])
        if not keep_corners:
            absolute = [self._offset_point(origin, point) for point in points]
            return self.path_planner.rounded_polyline_segments(absolute, corner_radius_mm=radius)

        segments = []
        chunk = [points[0]]
        for index in range(1, len(points)):
            chunk.append(points[index])
            if index in keep_corners and len(chunk) >= 2:
                absolute = [self._offset_point(origin, point) for point in chunk]
                segments.extend(self.path_planner.rounded_polyline_segments(absolute, corner_radius_mm=radius))
                chunk = [points[index]]
        if len(chunk) >= 2:
            absolute = [self._offset_point(origin, point) for point in chunk]
            segments.extend(self.path_planner.rounded_polyline_segments(absolute, corner_radius_mm=radius))
        return segments

    def build_car1_segments(self, x0, y0):
        origin = (x0, y0)
        raw = []
        raw.extend(
            self._rounded_chain(
                origin,
                [(0, 0), (0, 24), (72, 24), (72, 48), (108, 48), (120, 36), (120, 0), (108, 0)],
                radius=self.CAR_CORNER_RADIUS_MM,
            )
        )
        raw.append(self._arc(origin, (108, 0), (84, 0), 12, clockwise=True))
        raw.append(self._line(origin, (84, 0), (48, 0)))
        raw.append(self._arc(origin, (48, 0), (24, 0), 12, clockwise=True))
        raw.append(self._line(origin, (24, 0), (0, 0)))
        return [segment for segment in raw if segment is not None]

    def build_car2_segments(self, x0, y0):
        origin = (x0, y0)
        raw = []
        raw.extend(
            self._rounded_chain(
                origin,
                [(0, 0), (0, 20), (40, 20), (60, 40), (120, 40), (140, 20), (160, 20), (160, 0), (140, 0)],
                radius=self.CAR_CORNER_RADIUS_MM,
            )
        )
        raw.append(self._arc(origin, (140, 0), (116, 0), 12, clockwise=True))
        raw.append(self._line(origin, (116, 0), (44, 0)))
        raw.append(self._arc(origin, (44, 0), (20, 0), 12, clockwise=True))
        raw.append(self._line(origin, (20, 0), (0, 0)))
        return [segment for segment in raw if segment is not None]

    def plan_car_path(self):
        try:
            x0 = self._read_float(self.car_start_x, "小车起始X")
            y0 = self._read_float(self.car_start_y, "小车起始Y")
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            path, send_path = self.generate_geometry_motion(
                self.build_car1_segments(x0, y0),
                v,
                label="小车轨迹1",
                exact_stop_dwell_ms=self.CAR_EXACT_STOP_DWELL_MS,
            )
            if not path or not send_path:
                return
            self.preview_planned_path(path, "小车轨迹1")
            self.load_motion_gcode_job(send_path, preview_path=path)
            self.log_display.append(
                f"<font color='cyan'>小车轨迹1规划完成: 预览 {len(path)} 点，下发关键点 {len(send_path)} 点</font>"
            )
        except ValueError as e:
            self.log_error(f"小车轨迹1参数错误: {e}")
        except Exception as e:
            self.log_error(f"小车轨迹1规划错误: {e}")

    def plan_car2_path(self):
        try:
            x0 = self._read_float(self.car_start_x, "小车起始X")
            y0 = self._read_float(self.car_start_y, "小车起始Y")
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            path, send_path = self.generate_geometry_motion(
                self.build_car2_segments(x0, y0),
                v,
                label="小车轨迹2",
                exact_stop_dwell_ms=self.CAR_EXACT_STOP_DWELL_MS,
            )
            if not path or not send_path:
                return
            self.preview_planned_path(path, "小车轨迹2")
            self.load_motion_gcode_job(send_path, preview_path=path)
            self.log_display.append(
                f"<font color='cyan'>小车轨迹2规划完成: 预览 {len(path)} 点，下发关键点 {len(send_path)} 点</font>"
            )
        except ValueError as e:
            self.log_error(f"小车轨迹2参数错误: {e}")
        except Exception as e:
            self.log_error(f"小车轨迹2规划错误: {e}")

    def _stroke_geometry_groups(
        self,
        strokes,
        label="writing path",
        simplify_tolerance=None,
        min_spacing=None,
        corner_radius_mm=None,
        optimize_closed_start=False,
    ):
        """Convert writing strokes into true sharp-corner line groups."""
        self._sync_preview_planner_profile()
        simplify_tolerance = self.PATH_SIMPLIFY_TOLERANCE_MM if simplify_tolerance is None else simplify_tolerance
        min_spacing = self.PATH_MIN_POINT_SPACING_MM if min_spacing is None else min_spacing
        current = (float(self.cur_x), float(self.cur_y))
        groups = []

        for raw_stroke in strokes:
            was_closed = self._is_closed_stroke(raw_stroke, threshold=max(0.01, min_spacing * 1.5))
            stroke = self.preprocess_control_points(
                raw_stroke,
                simplify_tolerance=simplify_tolerance,
                min_spacing=min_spacing,
            )
            if was_closed and len(stroke) >= 3 and not self._is_closed_stroke(
                stroke, threshold=max(0.01, min_spacing * 1.5)
            ):
                stroke.append(stroke[0])
            if len(stroke) < 2:
                continue
            if optimize_closed_start and self._is_closed_stroke(stroke, threshold=max(0.01, min_spacing * 1.5)):
                stroke = self._rotate_closed_stroke_near_current(stroke, current)
            elif math.hypot(stroke[-1][0] - current[0], stroke[-1][1] - current[1]) < math.hypot(
                stroke[0][0] - current[0], stroke[0][1] - current[1]
            ):
                stroke = list(reversed(stroke))
            if not self.validate_trajectory_points(stroke, f"{label} control points"):
                return []
            # Do not replace a writing corner with a fillet. Every retained
            # endpoint is reached exactly, while GRBL junction deviation selects
            # a safe non-zero transition speed from the real corner angle.
            segments = self.path_planner.rounded_polyline_segments(stroke, corner_radius_mm=0.0)
            if segments:
                groups.append(segments)
                current = tuple(segments[-1].end)

        if not groups:
            self.log_error(f"{label} has no valid geometry.")
        return groups

    def _iter_stroke_geometry_gcode(self, groups, speed_max, start=None):
        """Yield sharp-corner writing geometry for GRBL junction planning."""
        cursor = tuple(start or (self.cur_x, self.cur_y))
        for index, segments in enumerate(groups):
            first = tuple(segments[0].start)
            connector_needed = math.hypot(first[0] - cursor[0], first[1] - cursor[1]) > 0.01
            if index > 0:
                yield "G4 P0.001"
            if connector_needed:
                x, y = self.ui_to_mcu_xy(float(first[0]), float(first[1]))
                yield f"G0 X{x:.3f} Y{y:.3f}"
                yield "G4 P0.001"
            yield from self._iter_geometry_gcode(segments, speed_max, start=first)
            cursor = tuple(segments[-1].end)

    def _stroke_geometry_command_count(self, groups, start=None):
        cursor = tuple(start or (self.cur_x, self.cur_y))
        count = 0
        for index, segments in enumerate(groups):
            first = tuple(segments[0].start)
            if index > 0:
                count += 1
            if math.hypot(first[0] - cursor[0], first[1] - cursor[1]) > 0.01:
                count += 2
            count += len(segments)
            cursor = tuple(segments[-1].end)
        return count

    def generate_stroke_motion(
        self,
        strokes,
        speed_max,
        label="writing path",
        simplify_tolerance=None,
        min_spacing=None,
        corner_radius_mm=None,
        optimize_closed_start=False,
    ):
        """Build a sampled preview and a compact GRBL geometry stream."""
        groups = self._stroke_geometry_groups(
            strokes,
            label=label,
            simplify_tolerance=simplify_tolerance,
            min_spacing=min_spacing,
            corner_radius_mm=corner_radius_mm,
            optimize_closed_start=optimize_closed_start,
        )
        if not groups:
            return [], (), 0

        preview = []
        cursor = (float(self.cur_x), float(self.cur_y))
        for segments in groups:
            first = tuple(segments[0].start)
            if math.hypot(first[0] - cursor[0], first[1] - cursor[1]) > 0.01:
                connector = self.generate_linear_path(cursor[0], cursor[1], first[0], first[1], speed_max, silent=True)
                if not connector:
                    return [], (), 0
                preview.extend(connector)
            body = self.generate_geometry_path(segments, speed_max, silent_first=False, label=label)
            if not body:
                return [], (), 0
            preview.extend(body)
            cursor = tuple(segments[-1].end)

        from ..communication.motion_senders import CountedCommandStream

        command_count = self._stroke_geometry_command_count(groups, start=(self.cur_x, self.cur_y))
        command_source = CountedCommandStream(
            self._iter_stroke_geometry_gcode(groups, speed_max),
            command_count,
        )
        return preview, command_source, command_count

    def generate_stroke_path(
        self,
        strokes,
        speed_max,
        label="写字轨迹",
        simplify_tolerance=None,
        min_spacing=None,
        corner_radius_mm=None,
        optimize_closed_start=False,
    ):
        preview, _, _ = self.generate_stroke_motion(
            strokes,
            speed_max,
            label=label,
            simplify_tolerance=simplify_tolerance,
            min_spacing=min_spacing,
            corner_radius_mm=corner_radius_mm,
            optimize_closed_start=optimize_closed_start,
        )
        return preview

    def _is_closed_stroke(self, stroke, threshold=0.35):
        if len(stroke) < 3:
            return False
        return math.hypot(stroke[0][0] - stroke[-1][0], stroke[0][1] - stroke[-1][1]) <= threshold

    def _rotate_closed_stroke_near_current(self, stroke, current):
        points = list(stroke)
        if len(points) >= 2 and self._is_closed_stroke(points):
            points = points[:-1]
        if len(points) < 3:
            return list(stroke)

        best = None
        for candidate in (points, list(reversed(points))):
            for index, point in enumerate(candidate):
                distance = math.hypot(point[0] - current[0], point[1] - current[1])
                if best is None or distance < best[0]:
                    rotated = candidate[index:] + candidate[:index]
                    best = (distance, rotated)
        rotated = best[1]
        return rotated + [rotated[0]]

    def _signed_area(self, points):
        area = 0.0
        for p0, p1 in zip(points, points[1:]):
            area += p0[0] * p1[1] - p1[0] * p0[1]
        return area * 0.5

    def _stroke_bounds(self, stroke):
        xs = [point[0] for point in stroke]
        ys = [point[1] for point in stroke]
        return min(xs), max(xs), min(ys), max(ys)

    def _order_text_strokes(self, strokes):
        boxed = []
        for stroke in strokes:
            xs = [point[0] for point in stroke]
            ys = [point[1] for point in stroke]
            boxed.append(
                {
                    "stroke": stroke,
                    "min_x": min(xs),
                    "max_x": max(xs),
                    "min_y": min(ys),
                    "max_y": max(ys),
                    "center_y": (min(ys) + max(ys)) * 0.5,
                }
            )
        if not boxed:
            return []

        avg_height = sum(item["max_y"] - item["min_y"] for item in boxed) / max(1, len(boxed))
        row_threshold = max(6.0, avg_height * 0.45)
        rows = []
        for item in sorted(boxed, key=lambda x: -x["center_y"]):
            for row in rows:
                if abs(row["center_y"] - item["center_y"]) <= row_threshold:
                    row["items"].append(item)
                    row["center_y"] = sum(member["center_y"] for member in row["items"]) / len(row["items"])
                    break
            else:
                rows.append({"center_y": item["center_y"], "items": [item]})

        ordered = []
        for row in sorted(rows, key=lambda x: -x["center_y"]):
            for item in sorted(row["items"], key=lambda x: (x["min_x"], -x["max_y"])):
                ordered.append(item["stroke"])
        return ordered

    def handwriting_strokes_to_robot(self, strokes):
        robot_strokes = []
        width_mm = 120.0
        height_mm = 85.0
        left = self.DRAW_CENTER_X - width_mm * 0.5
        bottom = self.DRAW_CENTER_Y - height_mm * 0.5
        for stroke in strokes:
            converted = []
            for x_norm, y_norm in stroke:
                converted.append((left + float(x_norm) * width_mm, bottom + (1.0 - float(y_norm)) * height_mm))
            if len(converted) >= 2:
                robot_strokes.append(converted)
        return robot_strokes

    def plan_handwriting_preview(self):
        try:
            strokes = self.handwriting_strokes_to_robot(self.handwriting_pad.normalized_strokes())
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            path, _, command_count = self.generate_stroke_motion(strokes, v, label="handwriting")
            if path:
                self.preview_planned_path(path, "手写轨迹")
                self.log_display.append(
                    f"<font color='cyan'>Handwriting preview: {len(path)} preview points, "
                    f"{command_count} geometry commands.</font>"
                )
        except Exception as e:
            self.log_error(f"手写轨迹预览错误: {e}")

    def plan_handwriting_path(self):
        try:
            strokes = self.handwriting_strokes_to_robot(self.handwriting_pad.normalized_strokes())
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            path, commands, command_count = self.generate_stroke_motion(strokes, v, label="handwriting")
            if path:
                self.preview_planned_path(path, "手写轨迹")
                self.load_motion_gcode_job(commands, preview_path=path)
                self.log_display.append(
                    f"<font color='cyan'>Handwriting loaded: {len(path)} preview points, "
                    f"{command_count} geometry commands.</font>"
                )
        except Exception as e:
            self.log_error(f"手写轨迹规划错误: {e}")

    def clear_handwriting(self):
        if hasattr(self, "handwriting_pad"):
            self.handwriting_pad.clear()
        self.preview_x, self.preview_y = [], []
        self.preview_label = ""
        self.update_plot()

    @staticmethod
    def _point_line_distance(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-12:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        return abs(dy * point[0] - dx * point[1] + end[0] * start[1] - end[1] * start[0]) / length

    def _flatten_cubic_bezier(self, p0, p1, p2, p3, tolerance):
        """Adaptively flatten one real font curve without crossing its endpoints."""
        tolerance = max(0.001, float(tolerance))
        points = []
        pending = [(p0, p1, p2, p3, 0)]
        while pending:
            a, b, c, d, depth = pending.pop()
            flatness = max(self._point_line_distance(b, a, d), self._point_line_distance(c, a, d))
            if flatness <= tolerance or depth >= 12:
                points.append(d)
                continue
            ab = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
            bc = ((b[0] + c[0]) * 0.5, (b[1] + c[1]) * 0.5)
            cd = ((c[0] + d[0]) * 0.5, (c[1] + d[1]) * 0.5)
            abc = ((ab[0] + bc[0]) * 0.5, (ab[1] + bc[1]) * 0.5)
            bcd = ((bc[0] + cd[0]) * 0.5, (bc[1] + cd[1]) * 0.5)
            mid = ((abc[0] + bcd[0]) * 0.5, (abc[1] + bcd[1]) * 0.5)
            pending.append((mid, bcd, cd, d, depth + 1))
            pending.append((a, ab, abc, mid, depth + 1))
        return points

    def _qt_path_contours(self, painter_path, transform):
        """Extract semantic font contours, preserving every true LineTo corner."""
        contours = []
        contour = []
        current = None
        index = 0
        while index < painter_path.elementCount():
            element = painter_path.elementAt(index)
            point = transform(float(element.x), float(element.y))
            if element.isMoveTo():
                if len(contour) >= 2:
                    contours.append(contour)
                contour = [point]
                current = point
            elif element.isLineTo():
                if current is None:
                    contour = [point]
                elif math.hypot(point[0] - current[0], point[1] - current[1]) > 0.001:
                    contour.append(point)
                current = point
            elif element.isCurveTo() and current is not None and index + 2 < painter_path.elementCount():
                control2 = painter_path.elementAt(index + 1)
                endpoint = painter_path.elementAt(index + 2)
                p2 = transform(float(control2.x), float(control2.y))
                p3 = transform(float(endpoint.x), float(endpoint.y))
                contour.extend(
                    self._flatten_cubic_bezier(current, point, p2, p3, self.TEXT_CURVE_TOLERANCE_MM)
                )
                current = p3
                index += 2
            index += 1
        if len(contour) >= 2:
            contours.append(contour)
        return contours

    def build_text_outline_strokes(
        self,
        text,
        font_family=None,
        height_mm=80.0,
        max_width_mm=135.0,
        max_height_mm=85.0,
    ):
        from PySide6.QtGui import QFont, QFontDatabase, QFontMetricsF, QPainterPath
        from PySide6.QtWidgets import QApplication

        if QApplication.instance() is None:
            self._text_outline_qt_app = QApplication([])

        text = str(text).strip()
        if not text:
            return []

        try:
            families = set(QFontDatabase.families())
        except TypeError:
            families = set(QFontDatabase().families())
        if font_family and font_family in families:
            family = font_family
        else:
            family = "Microsoft YaHei" if "Microsoft YaHei" in families else QFont().family()

        font = QFont(family, 100)
        metrics = QFontMetricsF(font)
        line_step = max(110.0, metrics.lineSpacing() * 1.15)
        raw_items = []
        bounds_list = []

        for line_index, line in enumerate(text.splitlines() or [text]):
            x_cursor = 0.0
            y_offset = line_index * line_step
            for char_index, char in enumerate(line):
                if char.isspace():
                    x_cursor += max(35.0, metrics.horizontalAdvance(char))
                    continue

                painter_path = QPainterPath()
                painter_path.addText(x_cursor, y_offset, font, char)
                if not painter_path.isEmpty():
                    bounds = painter_path.boundingRect()
                    bounds_list.append(bounds)
                    raw_items.append({"line": line_index, "char": char_index, "path": painter_path})

                advance = metrics.horizontalAdvance(char)
                if advance <= 1.0:
                    bounds = painter_path.boundingRect()
                    advance = bounds.width() + 8.0
                x_cursor += max(advance, 24.0)

        if not bounds_list:
            return []

        min_x = min(bounds.left() for bounds in bounds_list)
        max_x = max(bounds.right() for bounds in bounds_list)
        min_y = min(bounds.top() for bounds in bounds_list)
        max_y = max(bounds.bottom() for bounds in bounds_list)
        text_w = max(1.0, max_x - min_x)
        text_h = max(1.0, max_y - min_y)
        target_h = min(float(height_mm), float(max_height_mm))
        scale = min(float(max_width_mm) / text_w, target_h / text_h)
        out_w = text_w * scale
        out_h = text_h * scale
        left = self.DRAW_CENTER_X - out_w * 0.5
        bottom = self.DRAW_CENTER_Y - out_h * 0.5

        strokes = []
        for item in sorted(raw_items, key=lambda value: (value["line"], value["char"])):
            converted_strokes = []
            transform = lambda x, y: (left + (x - min_x) * scale, bottom + (max_y - y) * scale)
            for converted in self._qt_path_contours(item["path"], transform):
                clean = self.preprocess_control_points(
                    converted,
                    simplify_tolerance=0.0,
                    min_spacing=0.001,
                )
                if len(clean) >= 3:
                    if not self._is_closed_stroke(clean, threshold=self.TEXT_MIN_POINT_SPACING_MM * 1.5):
                        clean.append(clean[0])
                    min_sx, max_sx, min_sy, max_sy = self._stroke_bounds(clean)
                    converted_strokes.append(
                        {
                            "stroke": clean,
                            "area": abs(self._signed_area(clean)),
                            "min_x": min_sx,
                            "max_x": max_sx,
                            "min_y": min_sy,
                            "max_y": max_sy,
                        }
                    )

            for contour in sorted(converted_strokes, key=lambda value: (-value["area"], value["min_x"], -value["max_y"])):
                strokes.append(contour["stroke"])
        return strokes

    def _read_text_outline_options(self, override_text=None):
        text = override_text if override_text is not None else self.text_outline_input.text()
        text = str(text).strip()
        if not text:
            raise ValueError("文字内容不能为空")

        font_family = None
        if hasattr(self, "text_font_combo"):
            font_family = self.text_font_combo.currentFont().family()

        height_mm = 80.0
        if hasattr(self, "text_height_input"):
            height_mm = self._read_float(self.text_height_input, "文字高度", positive=True)
        return text, font_family, height_mm

    def _plan_text_outline(self, text=None, load=False):
        v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
        text, font_family, height_mm = self._read_text_outline_options(text)
        strokes = self.build_text_outline_strokes(text, font_family=font_family, height_mm=height_mm)
        path, commands, command_count = self.generate_stroke_motion(
            strokes,
            v,
            label=f"空心字{text}",
            simplify_tolerance=self.TEXT_SIMPLIFY_TOLERANCE_MM,
            min_spacing=self.TEXT_MIN_POINT_SPACING_MM,
            corner_radius_mm=self.TEXT_CORNER_RADIUS_MM,
            optimize_closed_start=True,
        )
        if path:
            self.preview_planned_path(path, f"空心字{text}")
            if load:
                self.load_motion_gcode_job(commands, preview_path=path)
                self.log_display.append(
                    f"<font color='cyan'>Text outline loaded: {len(path)} preview points, "
                    f"{command_count} geometry commands.</font>"
                )
            else:
                self.log_display.append(
                    f"<font color='cyan'>Text outline preview: {len(path)} preview points, "
                    f"{command_count} geometry commands.</font>"
                )

    def preview_text_outline_path(self):
        try:
            self._plan_text_outline(load=False)
        except Exception as e:
            self.log_error(f"空心字预览错误: {e}")

    def plan_text_outline_path(self):
        try:
            self._plan_text_outline(load=True)
        except Exception as e:
            self.log_error(f"空心字规划错误: {e}")

    def plan_fixed_text_path(self, text):
        try:
            self._plan_text_outline(text=text, load=True)
        except Exception as e:
            self.log_error(f"空心字{text}规划错误: {e}")

    def build_svg_outline_strokes(self, svg_path):
        """解析 SVG 矢量线条 → 等比缩放居中到画布 → 折线笔画列表。

        复用空心字的 ``_qt_path_contours`` 轮廓→折线管线：SVG 与字体同为 Y 向下，
        机器人 Y 向上，故 transform 对 y 做 (max_y - y) 翻转。
        """
        from ..trajectory.svg_loader import load_svg_paths

        painter_paths = load_svg_paths(svg_path)
        if not painter_paths:
            return []
        rects = [p.boundingRect() for p in painter_paths]
        min_x = min(r.left() for r in rects)
        max_x = max(r.right() for r in rects)
        min_y = min(r.top() for r in rects)
        max_y = max(r.bottom() for r in rects)
        src_w = max(1e-6, max_x - min_x)
        src_h = max(1e-6, max_y - min_y)
        scale = min(self.PATTERN_MAX_WIDTH_MM / src_w, self.PATTERN_MAX_HEIGHT_MM / src_h)
        out_w, out_h = src_w * scale, src_h * scale
        left = self.DRAW_CENTER_X - out_w * 0.5
        bottom = self.DRAW_CENTER_Y - out_h * 0.5
        transform = lambda x, y: (left + (x - min_x) * scale, bottom + (max_y - y) * scale)

        strokes = []
        for painter_path in painter_paths:
            for contour in self._qt_path_contours(painter_path, transform):
                clean = self.preprocess_control_points(
                    contour,
                    simplify_tolerance=self.TEXT_SIMPLIFY_TOLERANCE_MM,
                    min_spacing=self.TEXT_MIN_POINT_SPACING_MM,
                )
                if len(clean) >= 2:
                    strokes.append(clean)
        return strokes

    def plan_svg_pattern(self, svg_path):
        """运行 SVG 线条图案：解析→规划→预览→下发。出光与否跟随“激光开启”开关。"""
        name = os.path.splitext(os.path.basename(svg_path))[0]
        try:
            v = self._read_run_speed_mm_s()
            strokes = self.build_svg_outline_strokes(svg_path)
            if not strokes:
                self.log_error(f"SVG 图案“{name}”未解析出有效线条")
                return
            path, commands, command_count = self.generate_stroke_motion(
                strokes,
                v,
                label=f"SVG:{name}",
                simplify_tolerance=self.TEXT_SIMPLIFY_TOLERANCE_MM,
                min_spacing=self.TEXT_MIN_POINT_SPACING_MM,
            )
            if not path:
                return
            self.preview_planned_path(path, f"SVG:{name}")
            self.load_motion_gcode_job(commands, preview_path=path)
            self.log_display.append(
                f"<font color='cyan'>SVG 图案“{name}”规划完成: 预览 {len(path)} 点，"
                f"几何指令 {command_count} 条</font>"
            )
        except Exception as e:
            self.log_error(f"SVG 图案“{name}”规划错误: {e}")

    def build_halftone_dots(self, image_path):
        """图片 → 灰度 → 网格采样 → 暗格打点（UI 坐标，居中缩放，蛇形排序）。"""
        import cv2

        # np.fromfile 兼容中文路径（cv2.imread 在中文 Windows 路径上会失败）。
        img = cv2.imdecode(np.fromfile(image_path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise ValueError("无法解码图片（格式不支持或文件损坏）")
        src_h, src_w = img.shape[:2]
        if src_w < 1 or src_h < 1:
            return []
        # 按安全框与点间距决定网格列/行数，并保持图像宽高比。
        cols = max(1, int(self.HALFTONE_MAX_WIDTH_MM / self.HALFTONE_SPACING_MM))
        rows = max(1, int(self.HALFTONE_MAX_HEIGHT_MM / self.HALFTONE_SPACING_MM))
        aspect = src_w / float(src_h)
        if cols / float(rows) > aspect:
            cols = max(1, int(round(rows * aspect)))
        else:
            rows = max(1, int(round(cols / aspect)))
        grid = cv2.resize(img, (cols, rows), interpolation=cv2.INTER_AREA)
        out_w = (cols - 1) * self.HALFTONE_SPACING_MM if cols > 1 else 0.0
        out_h = (rows - 1) * self.HALFTONE_SPACING_MM if rows > 1 else 0.0
        left = self.DRAW_CENTER_X - out_w * 0.5
        bottom = self.DRAW_CENTER_Y - out_h * 0.5
        dots = []
        for row in range(rows):
            # 图片顶行(row=0)对应机器人上方 → Y 翻转。
            y = bottom + (rows - 1 - row) * self.HALFTONE_SPACING_MM
            # 蛇形：偶数行从左到右，奇数行反向，减少行间空程。
            col_iter = range(cols) if row % 2 == 0 else range(cols - 1, -1, -1)
            for col in col_iter:
                if int(grid[row, col]) < self.HALFTONE_THRESHOLD:
                    dots.append((left + col * self.HALFTONE_SPACING_MM, y))
        return dots

    def _halftone_commands(self, dots):
        """逐点激光点阵 G-code：移到点(不出光)→出光→驻点停留→关光。

        首条 ``M5`` 抵消任务级 preamble 自动加的 ``M4``，确保移到首点时不出光。
        真正出光需用户先开“激光开启”(LASER ARM)；未 ARM 时固件不出光，等同安全空走。
        """
        power = int(self._laser_s_word())
        dwell_s = self.HALFTONE_DWELL_MS / 1000.0
        yield "M5"
        for x, y in dots:
            mx, my = self.ui_to_mcu_xy(float(x), float(y))
            yield f"G0 X{mx:.3f} Y{my:.3f}"
            yield f"M4 S{power}"
            yield f"G4 P{dwell_s:.3f}"
            yield "M5"

    def plan_halftone_pattern(self, image_path):
        """运行图片激光点阵：解析→预览点位→下发。出光受激光安全链路控制。"""
        from ..communication.motion_senders import CountedCommandStream

        name = os.path.splitext(os.path.basename(image_path))[0]
        try:
            dots = self.build_halftone_dots(image_path)
            if not dots:
                self.log_error(f"点阵图案“{name}”无打点（图片过浅或阈值过低）")
                return
            if not self.validate_trajectory_points(dots, f"点阵“{name}”"):
                return
            preview = [(x, y, 0.0, False, True) for x, y in dots]
            commands = CountedCommandStream(self._halftone_commands(dots), len(dots) * 4 + 1)
            self.preview_planned_path(preview, f"点阵:{name}")
            self.load_motion_gcode_job(commands, preview_path=preview)
            self.log_display.append(
                f"<font color='cyan'>点阵图案“{name}”规划完成: {len(dots)} 个打点"
                f"（间距 {self.HALFTONE_SPACING_MM:.1f}mm，停留 {self.HALFTONE_DWELL_MS:.0f}ms）；"
                f"出光需开启“激光开启”。</font>"
            )
        except Exception as e:
            self.log_error(f"点阵图案“{name}”规划错误: {e}")

    def preset_trajectory_registry(self):
        """预设轨迹注册表：(显示名, 运行函数) 有序列表。

        固定项之外，自动扫描 ``trajectory/patterns/`` 目录：``.svg`` 作为矢量线条图案、
        ``.png/.jpg/.jpeg/.bmp`` 作为激光点阵图案，文件名即下拉框显示名——
        往该目录丢文件即可新增选项，无需改动代码。以 ``.``/``_`` 开头者忽略。
        """
        items = [
            ("福州大学", lambda: self.plan_fixed_text_path("福州大学")),
            ("FZU", lambda: self.plan_fixed_text_path("FZU")),
        ]
        pattern_dir = os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "trajectory", "patterns")
        )
        if os.path.isdir(pattern_dir):
            for filename in sorted(os.listdir(pattern_dir)):
                if filename.startswith((".", "_")):
                    continue
                full = os.path.join(pattern_dir, filename)
                if not os.path.isfile(full):
                    continue
                name, ext = os.path.splitext(filename)
                ext = ext.lower()
                if ext == ".svg":
                    items.append((f"✏{name}", lambda p=full: self.plan_svg_pattern(p)))
                elif ext in (".png", ".jpg", ".jpeg", ".bmp"):
                    items.append((f"🔥{name}", lambda p=full: self.plan_halftone_pattern(p)))
        return items

    def preset_trajectory_names(self):
        """返回预设轨迹显示名列表，用于填充下拉框。"""
        return [name for name, _ in self.preset_trajectory_registry()]

    def run_selected_preset_trajectory(self):
        """运行下拉框当前选中的预设轨迹。"""
        registry = self.preset_trajectory_registry()
        idx = self.preset_traj_combo.currentIndex()
        if 0 <= idx < len(registry):
            # 按下拉框顺序索引分发：combo 与注册表同源同序，名称/handler 永不错位。
            registry[idx][1]()
        else:
            self.log_error("请先选择一个有效的预设轨迹")

    def system_reset_simulated(self):
        """仿真回零：不播放 UI 本地动画，只让下位机 HOME_SIM 运动并回传 MPos:x,y。"""
        if self.waiting_for_ack or self.point_queue or getattr(self, "inflight_lines", None):
            self.log_error("当前流式任务尚未结束，不能开始回零；请先停止任务。")
            return
        if hasattr(self, "_force_laser_disarm"):
            self._force_laser_disarm()
        elif hasattr(self, "_reset_laser_task_ui"):
            self._reset_laser_task_ui()

        # 如果之前启动过 UI 本地回零动画，必须停止，避免和回传轨迹叠加。
        timer = getattr(self, "_home_anim_timer", None)
        if timer is not None and timer.isActive():
            timer.stop()

        self._home_anim_frames = []
        self._home_anim_index = 0

        # 清理上位机运动队列，避免回零时继续发送轨迹。
        self.point_queue = []
        self._clear_text_sender_state()
        self.last_sent_motion = None
        self.stream_waiting_buffer = False
        self.timeout_timer.stop()
        self._reset_jog_anchor()

        # 清空反馈轨迹，让这次显示只包含 HOME_SIM 回传轨迹。
        self.feedback_x, self.feedback_y = [], []
        self.preview_x, self.preview_y = [], []
        self.preview_label = ""
        self.history_x, self.history_y = [self.cur_x], [self.cur_y]

        # 进入回零反馈跟随模式：状态帧 MPos:x,y 将驱动当前机械臂姿态。
        self.home_feedback_active = True
        self.is_homed = False
        self.home_sensor_triggered = False

        # 建议自动切到“通讯接收内容”，让主机械臂姿态跟随回传。
        if hasattr(self, "plot_mode_combo"):
            self.plot_mode_combo.setCurrentText("通讯接收内容")

        self.update_plot(force=True)

        if not (self.ser and self.ser.is_open):
            self.log_error("仿真回零需要连接下位机：当前没有串口，无法接收 HOME_SIM 回传轨迹")
            return

        self.log_display.append(
            "<font color='cyan'>启动仿真回零：UI 不播放本地动画，只显示下位机 HOME_SIM 回传轨迹。</font>"
        )

        self.motion_preamble_needed = True
        self.load_gcode_job(["$HS"])

    def system_reset_real(self):
        """实机回零：真实 HOME 开关，UI 只跟随下位机 MPos:x,y 回传。"""
        if self.waiting_for_ack or self.point_queue or getattr(self, "inflight_lines", None):
            self.log_error("当前流式任务尚未结束，不能开始回零；请先停止任务。")
            return
        if hasattr(self, "_force_laser_disarm"):
            self._force_laser_disarm()
        elif hasattr(self, "_reset_laser_task_ui"):
            self._reset_laser_task_ui()

        timer = getattr(self, "_home_anim_timer", None)
        if timer is not None and timer.isActive():
            timer.stop()

        self.point_queue = []
        self._clear_text_sender_state()
        self.last_sent_motion = None
        self.stream_waiting_buffer = False
        self.timeout_timer.stop()
        self._reset_jog_anchor()

        self.feedback_x, self.feedback_y = [], []
        self.preview_x, self.preview_y = [], []
        self.preview_label = ""
        self.history_x, self.history_y = [self.cur_x], [self.cur_y]

        self.home_feedback_active = True
        self.is_homed = False
        self.home_sensor_triggered = False

        if hasattr(self, "plot_mode_combo"):
            self.plot_mode_combo.setCurrentText("通讯接收内容")

        self.update_plot(force=True)

        if not (self.ser and self.ser.is_open):
            self.log_error("实机回零需要连接串口")
            return

        self.log_display.append(
            "<font color='cyan'>启动实机回零：UI 只显示下位机 HOME_REAL 回传轨迹。</font>"
        )

        self.motion_preamble_needed = True
        self.load_gcode_job(["$H"])

    def system_reset(self, simulated=None):
        if simulated is None:
            simulated = bool(getattr(self, "board_only_debug", False))
        if simulated:
            return self.system_reset_simulated()
        return self.system_reset_real()
        
    def stop_motion(self):
        self.point_queue = []
        if hasattr(self, "_clear_text_sender_state"):
            self._clear_text_sender_state()
        else:
            self.waiting_for_ack = False
        self.stream_waiting_buffer = False
        self.last_sent_motion = None
        self.active_preview_path = []
        self.emergency_paused = False
        if hasattr(self, "_reset_jog_anchor"):
            self._reset_jog_anchor()
        if hasattr(self, "_set_emergency_button_paused"):
            self._set_emergency_button_paused(False)
        self.motion_preamble_needed = True
        if hasattr(self, "_begin_controller_reset"):
            self._begin_controller_reset("Stop requested by UI")
        if hasattr(self, "_force_laser_disarm"):
            self._force_laser_disarm()
        elif hasattr(self, "_reset_laser_task_ui"):
            self._reset_laser_task_ui()
        self.timeout_timer.stop()
        ts = self.get_timestamp()
        self.log_display.append(
            f"<font color='#f39c12'>TX {ts} Ctrl-X (reset and clear GRBL stream)</font>"
        )
        if self.ser and self.ser.is_open:
            self.ser.write(b"\x18")
            self.ser.write(b"?")

    def emergency_stop(self):
        if getattr(self, "emergency_paused", False):
            self.emergency_paused = False
            self._set_emergency_button_paused(False)
            if self.ser and self.ser.is_open:
                self.ser.write(b"~")
            self.process_queue()
            return
        self.emergency_paused = True
        self._set_emergency_button_paused(True)
        if hasattr(self, "_force_laser_disarm"):
            self._force_laser_disarm()
        if self.ser and self.ser.is_open:
            self.ser.write(b"!")
        self.log_display.append(
            f"<font color='#e74c3c'>TX {self.get_timestamp()} ! (feed hold, keep GRBL stream)</font>"
        )

    def _set_emergency_button_paused(self, paused):
        button = getattr(self, "btn_emergency_stop", None)
        if button is None:
            return
        if paused:
            button.setText("恢复运动")
            button.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold;")
        else:
            button.setText("急停 (保留队列)")
            button.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")

    def _joint_deg_to_pulse(self, q1, q2):
        p1, p2 = self._joint_deg_to_pulse_float(q1, q2)
        return int(round(p1)), int(round(p2))

    def _joint_deg_to_pulse_float(self, q1, q2):
        ppr = float(int(getattr(self, "current_ppr", 6400) or 6400))
        scale = ppr / self.RAD_PER_REV
        p1 = (math.radians(float(q1)) - self.JOINT_ZERO_RAD[0]) * scale
        p2 = (math.radians(float(q2)) - self.JOINT_ZERO_RAD[1]) * scale
        return p1, p2

    def _current_feedback_pulses(self):
        p1 = getattr(self, "feedback_p1", None)
        p2 = getattr(self, "feedback_p2", None)
        if p1 is not None and p2 is not None:
            return int(p1), int(p2)
        q1, q2 = self.inverse_kinematics(float(self.cur_x), float(self.cur_y))
        if q1 is None or q2 is None:
            raise ValueError("Current feedback pose has no valid IK.")
        return self._joint_deg_to_pulse(q1, q2)

    def _feedback_xy_from_pulses(self, pulses=None):
        p1, p2 = pulses if pulses is not None else self._current_feedback_pulses()
        ppr = int(getattr(self, "current_ppr", 6400) or 6400)
        xy = self._xy_from_pulse(int(p1), int(p2), ppr)
        if xy[0] is None or xy[1] is None:
            raise ValueError(f"Feedback pulses have no valid FK: P={int(p1)},{int(p2)}")
        return float(xy[0]), float(xy[1])

    def _reset_jog_anchor(self):
        self.jog_target_xy = None

    def _trapezoid_profile(self, distance, vmax, accel):
        distance = float(distance)
        vmax = float(vmax)
        accel = float(accel)
        if distance <= 0.0:
            return {
                "distance": 0.0,
                "vmax": 0.0,
                "accel": accel,
                "t_accel": 0.0,
                "t_flat": 0.0,
                "d_accel": 0.0,
                "total_time": 0.0,
                "peak": 0.0,
                "limited": False,
            }
        if vmax <= 0.0 or accel <= 0.0:
            raise ValueError("速度和加速度必须大于 0")
        t_accel = vmax / accel
        d_accel = 0.5 * accel * t_accel * t_accel
        if 2.0 * d_accel >= distance:
            t_accel = math.sqrt(distance / accel)
            peak = accel * t_accel
            return {
                "distance": distance,
                "vmax": vmax,
                "accel": accel,
                "t_accel": t_accel,
                "t_flat": 0.0,
                "d_accel": 0.5 * distance,
                "total_time": 2.0 * t_accel,
                "peak": peak,
                "limited": peak < vmax * 0.999,
            }
        d_flat = distance - 2.0 * d_accel
        t_flat = d_flat / vmax
        return {
            "distance": distance,
            "vmax": vmax,
            "accel": accel,
            "t_accel": t_accel,
            "t_flat": t_flat,
            "d_accel": d_accel,
            "total_time": 2.0 * t_accel + t_flat,
            "peak": vmax,
            "limited": False,
        }

    def _set_jog_motion_status(self, stats):
        self._last_jog_plan_stats = dict(stats)
        label = getattr(self, "jog_pps_label", None)
        if label is None:
            return
        peak = float(stats.get("peak_pps", 0.0))
        target = float(stats.get("target_pps", 0.0))
        period = 1000.0 / peak if peak > 1e-9 else 0.0
        limited = bool(stats.get("limited", False))
        low = peak < self.JOG_MIN_SMOOTH_PPS
        text = f"目标PPS {target:.1f}  峰值PPS {peak:.1f}  周期 {period:.2f}ms"
        if limited:
            text += "  受加速度限制"
        if low:
            text += "  低速步进区"
        label.setText(text)
        if low:
            label.setStyleSheet("color: #f1c40f;")
        elif limited:
            label.setStyleSheet("color: #ffd27f;")
        else:
            label.setStyleSheet("color: #7CFC98;")

    def _axis_circumference_mm(self, motor_id=1):
        arm_mm = max(1.0, float(getattr(self, "L1", 160.0)))
        return 2.0 * math.pi * arm_mm

    def _speed_mm_s_to_pps(self, speed_mm_s, motor_id=1):
        ppr = int(getattr(self, "current_ppr", 6400) or 6400)
        return float(speed_mm_s) * ppr / self._axis_circumference_mm(motor_id)

    def _accel_mm_s2_to_pps2(self, accel_mm_s2, motor_id=1):
        ppr = int(getattr(self, "current_ppr", 6400) or 6400)
        return float(accel_mm_s2) * ppr / self._axis_circumference_mm(motor_id)

    def update_jog_pps_preview(self):
        try:
            speed = self._read_run_speed_mm_s()
            accel = self._read_run_accel_mm_s2()
            step = self._read_jog_step_mm()
            target_pps = self._speed_mm_s_to_pps(speed)
            accel_pps2 = self._accel_mm_s2_to_pps2(accel)
            ppr = int(getattr(self, "current_ppr", 6400) or 6400)
            pulses = max(1.0, abs(step) * ppr / self._axis_circumference_mm())
            profile = self._trapezoid_profile(pulses, target_pps, accel_pps2)
            self._set_jog_motion_status(
                {
                    "target_pps": target_pps,
                    "peak_pps": profile["peak"],
                    "limited": profile["limited"],
                }
            )
        except Exception:
            label = getattr(self, "jog_pps_label", None)
            if label is not None:
                label.setText("目标PPS --  峰值PPS --  周期 --")
                label.setStyleSheet("color: #aaaaaa;")

    def motor_jog_direct(self, motor_id, direction):
        """Map the motor bring-up button onto the formal Cartesian G-code path."""
        return self.motor_jog(motor_id, direction)

    def add_jog_step(self, ux, uy):
        try:
            step = self._read_jog_step_mm()
            self.add_jog(float(ux) * step, float(uy) * step)
        except Exception as e:
            self.log_error(f"点动步长错误: {e}")

    def add_jog(self, dx, dy):
        """Plan an exact-stop Cartesian jog from the authoritative pulse anchor."""
        try:
            if self.waiting_for_ack or self.point_queue:
                self.log_error("The current motion must stop before starting a jog.")
                return
            v_max = self._read_run_speed_mm_s()
            sx, sy = float(self.cur_x), float(self.cur_y)
            tx, ty = sx + float(dx), sy + float(dy)
            if not self.check_workspace_safety(tx, ty):
                self.log_error(f"Jog target is unreachable: X={tx:.3f}, Y={ty:.3f}")
                return
            path = self.generate_linear_path(sx, sy, tx, ty, v_max)
            if not path:
                self.log_error("Jog distance is too short.")
                return
            self.preview_planned_path(path, "GRBL jog")
            command = f"$J=G91 X{float(dx):.3f} Y{float(dy):.3f} F{v_max * 60.0:.0f}"
            self.jog_target_xy = (tx, ty)
            self.load_motion_gcode_job([command], preview_path=path)
            self.log_display.append(
                f"<font color='#ffffff'>GRBL_JOG XY dx={dx:g} dy={dy:g} F={v_max * 60.0:.0f}</font>"
            )
            return
        except Exception as e:
            self.log_error(f"点动错误: {e}")
    
    def motor_jog(self, motor_id, direction):
        """单电机点动。

        空闲时：把单关节转动采样成若干末端点，再走正式 G-code 流式链路。
        队列中：保留旧的 ASCII 追加行为，避免中途打断当前运动。

        参数：
        - motor_id：1 或 2，表示点动哪个主动电机。
        - direction：1 为正向，-1 为反向。
        - half_turn：单次点动角度，单位 deg；需要更小步距可调低此值。
        - num_steps：关节点采样数；调大更贴近单关节曲线，但上传点更多。
        """
        try:
            if self.waiting_for_ack or self.point_queue:
                self.log_error("当前运动尚未停止，单电机点动未发送")
                return
            sx, sy = self.cur_x, self.cur_y
            current_angles = self.inverse_kinematics(sx, sy)
            if current_angles[0] is None or current_angles[1] is None:
                self.log_error("无法获取当前关节角度")
                return
            
            q1, q2 = current_angles
            
            half_turn = 3.0
            new_q1, new_q2 = q1, q2
            
            if motor_id == 1:
                new_q1 = q1 + direction * half_turn
            elif motor_id == 2:
                new_q2 = q2 + direction * half_turn
            else:
                self.log_error(f"未知电机编号: {motor_id}")
                return
            
            new_x, new_y = self.kinematics.forward(new_q1, new_q2)
            if new_x is None or new_y is None:
                self.log_error(f"电机{motor_id}点动目标不可解")
                return

            if not self.check_workspace_safety(new_x, new_y):
                self.log_error(f"电机{motor_id}点动目标不可达: X={new_x:.1f}, Y={new_y:.1f}")
                return

            v_max = self._read_run_speed_mm_s()
            num_steps = 20

            # 按关节角度均匀采样，保证电机点动本质上是一根轴在动，而不是 XY 直线点动。
            joint_points = []
            for i in range(num_steps + 1):
                t = i / num_steps
                if motor_id == 1:
                    jq1 = q1 + t * direction * half_turn
                    jq2 = q2
                else:
                    jq1 = q1
                    jq2 = q2 + t * direction * half_turn
                px, py = self.kinematics.forward(jq1, jq2)
                if px is None or py is None:
                    self.log_error(f"电机{motor_id}点动第{i}个采样点不可解")
                    return
                joint_points.append((px, py))

            # send_path 直接使用这组采样点；下位机会在相邻 G-code 段之间做 planner/segment 衔接。
            path = []
            for i in range(1, len(joint_points)):
                x0, y0 = joint_points[i - 1]
                x1, y1 = joint_points[i]
                path.append((x1, y1, v_max, False))

            if not path:
                self.log_error("电机点动未生成有效轨迹")
                return

            self.preview_planned_path(path, f"电机{motor_id}点动")
            self.load_motion_queue(path, append=False)
            self.log_display.append(
                f"<font color='cyan'>电机{motor_id} {'正向' if direction > 0 else '反向'}点动: ({sx:.1f},{sy:.1f}) -> ({new_x:.1f},{new_y:.1f})</font>"
            )

        except Exception as e:
            self.log_error(f"电机点动错误: {e}")
