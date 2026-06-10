import math
import numpy as np


class HostTimedSegment:
    def __init__(self, p1_abs, p2_abs, duration_ticks, flags=0):
        self.p1_abs = int(p1_abs)
        self.p2_abs = int(p2_abs)
        self.duration_ticks = int(duration_ticks)
        self.flags = int(flags)


class HostTimedBinaryPoint:
    def __init__(self, segment, host_flag, exact_stop_flag):
        ticks = int(segment.duration_ticks)
        if ticks < 1 or ticks > 65535:
            raise ValueError(f"invalid host segment ticks: {ticks}")
        self.p1_abs = int(segment.p1_abs)
        self.p2_abs = int(segment.p2_abs)
        self.v_dom_pps = ticks
        self.flags = int(host_flag) | (int(getattr(segment, "flags", 0)) & int(exact_stop_flag))


class ScaraMotionMixin:
    JOINT_LIMITS_DEG = ((-180.0, 180.0), (0.0, 360.0))
    # 轨迹精度参数说明：
    # - BINARY_LINE_TOLERANCE_MM：上位机把一条直线拆成二进制关键点时，允许的关节空间线性化误差。
    #   调小会更贴合直线，但关键点增多、上传更慢；调大则点数少但末端更容易偏离期望线。
    # - BINARY_ARC_SEGMENT_MM：圆弧关键点弧长间隔。调小圆弧更圆滑，调大会减少通信量。
    # - DEFAULT/CAR_CORNER_RADIUS_MM：默认设为 0，表示所有折线必须到达原始转折点，不提前圆角切入。
    ARC_SEGMENT_MM = 2.0
    BINARY_ARC_SEGMENT_MM = 0.75
    BINARY_GEOMETRY_ARC_SEGMENT_MM = 0.35
    BINARY_LINE_TOLERANCE_MM = 0.45
    BINARY_LINE_MAX_SEGMENT_MM = 10.0
    BINARY_PATH_SIMPLIFY_TOLERANCE_MM = 0.08
    BINARY_MRAD_PER_REV = 6283
    BINARY_ZERO_MRAD = (2251, 890)
    DEFAULT_CORNER_RADIUS_MM = 0.0
    CAR_CORNER_RADIUS_MM = 0.0
    PATH_SIMPLIFY_TOLERANCE_MM = 0.18
    PATH_MIN_POINT_SPACING_MM = 0.20
    TEXT_SIMPLIFY_TOLERANCE_MM = 0.06
    TEXT_MIN_POINT_SPACING_MM = 0.16
    TEXT_CORNER_RADIUS_MM = 0.0
    DRAW_CENTER_X = 75.0
    DRAW_CENTER_Y = 220.0
    BINARY_FLAG_EXACT_STOP = 0x0001
    BINARY_FLAG_CARTESIAN_LINE = 0x0002
    BINARY_FLAG_HOST_SEGMENT = 0x0004
    HOST_DDA_HZ = 10000
    HOST_DDA_MAX_PPS = 10000
    HOST_TRAJECTORY_SLICE_TICKS = 100
    HOST_MIN_SMOOTH_PPS = 50.0
    HOST_DEFAULT_ACCEL_MM_S2 = 100.0

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
            return self.HOST_DEFAULT_ACCEL_MM_S2
        return self._read_float(widget, "运行加速度", positive=True)

    def inverse_kinematics(self, x, y):
        return self.kinematics.inverse(x, y)

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
        d1 = math.hypot(x, y)
        d2 = math.hypot(x - c.base_distance, y)

        if y < c.min_y:
            violations.append(f"点{index}: Y 低于下限 {c.min_y:.1f}mm，超出 {c.min_y - y:.2f}mm")
        if d1 < c.min_anchor_dist:
            violations.append(f"点{index}: 左基座距离低于下限 {c.min_anchor_dist:.1f}mm，超出 {c.min_anchor_dist - d1:.2f}mm")
        if d1 > c.max_anchor_dist:
            violations.append(f"点{index}: 左基座距离超过上限 {c.max_anchor_dist:.1f}mm，超出 {d1 - c.max_anchor_dist:.2f}mm")
        if d2 < c.min_anchor_dist:
            violations.append(f"点{index}: 右基座距离低于下限 {c.min_anchor_dist:.1f}mm，超出 {c.min_anchor_dist - d2:.2f}mm")
        if d2 > c.max_anchor_dist:
            violations.append(f"点{index}: 右基座距离超过上限 {c.max_anchor_dist:.1f}mm，超出 {d2 - c.max_anchor_dist:.2f}mm")

        q1, q2 = self.inverse_kinematics(x, y)
        if q1 is None or q2 is None:
            violations.append(f"点{index}: 五连杆无逆解 X={x:.2f}, Y={y:.2f}")
            return violations

        for axis, angle in ((1, q1), (2, q2)):
            low, high = self.JOINT_LIMITS_DEG[axis - 1]
            if angle < low:
                violations.append(f"点{index}: M{axis} 低于下限 {low:.1f}deg，超出 {low - angle:.2f}deg")
            if angle > high:
                violations.append(f"点{index}: M{axis} 超过上限 {high:.1f}deg，超出 {angle - high:.2f}deg")

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
            violations.append(f"点{index}: 左右主动臂交叉，交叉量 {left_elbow[0] - right_elbow[0]:.2f}mm")
        if left_elbow[1] < 0.0:
            violations.append(f"点{index}: 左主动臂低于基座线，超出 {-left_elbow[1]:.2f}mm")
        if right_elbow[1] < 0.0:
            violations.append(f"点{index}: 右主动臂低于基座线，超出 {-right_elbow[1]:.2f}mm")

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
            self.log_error(f"{label}预检查失败，共 {total} 项超限，已拦截发送")
            for item in violations:
                self.log_error(item)
            if total > len(violations):
                self.log_error(f"其余 {total - len(violations)} 项超限已省略")
            return False
        return True

    def generate_linear_path(self, x1, y1, x2, y2, speed_max, v_start=0.0, v_end=0.0, silent=False):
        planned = self.path_planner.plan_line(
            (x1, y1),
            (x2, y2),
            feed_mm_s=speed_max,
            start_speed=v_start,
            end_speed=v_end,
            silent=silent,
        )
        path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
        if path and not self.validate_trajectory_points(path, "直线路径"):
            return []
        return path

    def generate_arc_path(self, x1, y1, x2, y2, radius, clockwise, speed_max, silent=False):
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
        path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
        if path and not self.validate_trajectory_points(path, "圆弧路径"):
            return []
        return path

    def generate_polyline_path(self, points, speed_max, silent_first=False):
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
        path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
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
        start = points[0]
        end = points[-1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        max_dist = -1.0
        split_index = 0
        for index in range(1, len(points) - 1):
            px, py = points[index]
            if length <= 1e-9:
                dist = math.hypot(px - start[0], py - start[1])
            else:
                dist = abs(dy * px - dx * py + end[0] * start[1] - end[1] * start[0]) / length
            if dist > max_dist:
                max_dist = dist
                split_index = index
        if max_dist > epsilon:
            left = self._rdp_points(points[: split_index + 1], epsilon)
            right = self._rdp_points(points[split_index:], epsilon)
            return left[:-1] + right
        return [start, end]

    def generate_geometry_path(self, segments, speed_max, silent_first=False, label="固定轨迹"):
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
        path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
        if path and not self.validate_trajectory_points(path, f"{label}采样点"):
            return []
        return path

    def generate_binary_targets_for_segments(self, segments, speed_max, start=None, include_connector=True):
        """把几何段转换为下位机二进制关节插补关键点。

        直线段只保留必要关键点，由下位机在关节空间同步插补；圆弧段按
        BINARY_ARC_SEGMENT_MM 采样。这里不做 UI 仿真绘图，只准备要上传给 MCU 的目标点。
        """
        segments = [segment for segment in segments if segment and segment.length > 0.001]
        if not segments:
            return []
        feed_mm_min = float(speed_max) * 60.0
        cursor = (float(start[0]), float(start[1])) if start is not None else tuple(segments[0].start)
        targets = []

        def append_points(points, silent=False, exact_last=True):
            for index, (x, y) in enumerate(points):
                if targets and math.hypot(float(x) - targets[-1][0], float(y) - targets[-1][1]) <= 0.001:
                    continue
                flags = self.BINARY_FLAG_EXACT_STOP if exact_last and index == len(points) - 1 else 0
                targets.append((float(x), float(y), feed_mm_min, silent, flags))

        if include_connector and math.hypot(cursor[0] - segments[0].start[0], cursor[1] - segments[0].start[1]) > 0.01:
            connector = self.generate_binary_line_targets(cursor, segments[0].start, speed_max, silent=True)
            targets.extend(connector)
            cursor = tuple(segments[0].start)

        for segment in segments:
            if math.hypot(cursor[0] - segment.start[0], cursor[1] - segment.start[1]) > 0.01:
                targets.extend(self.generate_binary_line_targets(cursor, segment.start, speed_max, silent=True))
            if segment.kind == "arc":
                count = max(2, int(math.ceil(segment.length / self.BINARY_GEOMETRY_ARC_SEGMENT_MM)))
                points = [segment.point_at(segment.length * i / count) for i in range(1, count + 1)]
                append_points(points, exact_last=True)
            else:
                line_targets = self.generate_binary_line_targets(segment.start, segment.end, speed_max, silent=False)
                for item in line_targets:
                    if targets and math.hypot(float(item[0]) - targets[-1][0], float(item[1]) - targets[-1][1]) <= 0.001:
                        continue
                    targets.append(item)
            cursor = tuple(segment.end)

        if targets and not self.validate_trajectory_points(targets, "二进制图形关键点"):
            return []
        return targets

    def generate_geometry_motion(self, segments, speed_max, label="固定轨迹"):
        """生成固定图形的双轨迹：preview 用于界面显示，send_path 用于 MCU 二进制插补。"""
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
        send_path = self.generate_binary_targets_for_segments(segments, speed_max, start=start, include_connector=True)
        return preview, send_path

    def generate_binary_send_from_path(self, path, speed_max=None, start=None, simplify_tolerance=None):
        if not path:
            return []
        start = (float(start[0]), float(start[1])) if start is not None else (float(self.cur_x), float(self.cur_y))
        tolerance = self.BINARY_PATH_SIMPLIFY_TOLERANCE_MM if simplify_tolerance is None else float(simplify_tolerance)
        result = []
        cursor = start
        chunk = []
        chunk_silent = None

        def point_silent(point):
            return bool(point[3]) if len(point) > 3 else False

        def point_feed_mm_s(point):
            if speed_max is not None:
                return max(0.1, float(speed_max))
            if len(point) > 2:
                return max(0.1, float(point[2]) / 60.0)
            return 1.0

        def append_targets(targets):
            for item in targets:
                if result and math.hypot(float(item[0]) - result[-1][0], float(item[1]) - result[-1][1]) <= 0.001:
                    continue
                result.append(item)

        def nearest_feed(points, target, start_index):
            if not points:
                return max(0.1, float(speed_max or 1.0)), start_index
            best_index = start_index
            best_dist = float("inf")
            for index in range(start_index, len(points)):
                dist = math.hypot(float(points[index][0]) - target[0], float(points[index][1]) - target[1])
                if dist < best_dist:
                    best_dist = dist
                    best_index = index
                elif index > best_index and dist > best_dist:
                    break
            return point_feed_mm_s(points[best_index]), best_index

        def flush_chunk(points, silent):
            nonlocal cursor
            if not points:
                return
            coords = [cursor] + [(float(p[0]), float(p[1])) for p in points]
            simplified = self._rdp_points_preserve_turns(coords, tolerance) if len(coords) > 2 and tolerance > 0.0 else coords
            source_index = 0
            for target in simplified[1:]:
                if math.hypot(target[0] - cursor[0], target[1] - cursor[1]) <= 0.001:
                    continue
                feed_mm_s, source_index = nearest_feed(points, target, source_index)
                append_targets(self.generate_binary_line_targets(cursor, target, feed_mm_s, silent=silent))
                cursor = target

        for point in path:
            silent = point_silent(point)
            if chunk and silent != chunk_silent:
                flush_chunk(chunk, chunk_silent)
                chunk = []
            chunk.append(point)
            chunk_silent = silent
        flush_chunk(chunk, chunk_silent)

        if result and not self.validate_trajectory_points(result, "二进制简化轨迹关键点"):
            return []
        return result

    def _rdp_points_preserve_turns(self, points, epsilon):
        if len(points) <= 2:
            return list(points)
        breaks = [0]
        for index in range(1, len(points) - 1):
            ax, ay = points[index - 1]
            bx, by = points[index]
            cx, cy = points[index + 1]
            v1x, v1y = bx - ax, by - ay
            v2x, v2y = cx - bx, cy - by
            l1 = math.hypot(v1x, v1y)
            l2 = math.hypot(v2x, v2y)
            if l1 <= 1e-6 or l2 <= 1e-6:
                continue
            dot = (v1x * v2x + v1y * v2y) / (l1 * l2)
            if dot < 0.985:
                breaks.append(index)
        breaks.append(len(points) - 1)

        simplified = []
        for start_index, end_index in zip(breaks, breaks[1:]):
            section = points[start_index : end_index + 1]
            reduced = self._rdp_points(section, epsilon) if len(section) > 2 else section
            if simplified:
                simplified.extend(reduced[1:])
            else:
                simplified.extend(reduced)
        return simplified

    def generate_arc_control_points(self, start, end, radius, clockwise):
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        chord = math.hypot(dx, dy)
        if chord < 0.001:
            raise ValueError("圆弧起点和终点重合；当前界面未定义整圆圆心")
        min_radius = chord * 0.5
        if radius < min_radius:
            raise ValueError(f"圆弧半径过小，至少需要 {min_radius:.2f}mm，当前为 {radius:.2f}mm")

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
        segments = max(8, int(math.ceil(arc_len / self.BINARY_ARC_SEGMENT_MM)))
        segments = min(segments, 5000)
        points = []
        for i in range(segments + 1):
            t = i / segments
            a = a0 + delta * t
            points.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
        points[0] = start
        points[-1] = end
        return points

    def generate_binary_arc_targets(self, start, end, radius, clockwise, speed_max, silent=False):
        points = self.generate_arc_control_points(start, end, radius, clockwise)
        if not self.validate_trajectory_points(points, "二进制圆弧关键点"):
            return []
        feed_mm_min = float(speed_max) * 60.0
        targets = []
        for index, (x, y) in enumerate(points[1:], start=1):
            flags = self.BINARY_FLAG_EXACT_STOP if index == len(points) - 1 else 0
            targets.append((float(x), float(y), feed_mm_min, silent, flags))
        return targets

    def _binary_pulse_from_xy(self, point, ppr):
        q1, q2 = self.inverse_kinematics(float(point[0]), float(point[1]))
        if q1 is None or q2 is None:
            return None
        theta1_mrad = int(round(math.radians(q1) * 1000.0))
        theta2_mrad = int(round(math.radians(q2) * 1000.0))
        p1 = int(round(((theta1_mrad - self.BINARY_ZERO_MRAD[0]) * int(ppr)) / self.BINARY_MRAD_PER_REV))
        p2 = int(round(((theta2_mrad - self.BINARY_ZERO_MRAD[1]) * int(ppr)) / self.BINARY_MRAD_PER_REV))
        return p1, p2

    def _binary_xy_from_pulse(self, p1, p2, ppr):
        q1 = math.degrees(((float(p1) * self.BINARY_MRAD_PER_REV / float(ppr)) + self.BINARY_ZERO_MRAD[0]) / 1000.0)
        q2 = math.degrees(((float(p2) * self.BINARY_MRAD_PER_REV / float(ppr)) + self.BINARY_ZERO_MRAD[1]) / 1000.0)
        return self.kinematics.forward(q1, q2)

    def _point_to_line_error(self, point, start, end):
        px, py = point
        ax, ay = start
        bx, by = end
        vx = bx - ax
        vy = by - ay
        den = vx * vx + vy * vy
        if den <= 1e-12:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * vx + (py - ay) * vy) / den
        t = max(0.0, min(1.0, t))
        qx = ax + vx * t
        qy = ay + vy * t
        return math.hypot(px - qx, py - qy)

    def _joint_linearized_line_error(self, start, end, ppr):
        p0 = self._binary_pulse_from_xy(start, ppr)
        p1 = self._binary_pulse_from_xy(end, ppr)
        if p0 is None or p1 is None:
            return float("inf")
        max_error = 0.0
        steps = max(abs(p1[0] - p0[0]), abs(p1[1] - p0[1]), 1)
        if steps <= 64:
            sample_indices = range(1, steps)
        else:
            sample_indices = [int(round(steps * i / 32.0)) for i in range(1, 32)]
        for index in sample_indices:
            t = index / steps
            ip1 = int(round(p0[0] + (p1[0] - p0[0]) * t))
            ip2 = int(round(p0[1] + (p1[1] - p0[1]) * t))
            xy = self._binary_xy_from_pulse(ip1, ip2, ppr)
            if xy[0] is None or xy[1] is None:
                return float("inf")
            max_error = max(max_error, self._point_to_line_error(xy, start, end))
        return max_error

    def _adaptive_binary_line_points(self, start, end, ppr, depth=0):
        """按关节脉冲量化误差递归拆分直线，确保下位机关节插补后仍贴近笛卡尔直线。"""
        length = math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
        error = self._joint_linearized_line_error(start, end, ppr)
        if (
            depth >= 12
            or length <= 0.10
            or (length <= self.BINARY_LINE_MAX_SEGMENT_MM and error <= self.BINARY_LINE_TOLERANCE_MM)
        ):
            return [end]
        mid = ((float(start[0]) + float(end[0])) * 0.5, (float(start[1]) + float(end[1])) * 0.5)
        return (
            self._adaptive_binary_line_points(start, mid, ppr, depth + 1)
            + self._adaptive_binary_line_points(mid, end, ppr, depth + 1)
        )

    def generate_binary_line_targets(self, start, end, speed_max, silent=False):
        """生成一条直线的二进制关键点；实际插补在下位机 10kHz 控制周期内完成。"""
        keypoints = [(float(end[0]), float(end[1]))]
        if not self.validate_trajectory_points(keypoints, "二进制直线关键点"):
            return []
        feed_mm_min = float(speed_max) * 60.0
        flags = self.BINARY_FLAG_CARTESIAN_LINE | self.BINARY_FLAG_EXACT_STOP
        return [(float(x), float(y), feed_mm_min, silent, flags) for x, y in keypoints]

    def start_recording(self):
        self.teach_data = [] 
        self.is_recording = True 
        self.log_display.append("<font color='green'>录制开始</font>")
        
    def stop_recording(self): 
        self.is_recording = False 
        self.log_display.append(f"录制结束")
        
    def record_single_point(self):
        if self.inverse_kinematics(self.cur_x, self.cur_y)[0] is not None:
            self.teach_points.append((self.cur_x,self.cur_y))
            self.log_display.append(f"<font color='cyan'>手动记录点: ({self.cur_x:.1f}, {self.cur_y:.1f})</font>")
                
    def clear_teach_points(self): 
        self.teach_points = []
        self.teach_data = []
        self.log_display.append("轨迹已清空")
    
    def start_playback(self):
        v = float(self.hw_speed_input.text())
        if self.teach_points:
            pts = [(self.cur_x, self.cur_y)] + self.teach_points
            path = self.generate_polyline_path(pts, v, silent_first=True)
            self.preview_planned_path(path, "示教复现")
            send_path = self.generate_binary_send_from_path(path, v)
            self.load_motion_queue(path, send_path=send_path)

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
                send_path = self.generate_binary_line_targets(start, end, v, silent=silent)
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
                send_path = self.generate_binary_arc_targets(start, end, radius, clockwise, v, silent=silent)
                label = "G2 顺圆" if mode.startswith("G2") else "G3 逆圆"
            else:
                raise ValueError(f"不支持的轨迹模式: {mode}")

            if not path or not send_path:
                self.log_error(f"{label}未生成可发送轨迹")
                return
            self.preview_planned_path(path, label)
            self.load_motion_queue(path, send_path=send_path)
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
            path, send_path = self.generate_geometry_motion(self.build_car1_segments(x0, y0), v, label="小车轨迹1")
            if not path or not send_path:
                return
            self.preview_planned_path(path, "小车轨迹1")
            self.load_motion_queue(path, send_path=send_path)
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
            path, send_path = self.generate_geometry_motion(self.build_car2_segments(x0, y0), v, label="小车轨迹2")
            if not path or not send_path:
                return
            self.preview_planned_path(path, "小车轨迹2")
            self.load_motion_queue(path, send_path=send_path)
            self.log_display.append(
                f"<font color='cyan'>小车轨迹2规划完成: 预览 {len(path)} 点，下发关键点 {len(send_path)} 点</font>"
            )
        except ValueError as e:
            self.log_error(f"小车轨迹2参数错误: {e}")
        except Exception as e:
            self.log_error(f"小车轨迹2规划错误: {e}")

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
        path = []
        current = (self.cur_x, self.cur_y)
        valid_strokes = []
        simplify_tolerance = self.PATH_SIMPLIFY_TOLERANCE_MM if simplify_tolerance is None else simplify_tolerance
        min_spacing = self.PATH_MIN_POINT_SPACING_MM if min_spacing is None else min_spacing
        corner_radius_mm = self.DEFAULT_CORNER_RADIUS_MM if corner_radius_mm is None else corner_radius_mm
        for stroke in strokes:
            was_closed = self._is_closed_stroke(stroke, threshold=max(0.01, min_spacing * 1.5))
            clean = self.preprocess_control_points(
                stroke,
                simplify_tolerance=simplify_tolerance,
                min_spacing=min_spacing,
            )
            if was_closed and len(clean) >= 3 and not self._is_closed_stroke(clean, threshold=max(0.01, min_spacing * 1.5)):
                clean.append(clean[0])
            if len(clean) >= 2:
                valid_strokes.append(clean)

        if not valid_strokes:
            self.log_error(f"{label}没有足够的有效笔画")
            return []

        for stroke in valid_strokes:
            if optimize_closed_start and self._is_closed_stroke(stroke, threshold=max(0.01, min_spacing * 1.5)):
                stroke = self._rotate_closed_stroke_near_current(stroke, current)
            elif math.hypot(stroke[-1][0] - current[0], stroke[-1][1] - current[1]) < math.hypot(
                stroke[0][0] - current[0], stroke[0][1] - current[1]
            ):
                stroke = list(reversed(stroke))
            first = stroke[0]
            if math.hypot(first[0] - current[0], first[1] - current[1]) > 0.01:
                connector = self.generate_linear_path(current[0], current[1], first[0], first[1], speed_max, silent=True)
                if not connector:
                    return []
                path.extend(connector)

            if not self.validate_trajectory_points(stroke, f"{label}控制点"):
                return []
            planned = self.path_planner.plan_rounded_polyline(
                stroke,
                feed_mm_s=speed_max,
                corner_radius_mm=corner_radius_mm,
                silent_first=False,
            )
            stroke_path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
            if stroke_path and not self.validate_trajectory_points(stroke_path, f"{label}采样点"):
                return []
            path.extend(stroke_path)
            current = stroke[-1]

        return path

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
            path = self.generate_stroke_path(strokes, v, label="手写轨迹")
            if path:
                self.preview_planned_path(path, "手写轨迹")
                self.log_display.append(f"<font color='cyan'>手写轨迹预览完成: {len(path)} 点</font>")
        except Exception as e:
            self.log_error(f"手写轨迹预览错误: {e}")

    def plan_handwriting_path(self):
        try:
            strokes = self.handwriting_strokes_to_robot(self.handwriting_pad.normalized_strokes())
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            path = self.generate_stroke_path(strokes, v, label="手写轨迹")
            if path:
                self.preview_planned_path(path, "手写轨迹")
                send_path = self.generate_binary_send_from_path(path, v)
                self.load_motion_queue(path, send_path=send_path)
                self.log_display.append(f"<font color='cyan'>手写轨迹已装载: {len(path)} 点</font>")
        except Exception as e:
            self.log_error(f"手写轨迹规划错误: {e}")

    def clear_handwriting(self):
        if hasattr(self, "handwriting_pad"):
            self.handwriting_pad.clear()
        self.preview_x, self.preview_y = [], []
        self.preview_label = ""
        self.update_plot()

    def build_text_outline_strokes(
        self,
        text,
        font_family=None,
        height_mm=80.0,
        max_width_mm=135.0,
        max_height_mm=85.0,
    ):
        from PySide6.QtGui import QFont, QFontDatabase, QFontMetricsF, QPainterPath, QTransform
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
        all_points = []

        for line_index, line in enumerate(text.splitlines() or [text]):
            x_cursor = 0.0
            y_offset = line_index * line_step
            for char_index, char in enumerate(line):
                if char.isspace():
                    x_cursor += max(35.0, metrics.horizontalAdvance(char))
                    continue

                painter_path = QPainterPath()
                painter_path.addText(x_cursor, y_offset, font, char)
                polygons = painter_path.toSubpathPolygons(QTransform())
                char_strokes = []
                for polygon in polygons:
                    points = [(float(point.x()), float(point.y())) for point in polygon]
                    if len(points) < 3:
                        continue
                    if math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1]) > 0.01:
                        points.append(points[0])
                    char_strokes.append(points)
                    all_points.extend(points)

                if char_strokes:
                    raw_items.append({"line": line_index, "char": char_index, "strokes": char_strokes})

                advance = metrics.horizontalAdvance(char)
                if advance <= 1.0:
                    bounds = painter_path.boundingRect()
                    advance = bounds.width() + 8.0
                x_cursor += max(advance, 24.0)

        if not all_points:
            return []

        min_x = min(p[0] for p in all_points)
        max_x = max(p[0] for p in all_points)
        min_y = min(p[1] for p in all_points)
        max_y = max(p[1] for p in all_points)
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
            for raw in item["strokes"]:
                converted = []
                for x, y in raw:
                    rx = left + (x - min_x) * scale
                    ry = bottom + (max_y - y) * scale
                    converted.append((rx, ry))
                clean = self.preprocess_control_points(
                    converted,
                    simplify_tolerance=self.TEXT_SIMPLIFY_TOLERANCE_MM,
                    min_spacing=self.TEXT_MIN_POINT_SPACING_MM,
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
        path = self.generate_stroke_path(
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
                send_path = self.generate_binary_send_from_path(path, v)
                self.load_motion_queue(path, send_path=send_path)
                self.log_display.append(f"<font color='cyan'>空心字{text}已装载: {len(path)} 点</font>")
            else:
                self.log_display.append(f"<font color='cyan'>空心字{text}预览完成: {len(path)} 点</font>")

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

    def system_reset_simulated(self):
        """仿真回零：不播放 UI 本地动画，只让下位机 HOME_SIM 运动并回传 M:x,y。"""

        # 如果之前启动过 UI 本地回零动画，必须停止，避免和回传轨迹叠加。
        timer = getattr(self, "_home_anim_timer", None)
        if timer is not None and timer.isActive():
            timer.stop()

        self._home_anim_frames = []
        self._home_anim_index = 0

        # 清理上位机运动队列，避免回零时继续发送轨迹。
        self.point_queue = []
        self.waiting_for_ack = False
        self.last_sent_motion = None
        self.stream_waiting_buffer = False
        self.timeout_timer.stop()

        # 清空反馈轨迹，让这次显示只包含 HOME_SIM 回传轨迹。
        self.feedback_x, self.feedback_y = [], []
        self.preview_x, self.preview_y = [], []
        self.preview_label = ""
        self.history_x, self.history_y = [self.cur_x], [self.cur_y]

        # 进入回零反馈跟随模式：状态帧 M:x,y 将驱动当前机械臂姿态。
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

        self.send_ascii_line("$X", "HOME_SIM_PREP")
        self.send_ascii_line("M17", "HOME_SIM_PREP")
        self.send_ascii_line("$H S", "HOME_SIM")

    def system_reset_real(self):
        """实机回零：真实 HOME 开关，UI 只跟随下位机 M:x,y 回传。"""

        timer = getattr(self, "_home_anim_timer", None)
        if timer is not None and timer.isActive():
            timer.stop()

        self.point_queue = []
        self.waiting_for_ack = False
        self.last_sent_motion = None
        self.stream_waiting_buffer = False
        self.timeout_timer.stop()

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

        self.send_ascii_line("$X", "HOME_REAL_PREP")
        self.send_ascii_line("M17", "HOME_REAL_PREP")
        self.send_ascii_line("$H", "HOME_REAL")

    def system_reset(self, simulated=None): 
        if simulated is None:
            simulated = bool(getattr(self, "board_only_debug", False))
        if False and self.board_only_debug:
            # 当前仅连接控制板，没有接 HOME 微动开关与电机，先用软件零点完成串口和队列调试。
            # 后续接入真实硬件后，将 board_only_debug 改为 False，即恢复发送 $H 的真实回零流程。
            self.is_homed = True
            self.home_sensor_triggered = False
            self.point_queue = []
            self.waiting_for_ack = False
            self.motion_preamble_needed = True
            self.cur_x, self.cur_y = self.HOME_X, self.HOME_Y
            self.history_x, self.history_y = [self.cur_x], [self.cur_y]
            self.update_plot()
            self.timeout_timer.stop()
            self.log_display.append(
                "<font color='yellow'>BOARD_ONLY_DEBUG: 未接 HOME 开关，跳过真实 $H，发送 ZERO 作为软件零点。</font>"
            )
            self.send_ascii_line("G92 A0 B0", "SOFT_ZERO")
            return
        if self.ser and self.ser.is_open:
            ts = self.get_timestamp(); cmd = "$H S" if simulated else "$H"; self.is_homed = False
            self.last_sent_cs = self.calculate_checksum(cmd); self.last_sent_package = cmd
            self.log_display.append(f"<font color='#ffffff'>TX {ts} [HOME] {cmd}</font>")
            self.ser.write((cmd + "\n").encode('ascii')); self.waiting_for_ack = True; self.timeout_timer.start(30000) 
        else: self.log_error("串口未连接")
        
    def stop_motion(self):
        if hasattr(self, "_stop_binary_stream"):
            self._stop_binary_stream()
        if hasattr(self, "_finish_binary_motion"):
            self._finish_binary_motion("Stopped")
        else:
            self.binary_motion_active = False
        self.point_queue = []
        if hasattr(self, "_clear_text_sender_state"):
            self._clear_text_sender_state()
        else:
            self.waiting_for_ack = False
        self.stream_waiting_buffer = False
        self.last_sent_motion = None
        self.emergency_resume_path = []
        self.active_binary_send_path = []
        self.active_preview_path = []
        self.emergency_paused = False
        if hasattr(self, "_set_emergency_button_paused"):
            self._set_emergency_button_paused(False)
        self.motion_preamble_needed = True
        self.timeout_timer.stop()
        ts = self.get_timestamp()
        self.log_display.append(
            f"<font color='#f39c12'>TX {ts} STOP (\u505c\u6b62\uff1a\u6e05\u9664\u4e0a\u4f4d\u673a\u961f\u5217)</font>"
        )
        if self.ser and self.ser.is_open:
            self.ser.write(b"STOP\n")

    def emergency_stop(self):
        if getattr(self, "emergency_paused", False):
            self._resume_emergency_motion()
            return

        if self.last_sent_motion is not None:
            self.point_queue.insert(0, self.last_sent_motion)
            self.last_sent_motion = None
            self.sent_point_id = max(0, self.sent_point_id - 1)
        self.emergency_resume_path = self._capture_emergency_resume_path()
        if hasattr(self, "_stop_binary_stream"):
            self._stop_binary_stream()
        self.binary_motion_active = False
        self.waiting_for_ack = False
        self.stream_waiting_buffer = False
        self.motion_preamble_needed = True
        self.timeout_timer.stop()
        self.emergency_paused = True
        if hasattr(self, "_set_emergency_button_paused"):
            self._set_emergency_button_paused(True)
        ts = self.get_timestamp()
        self.log_display.append(
            f"<font color='#e74c3c'>TX {ts} ESTOP (pause and keep remaining queue)</font>"
        )
        if self.ser and self.ser.is_open:
            self.ser.write(b"M112\n")

    def _capture_emergency_resume_path(self):
        if getattr(self, "point_queue", None):
            return list(self.point_queue)
        send_path = list(getattr(self, "active_binary_send_path", []) or [])
        if not send_path:
            return []
        cx, cy = float(self.cur_x), float(self.cur_y)
        best_index = 0
        best_dist = float("inf")
        for index, point in enumerate(send_path):
            dist = math.hypot(float(point[0]) - cx, float(point[1]) - cy)
            if dist < best_dist:
                best_dist = dist
                best_index = index
        remaining = send_path[min(best_index + 1, len(send_path)) :]
        return list(remaining)

    def _resume_emergency_motion(self):
        remaining = list(getattr(self, "emergency_resume_path", []) or [])
        self.emergency_paused = False
        if hasattr(self, "_set_emergency_button_paused"):
            self._set_emergency_button_paused(False)
        self.motion_preamble_needed = True
        if self.ser and self.ser.is_open:
            self.ser.write(b"$X\n")
            self.ser.write(b"M17\n")
        if not remaining:
            self.log_display.append("<font color='orange'>RESUME: no remaining motion queue.</font>")
            return
        self.log_display.append(
            f"<font color='#2ecc71'>RESUME: reloading {len(remaining)} remaining keypoints.</font>"
        )
        self.emergency_resume_path = []
        self.load_motion_queue(remaining, send_path=remaining)

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

    def _binary_host_segment_class(self):
        return HostTimedSegment

    def _joint_deg_to_pulse(self, q1, q2):
        p1, p2 = self._joint_deg_to_pulse_float(q1, q2)
        return int(round(p1)), int(round(p2))

    def _joint_deg_to_pulse_float(self, q1, q2):
        ppr = float(int(getattr(self, "current_ppr", 3200) or 3200))
        theta1_mrad = math.radians(float(q1)) * 1000.0
        theta2_mrad = math.radians(float(q2)) * 1000.0
        p1 = ((theta1_mrad - float(self.BINARY_ZERO_MRAD[0])) * ppr) / float(self.BINARY_MRAD_PER_REV)
        p2 = ((theta2_mrad - float(self.BINARY_ZERO_MRAD[1])) * ppr) / float(self.BINARY_MRAD_PER_REV)
        return p1, p2

    def _current_feedback_pulses(self):
        p1 = getattr(self, "feedback_p1", None)
        p2 = getattr(self, "feedback_p2", None)
        if p1 is not None and p2 is not None:
            return int(p1), int(p2)
        q1, q2 = self.inverse_kinematics(float(self.cur_x), float(self.cur_y))
        if q1 is None or q2 is None:
            raise ValueError("Current pose has no valid IK; cannot plan host segments.")
        return self._joint_deg_to_pulse(q1, q2)

    def _trapezoid_distance_at(self, distance, vmax, accel, t):
        distance = max(0.0, float(distance))
        vmax = max(1e-6, float(vmax))
        accel = max(1e-6, float(accel))
        t_accel = vmax / accel
        d_accel = 0.5 * accel * t_accel * t_accel
        if 2.0 * d_accel >= distance:
            t_accel = math.sqrt(distance / accel)
            total_time = 2.0 * t_accel
            if t <= t_accel:
                return 0.5 * accel * t * t, total_time
            td = max(0.0, t - t_accel)
            peak = accel * t_accel
            return min(distance, 0.5 * distance + peak * td - 0.5 * accel * td * td), total_time
        d_flat = distance - 2.0 * d_accel
        t_flat = d_flat / vmax
        total_time = 2.0 * t_accel + t_flat
        if t <= t_accel:
            return 0.5 * accel * t * t, total_time
        if t <= t_accel + t_flat:
            return d_accel + vmax * (t - t_accel), total_time
        td = max(0.0, t - t_accel - t_flat)
        return min(distance, d_accel + d_flat + vmax * td - 0.5 * accel * td * td), total_time

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

    def _trapezoid_time_at_distance(self, scalar, profile):
        distance = profile["distance"]
        if scalar <= 0.0:
            return 0.0
        if scalar >= distance:
            return profile["total_time"]
        accel = profile["accel"]
        t_accel = profile["t_accel"]
        d_accel = profile["d_accel"]
        t_flat = profile["t_flat"]
        vmax = profile["peak"] if t_flat <= 0.0 else profile["vmax"]
        if scalar <= d_accel:
            return math.sqrt((2.0 * scalar) / accel)
        flat_end = distance - d_accel
        if t_flat > 0.0 and scalar <= flat_end:
            return t_accel + (scalar - d_accel) / vmax
        remain = max(0.0, distance - scalar)
        return profile["total_time"] - math.sqrt((2.0 * remain) / accel)

    def _set_host_motion_status(self, stats):
        self._last_host_plan_stats = dict(stats)
        label = getattr(self, "jog_pps_label", None)
        if label is None:
            return
        peak = float(stats.get("peak_pps", 0.0))
        target = float(stats.get("target_pps", 0.0))
        period = 1000.0 / peak if peak > 1e-9 else 0.0
        limited = bool(stats.get("limited", False))
        low = peak < self.HOST_MIN_SMOOTH_PPS
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
        ppr = int(getattr(self, "current_ppr", 3200) or 3200)
        return float(speed_mm_s) * ppr / self._axis_circumference_mm(motor_id)

    def _accel_mm_s2_to_pps2(self, accel_mm_s2, motor_id=1):
        ppr = int(getattr(self, "current_ppr", 3200) or 3200)
        return float(accel_mm_s2) * ppr / self._axis_circumference_mm(motor_id)

    def update_jog_pps_preview(self):
        try:
            speed = self._read_run_speed_mm_s()
            accel = self._read_run_accel_mm_s2()
            step = self._read_jog_step_mm()
            target_pps = self._speed_mm_s_to_pps(speed)
            accel_pps2 = self._accel_mm_s2_to_pps2(accel)
            ppr = int(getattr(self, "current_ppr", 3200) or 3200)
            pulses = max(1.0, abs(step) * ppr / self._axis_circumference_mm())
            profile = self._trapezoid_profile(pulses, target_pps, accel_pps2)
            self._set_host_motion_status(
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

    def _build_joint_host_segments(self, start_pulses, end_pulses, vmax_pps, accel_pps2, sample_ticks=None):
        BinaryHostSegment = self._binary_host_segment_class()
        start1, start2 = int(start_pulses[0]), int(start_pulses[1])
        end1, end2 = int(end_pulses[0]), int(end_pulses[1])
        d1 = end1 - start1
        d2 = end2 - start2
        dom = max(abs(d1), abs(d2))
        if dom == 0:
            return []
        vmax_pps = float(vmax_pps)
        accel_pps2 = float(accel_pps2)
        if vmax_pps > self.HOST_DDA_MAX_PPS:
            raise ValueError(
                f"目标 {vmax_pps:.1f}pps 超过当前 10kHz DDA 稳定上限 {self.HOST_DDA_MAX_PPS}pps；请降低速度或改用硬件定时器脉冲。"
            )
        profile = self._trapezoid_profile(dom, vmax_pps, accel_pps2)
        if profile["peak"] > self.HOST_DDA_MAX_PPS:
            raise ValueError(
                f"峰值 {profile['peak']:.1f}pps 超过当前 10kHz DDA 稳定上限 {self.HOST_DDA_MAX_PPS}pps。"
            )
        self._set_host_motion_status(
            {
                "target_pps": vmax_pps,
                "peak_pps": profile["peak"],
                "limited": profile["limited"],
            }
        )

        segments = []
        last_tick = 0
        last1, last2 = start1, start2

        for event in range(1, dom + 1):
            t = self._trapezoid_time_at_distance(event, profile)
            tick = int(round(t * self.HOST_DDA_HZ))
            if tick <= last_tick:
                raise ValueError(
                    f"规划段要求一个 100us tick 内输出多个脉冲: event={event}, tick={tick}, last={last_tick}"
                )
            duration = tick - last_tick
            if duration > 65535:
                raise ValueError(
                    f"相邻脉冲间隔 {duration} ticks 超过协议上限；请提高速度或加速度。"
                )
            if event == dom:
                target1, target2 = end1, end2
            else:
                target1 = start1 + int(round(d1 * event / dom))
                target2 = start2 + int(round(d2 * event / dom))
            if target1 == last1 and target2 == last2:
                last_tick = tick
                continue
            segments.append(BinaryHostSegment(target1, target2, duration))
            last1, last2 = target1, target2
            last_tick = tick
        return segments

    def _build_cartesian_host_segments(self, start_xy, end_xy, speed_mm_s, accel_mm_s2=None, sample_ticks=None):
        sx, sy = float(start_xy[0]), float(start_xy[1])
        ex, ey = float(end_xy[0]), float(end_xy[1])
        dx, dy = ex - sx, ey - sy
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return []

        speed_mm_s = float(speed_mm_s)
        accel_mm_s2 = float(accel_mm_s2 or self.HOST_DEFAULT_ACCEL_MM_S2)
        if speed_mm_s <= 0.0 or accel_mm_s2 <= 0.0:
            raise ValueError("点动速度和加速度必须大于 0")
        start_pulses = self._current_feedback_pulses()
        q1, q2 = self.inverse_kinematics(ex, ey)
        if q1 is None or q2 is None:
            raise ValueError(f"Host segment IK failed at X={ex:.3f}, Y={ey:.3f}")
        end1, end2 = self._joint_deg_to_pulse(q1, q2)
        dom = max(abs(end1 - start_pulses[0]), abs(end2 - start_pulses[1]))
        if dom == 0:
            return []
        pulses_per_mm = dom / distance
        vmax_pps = speed_mm_s * pulses_per_mm
        accel_pps2 = accel_mm_s2 * pulses_per_mm
        return self._build_joint_host_segments(start_pulses, (end1, end2), vmax_pps, accel_pps2)

    def build_host_segments_from_path(self, path, start_xy=None, label="host trajectory"):
        """Convert a planned UI XY path into smoothed timed joint DDA segments."""
        BinaryHostSegment = self._binary_host_segment_class()
        if not path:
            return []

        start_xy = (float(start_xy[0]), float(start_xy[1])) if start_xy is not None else (float(self.cur_x), float(self.cur_y))
        q1_start, q2_start = self.inverse_kinematics(start_xy[0], start_xy[1])
        if q1_start is None or q2_start is None:
            start_i = self._current_feedback_pulses()
            start_f = (float(start_i[0]), float(start_i[1]))
        else:
            start_f = self._joint_deg_to_pulse_float(q1_start, q2_start)
            start_i = (int(round(start_f[0])), int(round(start_f[1])))

        samples = [{"u": 0.0, "t": 0.0, "p1": start_f[0], "p2": start_f[1], "flags": 0}]
        last_xy = start_xy
        last_pulse_f = start_f
        last_speed = 0.0

        def point_feed_mm_s(point):
            if len(point) > 2:
                return max(0.0, float(point[2]) / 60.0)
            return max(0.0, self._read_run_speed_mm_s())

        def point_flags(point):
            return int(point[4]) if len(point) > 4 else 0

        for index, point in enumerate(path):
            x = float(point[0])
            y = float(point[1])
            speed = point_feed_mm_s(point)
            ds = math.hypot(x - last_xy[0], y - last_xy[1])
            if ds <= 1e-6:
                last_speed = speed
                last_xy = (x, y)
                continue
            q1, q2 = self.inverse_kinematics(x, y)
            if q1 is None or q2 is None:
                raise ValueError(f"{label}: IK failed at index={index} X={x:.3f}, Y={y:.3f}")
            pulse_f = self._joint_deg_to_pulse_float(q1, q2)
            avg_speed = 0.5 * (max(0.0, last_speed) + max(0.0, speed))
            if avg_speed <= 1e-6:
                avg_speed = max(0.1, speed, last_speed)
            dt = ds / avg_speed
            du = max(abs(pulse_f[0] - last_pulse_f[0]), abs(pulse_f[1] - last_pulse_f[1]))
            samples.append(
                {
                    "u": samples[-1]["u"] + max(0.0, du),
                    "t": samples[-1]["t"] + max(0.0, dt),
                    "p1": float(pulse_f[0]),
                    "p2": float(pulse_f[1]),
                    "flags": point_flags(point),
                }
            )
            last_xy = (x, y)
            last_pulse_f = pulse_f
            last_speed = speed

        if len(samples) <= 1 or samples[-1]["u"] <= 1e-9:
            return []

        segments = []
        sample_index = 1
        last_tick = 0
        last_pulse = (int(start_i[0]), int(start_i[1]))
        peak_pps = 0.0
        target_pps = 0.0

        def interpolate_at_time(target_time):
            nonlocal sample_index
            target_time = max(0.0, min(float(target_time), samples[-1]["t"]))
            while sample_index < len(samples) - 1 and samples[sample_index]["t"] < target_time:
                sample_index += 1
            prev = samples[sample_index - 1]
            cur = samples[sample_index]
            span = cur["t"] - prev["t"]
            ratio = 1.0 if span <= 1e-9 else max(0.0, min(1.0, (target_time - prev["t"]) / span))
            return {
                "p1": prev["p1"] + (cur["p1"] - prev["p1"]) * ratio,
                "p2": prev["p2"] + (cur["p2"] - prev["p2"]) * ratio,
                "flags": cur["flags"],
            }

        def append_one(target, duration, flags=0):
            nonlocal last_pulse, peak_pps, target_pps
            target = (int(target[0]), int(target[1]))
            d1 = target[0] - last_pulse[0]
            d2 = target[1] - last_pulse[1]
            dom = max(abs(d1), abs(d2))
            if dom == 0:
                return
            duration = int(duration)
            if duration > 65535:
                raise ValueError(f"{label}: segment interval {duration} ticks exceeds protocol limit; increase speed/accel or PPR.")
            if dom > duration:
                requested_pps = dom * self.HOST_DDA_HZ / max(1, duration)
                raise ValueError(f"{label}: requested {requested_pps:.1f}pps exceeds 10kHz DDA execution; reduce speed or use hardware timer.")
            segment_flags = self.BINARY_FLAG_HOST_SEGMENT | (int(flags) & self.BINARY_FLAG_EXACT_STOP)
            segments.append(BinaryHostSegment(target[0], target[1], duration, flags=segment_flags))
            pps = dom * self.HOST_DDA_HZ / max(1, duration)
            peak_pps = max(peak_pps, pps)
            target_pps = max(target_pps, pps)
            last_pulse = target

        total_tick = max(1, int(round(samples[-1]["t"] * self.HOST_DDA_HZ)))
        final_target = (int(round(samples[-1]["p1"])), int(round(samples[-1]["p2"])))
        slice_ticks = max(10, int(self.HOST_TRAJECTORY_SLICE_TICKS))
        tick = slice_ticks
        while tick <= total_tick:
            interp = interpolate_at_time(tick / self.HOST_DDA_HZ)
            is_final = tick >= total_tick
            target = final_target if is_final else (int(round(interp["p1"])), int(round(interp["p2"])))
            flags = (interp["flags"] | self.BINARY_FLAG_EXACT_STOP) if is_final else interp["flags"]
            before_count = len(segments)
            append_one(target, tick - last_tick, flags)
            if len(segments) != before_count:
                last_tick = tick
            tick += slice_ticks

        if last_tick < total_tick:
            interp = interpolate_at_time(samples[-1]["t"])
            before_count = len(segments)
            append_one(final_target, total_tick - last_tick, interp["flags"] | self.BINARY_FLAG_EXACT_STOP)
            if len(segments) != before_count:
                last_tick = total_tick

        if segments and (segments[-1].p1_abs, segments[-1].p2_abs) != final_target:
            append_one(final_target, max(1, total_tick - last_tick), self.BINARY_FLAG_EXACT_STOP)
        if not segments:
            return []
        if (segments[-1].flags & self.BINARY_FLAG_EXACT_STOP) == 0:
            segments[-1].flags |= self.BINARY_FLAG_EXACT_STOP
        self._set_host_motion_status(
            {
                "target_pps": target_pps,
                "peak_pps": peak_pps,
                "limited": peak_pps < self.HOST_MIN_SMOOTH_PPS,
            }
        )
        return segments

    def binary_points_from_host_segments(self, segments):
        return [
            HostTimedBinaryPoint(segment, self.BINARY_FLAG_HOST_SEGMENT, self.BINARY_FLAG_EXACT_STOP)
            for segment in segments
        ]

    def _upload_host_segments(self, segments, label):
        if not segments:
            self.log_error("Host segment planner produced no motion.")
            return False
        if not (self.ser and self.ser.is_open):
            self.log_error("Serial is not connected; host segments require the controller.")
            return False
        if self.waiting_for_ack or self.point_queue or getattr(self, "binary_motion_active", False):
            self.log_error("Motion is busy; wait or stop before host-planned jog.")
            return False
        if bool(getattr(self, "host_timed_segment_mode", False)) and hasattr(self, "_queue_gcode_segments"):
            return self._queue_gcode_segments(segments, label=label)
        if hasattr(self, "_upload_binary_points"):
            try:
                points = self.binary_points_from_host_segments(segments)
            except Exception as exc:
                self.log_error(f"Timed binary segment conversion failed: {exc}")
                return False
            self._publish_commanded_segments(segments)
            return self._upload_binary_points(points, label=f"{label} timed binary")
        self.log_error("Buffered timed segment uploader is unavailable.")
        return False

    def _publish_commanded_segments(self, segments):
        monitor = getattr(self, "velocity_monitor", None)
        if monitor is None or not hasattr(monitor, "process_commanded_pps"):
            return
        try:
            last1, last2 = self._current_feedback_pulses()
            ppr = int(getattr(self, "current_ppr", 3200) or 3200)
            for segment in segments:
                ticks = max(1, int(segment.duration_ticks))
                pps1 = (int(segment.p1_abs) - last1) * 10000.0 / ticks
                pps2 = (int(segment.p2_abs) - last2) * 10000.0 / ticks
                monitor.process_commanded_pps(pps1, pps2, ticks, ppr)
                last1, last2 = int(segment.p1_abs), int(segment.p2_abs)
        except Exception:
            return

    def motor_jog_direct(self, motor_id, direction):
        """Direct single-axis pulse jog for motor bring-up."""
        try:
            if motor_id not in (1, 2):
                self.log_error(f"Unknown motor id: {motor_id}")
                return
            start = self._current_feedback_pulses()
            ppr = int(getattr(self, "current_ppr", 3200) or 3200)
            step_mm = self._read_jog_step_mm()
            v_mm_s = self._read_run_speed_mm_s()
            accel_mm_s2 = self._read_run_accel_mm_s2()
            delta = int(round(direction * step_mm * ppr / self._axis_circumference_mm(motor_id)))
            if delta == 0:
                self.log_error("点动步长换算后小于 1 脉冲，请增大步长或拨码 PPR。")
                return
            end = [int(start[0]), int(start[1])]
            end[motor_id - 1] += delta
            v_pps = self._speed_mm_s_to_pps(v_mm_s, motor_id)
            accel_pps2 = self._accel_mm_s2_to_pps2(accel_mm_s2, motor_id)
            segments = self._build_joint_host_segments(start, tuple(end), v_pps, accel_pps2)
            if self._upload_host_segments(segments, f"host axis jog M{motor_id}"):
                stats = getattr(self, "_last_host_plan_stats", {})
                self.log_display.append(
                    f"<font color='#ffffff'>HOST_JOG M{motor_id} step={step_mm:g}mm delta={delta} segments={len(segments)} target={v_pps:.1f}pps peak={float(stats.get('peak_pps', 0.0)):.1f}pps</font>"
                )
        except Exception as e:
            self.log_error(f"Single-axis jog error: {e}")

    def add_jog_step(self, ux, uy):
        try:
            step = self._read_jog_step_mm()
            self.add_jog(float(ux) * step, float(uy) * step)
        except Exception as e:
            self.log_error(f"点动步长错误: {e}")

    def add_jog(self, dx, dy):
        """方向点动：空闲时走二进制关节插补，运动队列中才退回 ASCII 追加。

        调节入口：
        - 点动距离来自 UI 按钮绑定的 dx/dy。
        - 点动速度来自 hw_speed_input，单位 mm/s。
        - 直线平滑度由 BINARY_LINE_TOLERANCE_MM 和当前 PPR 决定。
        """
        try:
            v_max = self._read_run_speed_mm_s()
            if getattr(self, "binary_stream_active", False):
                self.log_error("二进制轨迹续传中，暂不追加点动；请等待当前轨迹完成或先停止")
                return
            sx, sy = (self.point_queue[-1][0], self.point_queue[-1][1]) if self.point_queue else (self.cur_x, self.cur_y)
            if not self.check_workspace_safety(sx, sy):
                sx, sy = self.kinematics.find_safe_home((self.HOME_X, self.HOME_Y))
                self.cur_x, self.cur_y = sx, sy
                self.history_x, self.history_y = [sx], [sy]
                self.log_display.append(
                    f"<font color='yellow'>当前位置不可达，已校正到安全点 X={sx:.1f}, Y={sy:.1f}</font>"
                )
                self.update_plot()
            tx, ty = sx + dx, sy + dy
            if self.check_workspace_safety(tx, ty):
                moving = bool(self.waiting_for_ack or self.point_queue)
                if not moving:
                    path = self.generate_linear_path(sx, sy, tx, ty, v_max)
                    if not path:
                        self.log_error("点动距离过短，未生成轨迹")
                        return
                    self.preview_planned_path(path, "点动")
                    accel = self._read_run_accel_mm_s2()
                    segments = self._build_cartesian_host_segments((sx, sy), (tx, ty), v_max, accel)
                    if self._upload_host_segments(segments, "host cartesian jog"):
                        stats = getattr(self, "_last_host_plan_stats", {})
                        self.log_display.append(
                            f"<font color='#ffffff'>HOST_JOG XY dx={dx:g} dy={dy:g} segments={len(segments)} peak={float(stats.get('peak_pps', 0.0)):.1f}pps</font>"
                        )
                    return
                blend_speed = v_max * 0.35
                path = self.generate_linear_path(sx, sy, tx, ty, v_max, v_start=blend_speed, v_end=blend_speed)
                if not path:
                    self.log_error("点动距离过短，未生成轨迹")
                    return
                self.preview_planned_path(path, "点动")
                send_path = None
                self.load_motion_queue(path, append=moving, send_path=send_path)
            else: self.log_error(f"不可达区域: X={tx:.1f}, Y={ty:.1f}")
        except Exception as e: self.log_error(f"点动错误: {e}")
    
    def motor_jog(self, motor_id, direction):
        """单电机点动。

        空闲时：把单关节转动采样成若干末端点，再以二进制关节目标上传，由下位机插补。
        队列中：保留旧的 ASCII 追加行为，避免中途打断当前运动。

        参数：
        - motor_id：1 或 2，表示点动哪个主动电机。
        - direction：1 为正向，-1 为反向。
        - half_turn：单次点动角度，单位 deg；需要更小步距可调低此值。
        - num_steps：关节点采样数；调大更贴近单关节曲线，但上传点更多。
        """
        try:
            if getattr(self, "binary_stream_active", False):
                self.log_error("二进制轨迹续传中，暂不追加电机点动；请等待当前轨迹完成或先停止")
                return
            # 若 ASCII 队列正在运行，则以队列尾点作为下一段起点；否则使用当前反馈/仿真坐标。
            if self.waiting_for_ack or self.point_queue:
                sx, sy = self.point_queue[-1][0], self.point_queue[-1][1]
            else:
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

            # send_path 直接使用这组采样点；下位机会在相邻关节目标之间做 10kHz 插补。
            path = []
            for i in range(1, len(joint_points)):
                x0, y0 = joint_points[i - 1]
                x1, y1 = joint_points[i]
                path.append((x1, y1, v_max, False))

            if not path:
                self.log_error("电机点动未生成有效轨迹")
                return

            self.preview_planned_path(path, f"电机{motor_id}点动")
            moving = bool(self.waiting_for_ack or self.point_queue)
            self.load_motion_queue(path, append=moving, send_path=None if moving else path)
            self.log_display.append(
                f"<font color='cyan'>电机{motor_id} {'正向' if direction > 0 else '反向'}点动: ({sx:.1f},{sy:.1f}) -> ({new_x:.1f},{new_y:.1f})</font>"
            )

        except Exception as e:
            self.log_error(f"电机点动错误: {e}")
