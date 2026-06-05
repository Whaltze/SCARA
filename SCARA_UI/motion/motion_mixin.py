import math
import numpy as np


class ScaraMotionMixin:
    JOINT_LIMITS_DEG = ((-180.0, 180.0), (0.0, 360.0))
    ARC_SEGMENT_MM = 2.0
    DEFAULT_CORNER_RADIUS_MM = 2.0
    CAR_CORNER_RADIUS_MM = 1.0
    PATH_SIMPLIFY_TOLERANCE_MM = 0.18
    PATH_MIN_POINT_SPACING_MM = 0.20

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
        segments = max(8, int(math.ceil(arc_len / self.ARC_SEGMENT_MM)))
        segments = min(segments, 5000)
        points = []
        for i in range(segments + 1):
            t = i / segments
            a = a0 + delta * t
            points.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
        points[0] = start
        points[-1] = end
        return points

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
            self.load_motion_queue(path)

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
                label = "G1 直线"
            elif mode.startswith("G2") or mode.startswith("G3"):
                radius = self._read_float(self.radius_r, "圆弧半径", positive=True)
                path = self.generate_arc_path(
                    start[0],
                    start[1],
                    end[0],
                    end[1],
                    radius,
                    clockwise=mode.startswith("G2"),
                    speed_max=v,
                    silent=silent,
                )
                label = "G2 顺圆" if mode.startswith("G2") else "G3 逆圆"
            else:
                raise ValueError(f"不支持的轨迹模式: {mode}")

            if not path:
                self.log_error(f"{label}未生成可发送轨迹")
                return
            self.preview_planned_path(path, label)
            self.load_motion_queue(path)
            self.log_display.append(
                f"<font color='cyan'>{label}规划完成: {len(path)} 点，目标 X={tx:.1f}, Y={ty:.1f}</font>"
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
                keep_corners={4, 5},
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
                keep_corners={3, 4, 5},
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
            path = self.generate_geometry_path(self.build_car1_segments(x0, y0), v, silent_first=True, label="小车轨迹1")
            self.preview_planned_path(path, "小车轨迹1")
            self.load_motion_queue(path)
        except ValueError as e:
            self.log_error(f"小车轨迹1参数错误: {e}")
        except Exception as e:
            self.log_error(f"小车轨迹1规划错误: {e}")

    def plan_car2_path(self):
        try:
            x0 = self._read_float(self.car_start_x, "小车起始X")
            y0 = self._read_float(self.car_start_y, "小车起始Y")
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            path = self.generate_geometry_path(self.build_car2_segments(x0, y0), v, silent_first=True, label="小车轨迹2")
            self.preview_planned_path(path, "小车轨迹2")
            self.load_motion_queue(path)
        except ValueError as e:
            self.log_error(f"小车轨迹2参数错误: {e}")
        except Exception as e:
            self.log_error(f"小车轨迹2规划错误: {e}")

    def generate_stroke_path(self, strokes, speed_max, label="写字轨迹"):
        path = []
        current = (self.cur_x, self.cur_y)
        valid_strokes = []
        for stroke in strokes:
            clean = self.preprocess_control_points(
                stroke,
                simplify_tolerance=self.PATH_SIMPLIFY_TOLERANCE_MM,
                min_spacing=self.PATH_MIN_POINT_SPACING_MM,
            )
            if len(clean) >= 2:
                valid_strokes.append(clean)

        if not valid_strokes:
            self.log_error(f"{label}没有足够的有效笔画")
            return []

        for stroke in valid_strokes:
            if math.hypot(stroke[-1][0] - current[0], stroke[-1][1] - current[1]) < math.hypot(
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
                corner_radius_mm=self.DEFAULT_CORNER_RADIUS_MM,
                silent_first=False,
            )
            stroke_path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
            if stroke_path and not self.validate_trajectory_points(stroke_path, f"{label}采样点"):
                return []
            path.extend(stroke_path)
            current = stroke[-1]

        return path

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
        left = self.HOME_X - width_mm * 0.5
        bottom = self.HOME_Y - height_mm * 0.5
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
                self.load_motion_queue(path)
                self.log_display.append(f"<font color='cyan'>手写轨迹已装载: {len(path)} 点</font>")
        except Exception as e:
            self.log_error(f"手写轨迹规划错误: {e}")

    def clear_handwriting(self):
        if hasattr(self, "handwriting_pad"):
            self.handwriting_pad.clear()
        self.preview_x, self.preview_y = [], []
        self.preview_label = ""
        self.update_plot()

    def build_text_outline_strokes(self, text, max_width_mm=135.0, max_height_mm=85.0):
        from PySide6.QtGui import QFont, QFontDatabase, QPainterPath, QTransform
        from PySide6.QtWidgets import QApplication

        if QApplication.instance() is None:
            self._text_outline_qt_app = QApplication([])

        try:
            families = set(QFontDatabase.families())
        except TypeError:
            families = set(QFontDatabase().families())
        family = "Microsoft YaHei" if "Microsoft YaHei" in families else QFont().family()
        font = QFont(family, 100)
        painter_path = QPainterPath()
        painter_path.addText(0.0, 0.0, font, text)
        polygons = painter_path.toSubpathPolygons(QTransform())

        raw_strokes = []
        all_points = []
        for polygon in polygons:
            points = [(float(point.x()), float(point.y())) for point in polygon]
            if len(points) < 3:
                continue
            if math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1]) > 0.01:
                points.append(points[0])
            raw_strokes.append(points)
            all_points.extend(points)

        if not all_points:
            return []

        min_x = min(p[0] for p in all_points)
        max_x = max(p[0] for p in all_points)
        min_y = min(p[1] for p in all_points)
        max_y = max(p[1] for p in all_points)
        text_w = max(1.0, max_x - min_x)
        text_h = max(1.0, max_y - min_y)
        scale = min(max_width_mm / text_w, max_height_mm / text_h)
        out_w = text_w * scale
        out_h = text_h * scale
        left = self.HOME_X - out_w * 0.5
        bottom = self.HOME_Y - out_h * 0.5

        strokes = []
        for raw in raw_strokes:
            converted = []
            for x, y in raw:
                rx = left + (x - min_x) * scale
                ry = bottom + (max_y - y) * scale
                converted.append((rx, ry))
            clean = self.preprocess_control_points(converted, simplify_tolerance=0.18, min_spacing=0.35)
            if len(clean) >= 3:
                if math.hypot(clean[0][0] - clean[-1][0], clean[0][1] - clean[-1][1]) > 0.35:
                    clean.append(clean[0])
                strokes.append(clean)
        return self._order_text_strokes(strokes)

    def plan_fixed_text_path(self, text):
        try:
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            strokes = self.build_text_outline_strokes(text)
            path = self.generate_stroke_path(strokes, v, label=f"空心字{text}")
            if path:
                self.preview_planned_path(path, f"空心字{text}")
                self.load_motion_queue(path)
                self.log_display.append(f"<font color='cyan'>空心字{text}已装载: {len(path)} 点</font>")
        except Exception as e:
            self.log_error(f"空心字{text}规划错误: {e}")

    def system_reset(self): 
        if self.board_only_debug:
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
            self.send_ascii_line("ZERO", "SOFT_ZERO")
            return
        if self.ser and self.ser.is_open:
            ts = self.get_timestamp(); cmd = "$H"; self.is_homed = False
            self.last_sent_cs = self.calculate_checksum(cmd); self.last_sent_package = cmd
            self.log_display.append(f"<font color='#ffffff'>TX {ts} [HOME] $H</font>")
            self.ser.write((cmd + "\n").encode('ascii')); self.waiting_for_ack = True; self.timeout_timer.start(10000) 
        else: self.log_error("串口未连接")
        
    def stop_motion(self):
        self.point_queue = []
        self.waiting_for_ack = False
        self.stream_waiting_buffer = False
        self.last_sent_motion = None
        self.motion_preamble_needed = True
        self.timeout_timer.stop()
        ts = self.get_timestamp()
        self.log_display.append(
            f"<font color='#f39c12'>TX {ts} STOP (\u505c\u6b62\uff1a\u6e05\u9664\u4e0a\u4f4d\u673a\u961f\u5217)</font>"
        )
        if self.ser and self.ser.is_open:
            self.ser.write(b"STOP\n")

    def emergency_stop(self):
        if self.last_sent_motion is not None:
            self.point_queue.insert(0, self.last_sent_motion)
            self.last_sent_motion = None
            self.sent_point_id = max(0, self.sent_point_id - 1)
        self.waiting_for_ack = False
        self.stream_waiting_buffer = False
        self.motion_preamble_needed = True
        self.timeout_timer.stop()
        ts = self.get_timestamp()
        self.log_display.append(
            f"<font color='#e74c3c'>TX {ts} ESTOP (\u6025\u505c\uff1a\u4fdd\u7559\u4e0a\u4f4d\u673a\u961f\u5217)</font>"
        )
        if self.ser and self.ser.is_open:
            self.ser.write(b"ESTOP\n")

    def add_jog(self, dx, dy):
        try:
            v_max = float(self.jog_speed_input.text())
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
                blend_speed = min(v_max * 0.35, 8.0) if (self.waiting_for_ack or self.point_queue) else 0.0
                path = self.generate_linear_path(sx, sy, tx, ty, v_max, v_start=blend_speed, v_end=blend_speed)
                if not path:
                    self.log_error("点动距离过短，未生成轨迹")
                    return
                self.preview_planned_path(path, "点动")
                self.load_motion_queue(path, append=True)
            else: self.log_error(f"不可达区域: X={tx:.1f}, Y={ty:.1f}")
        except Exception as e: self.log_error(f"点动错误: {e}")
    
    def motor_jog(self, motor_id, direction):
        """
        ???????????/????????????
        :param motor_id: ???? (1 ? 2)
        :param direction: ?? (1: ??, -1: ??)
        """
        try:
            current_angles = self.inverse_kinematics(self.cur_x, self.cur_y)
            if current_angles[0] is None or current_angles[1] is None:
                self.log_error("??????????")
                return
            
            q1, q2 = current_angles
            
            half_turn = 3.0
            new_q1, new_q2 = q1, q2
            
            if motor_id == 1:
                new_q1 = q1 + direction * half_turn
            elif motor_id == 2:
                new_q2 = q2 + direction * half_turn
            else:
                self.log_error(f"??????: {motor_id}")
                return
            
            new_x, new_y = self.kinematics.forward(new_q1, new_q2)
            if new_x is None or new_y is None:
                self.log_error(f"??{motor_id}????????")
                return

            if not self.check_workspace_safety(new_x, new_y):
                self.log_error(f"??{motor_id}?????????: X={new_x:.1f}, Y={new_y:.1f}")
                return

            v_max = float(self.jog_speed_input.text())
            num_steps = 20  # ??????????

            # ---- ???????????????????? ----
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
                    self.log_error(f"??{motor_id}???{i}???")
                    return
                joint_points.append((px, py))

            # ---- ?????TCP ????? ----
            path = []
            for i in range(1, len(joint_points)):
                x0, y0 = joint_points[i - 1]
                x1, y1 = joint_points[i]
                path.append((x1, y1, v_max, False))

            if not path:
                self.log_error("??????????????")
                return

            self.preview_planned_path(path, f"??{motor_id}??")
            self.load_motion_queue(path, append=True)
            self.log_display.append(
                f"<font color='cyan'>??{motor_id} {'??' if direction > 0 else '??'}????????: ({self.cur_x:.1f},{self.cur_y:.1f}) -> ({new_x:.1f},{new_y:.1f})</font>"
            )

        except Exception as e:
            self.log_error(f"??????: {e}")
