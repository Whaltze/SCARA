import re
import time

import serial
import serial.tools.list_ports
from PySide6.QtCore import QTimer
from .motion_senders import GRBL_GCODE_SENDER
from .serial_protocol import build_g1_line, build_ppr_line, parse_ok_ack
from .serial_worker import SerialThreadTransport


class ScaraSerialMixin:
    ACK_TIMEOUT_MS = 1000
    ACK_RECHECK_MS = 1500
    HOME_STATUS_POLL_MS = 2000
    CONTROLLER_SILENCE_LIMIT_S = 6.0
    IDLE_ACK_STALL_LIMIT_S = 3.0
    IDLE_ACK_STALL_POLLS = 2
    STREAM_LOG_FIRST = 20
    STREAM_LOG_EVERY = 100

    def _sender_now(self):
        return time.time()

    def _apply_home_state(self, state):
        state = (state or "").strip().lower()
        if state == "done":
            self.is_homed = True
            self.home_sensor_triggered = False
            self.home_feedback_active = False
            self.motion_preamble_needed = True
            if hasattr(self, "_reset_jog_anchor"):
                self._reset_jog_anchor()
        elif state == "error":
            self.is_homed = False

    def _laser_requested(self):
        toggle = getattr(self, "laser_enable_toggle", None)
        return bool(toggle is not None and toggle.isChecked())

    def _laser_power_from_ui(self):
        widget = getattr(self, "laser_power_input", None)
        percent = float(widget.value()) if widget is not None else 1.0
        return max(1, min(50, int(round(percent * 10.0))))

    def _laser_s_word(self):
        return max(0, min(1000, int(round(float(getattr(self, "laser_power_permille", 10)) * 20.0))))

    def _begin_laser_task_from_ui(self):
        if getattr(self, "laser_task_active", False):
            return True
        if not self._laser_requested():
            return False
        if not (self.ser and self.ser.is_open):
            self.log_error("激光加工使能已取消：串口未连接。")
            self._reset_laser_task_ui()
            return False
        self.laser_power_permille = self._laser_power_from_ui()
        self.laser_task_active = True
        self.laser_preamble_needed = True
        self.motion_preamble_needed = True
        return True

    def _set_laser_button_visual(self, enabled):
        button = getattr(self, "laser_enable_toggle", None)
        if button is None:
            return
        if hasattr(button, "setText"):
            button.setText("激光关闭" if enabled else "激光开启")
        if hasattr(button, "setStyleSheet"):
            color = "#e74c3c" if enabled else "#34495e"
            button.setStyleSheet(f"background-color: {color}; color: white; font-weight: bold;")

    def _set_laser_button_checked(self, checked):
        button = getattr(self, "laser_enable_toggle", None)
        if button is None:
            return
        blocked = False
        if hasattr(button, "blockSignals"):
            blocked = button.blockSignals(True)
        if hasattr(button, "setChecked"):
            button.setChecked(bool(checked))
        if hasattr(button, "blockSignals"):
            button.blockSignals(blocked)
        self._set_laser_button_visual(bool(checked))

    def _write_laser_command(self, cmd):
        if not (self.ser and self.ser.is_open):
            return False
        self.ser.write((cmd + "\n").encode("ascii"))
        if hasattr(self, "log_display"):
            self.log_display.append(f"<font color='#bbbbbb'>TX {self.get_timestamp()} [LASER] {cmd}</font>")
        return True

    def _force_laser_disarm(self):
        try:
            if self.ser and self.ser.is_open:
                self._write_laser_command("LASER DISARM")
        except Exception:
            pass
        self._reset_laser_task_ui()

    def _send_laser_power_now(self):
        if not getattr(self, "laser_task_active", False):
            return False
        power = int(getattr(self, "laser_power_permille", self._laser_power_from_ui()))
        if self._write_laser_command(f"LASER POWER {power}"):
            self.pending_laser_power_permille = None
            return True
        return False

    def _flush_pending_laser_power(self, force=False):
        pending = getattr(self, "pending_laser_power_permille", None)
        if pending is None:
            return
        if not getattr(self, "laser_task_active", False):
            self.pending_laser_power_permille = None
            return
        if self.ser and self.ser.is_open and not getattr(self, "waiting_for_ack", False):
            self.laser_power_permille = int(pending)
            self._send_laser_power_now()

    def on_laser_power_changed(self, *_):
        self.laser_power_permille = self._laser_power_from_ui()
        if not getattr(self, "laser_task_active", False):
            return
        if self.ser and self.ser.is_open and not getattr(self, "waiting_for_ack", False):
            self._send_laser_power_now()
        else:
            self.pending_laser_power_permille = int(self.laser_power_permille)

    def on_laser_enable_toggled(self, checked):
        if checked:
            if not (self.ser and self.ser.is_open):
                self.log_error("激光开启失败：串口未连接。")
                self._reset_laser_task_ui()
                return
            self.laser_power_permille = self._laser_power_from_ui()
            self.laser_task_active = True
            self.laser_preamble_needed = False
            self.motion_preamble_needed = True
            self._set_laser_button_visual(True)
            try:
                self._write_laser_command(f"LASER POWER {int(self.laser_power_permille)}")
                self._write_laser_command("LASER ARM")
                self.laser_arm_sent_at = time.time()
            except Exception as exc:
                self.log_error(f"Laser enable failed: {exc}")
                self._force_laser_disarm()
        else:
            self._force_laser_disarm()

    def _reset_laser_task_ui(self):
        self.laser_task_active = False
        self.laser_preamble_needed = False
        self.laser_arm_sent_at = 0.0
        self.pending_laser_power_permille = None
        toggle = getattr(self, "laser_enable_toggle", None)
        if toggle is not None:
            self._set_laser_button_checked(False)
        label = getattr(self, "laser_status_label", None)
        if label is not None:
            label.setText("下位机状态: 断开")
            label.setStyleSheet("color: #aaaaaa; font-weight: bold;")

    def _update_laser_status(self, raw):
        match = re.search(r'(?:Lz:|laser=)(\d+),(\d+),(\d+),(\d+)', raw)
        if not match:
            return
        armed, ready, marking, power = (int(match.group(index)) for index in range(1, 5))
        self.laser_power_permille = power
        label = getattr(self, "laser_status_label", None)
        if armed == 0:
            text, color = "下位机状态: 断开", "#aaaaaa"
        elif ready == 0:
            text, color = "下位机状态: 准备中", "#f39c12"
        elif marking:
            text, color = f"下位机状态: 落笔 {power / 10.0:.1f}%", "#e74c3c"
        else:
            text, color = f"下位机状态: 抬笔 {power / 10.0:.1f}%", "#2ecc71"
        if label is not None:
            label.setText(text)
            label.setStyleSheet(f"color: {color}; font-weight: bold;")
        arm_age = time.time() - float(getattr(self, "laser_arm_sent_at", 0.0) or 0.0)
        if armed == 0 and getattr(self, "laser_task_active", False) and arm_age > 0.5:
            self._reset_laser_task_ui()

    def ui_to_mcu_xy(self, x, y):
        return x - self.L0 * 0.5, y

    def mcu_to_ui_xy(self, x, y):
        return x + self.L0 * 0.5, y

    def _write_motion_preamble(self):
        # Preamble commands are part of the same character-counted GcodeJob.
        return

    def load_gcode_job(self, commands, preview_path=None, append=False):
        """Queue real G-code without converting the command iterable to a list."""
        if not commands:
            return False
        accepted = GRBL_GCODE_SENDER.send(self, commands, append=append, send_path=None)
        if preview_path is not None:
            self.active_preview_path = preview_path
        return bool(accepted)

    def _motion_profile_preamble(self):
        profile = tuple(getattr(self, "_pending_motion_profile", ()) or ())
        self._pending_motion_profile = ()
        return profile

    def _prepare_motion_profile(self):
        speed_mm_s = float(self._read_run_speed_mm_s())
        accel_mm_s2 = float(self._read_run_accel_mm_s2())
        junction_deviation = float(getattr(self, "junction_dev", 0.02))
        if hasattr(self, "path_planner"):
            self.path_planner.accel_mm_s2 = max(1.0, accel_mm_s2)
            self.path_planner.junction_deviation = max(0.001, junction_deviation)

        commands = []
        if getattr(self, "microstep_dirty", False):
            ppr = int(self.microstep_combo.currentText())
            commands.append(build_ppr_line(ppr))
            self.current_ppr = ppr
            self.microstep_dirty = False
        commands.extend(
            (
                f"$110={max(1, int(round(speed_mm_s * 60.0)))}",
                f"$120={max(1, int(round(accel_mm_s2)))}",
                f"$11={max(1, int(round(junction_deviation * 1000.0)))}",
            )
        )
        self._pending_motion_profile = tuple(commands)

    def load_motion_gcode_job(self, commands, preview_path=None, append=False):
        """Queue geometry G-code with one GRBL motion profile per new task."""
        if getattr(self, "controller_reset_pending", False):
            reason = str(getattr(self, "controller_reset_reason", "") or "controller reset pending")
            self.log_error(
                f"Motion blocked: {reason}. Wait for Idle/Q:0/Seg:0/E:0 after Stop/reset before starting a new task."
            )
            return False
        sender_active = bool(self.waiting_for_ack or self.point_queue or getattr(self, "inflight_lines", None))
        controller_state = str(getattr(self, "last_controller_state", "") or "").lower()
        if not append and (sender_active or controller_state in ("run", "hold")):
            self.log_error(
                f"Motion blocked: current stream/controller is still active ({controller_state or 'pending ACK'}). "
                "Stop it or wait for Idle before starting a new task."
            )
            return False
        if not self._preflight_motion_path(preview_path):
            return False
        appending_active_motion = bool(
            append and (self.waiting_for_ack or self.point_queue or getattr(self, "inflight_lines", None))
        )
        if not appending_active_motion:
            try:
                self._prepare_motion_profile()
            except Exception as exc:
                self.log_error(f"Motion profile is invalid: {exc}")
                return False
            self.motion_profile_sync_requested = True
            self._begin_laser_task_from_ui()
        return self.load_gcode_job(commands, preview_path=preview_path, append=append)

    def _preflight_motion_path(self, preview_path):
        """Validate every reusable preview point before any command is queued."""
        if preview_path is None:
            self.log_error("Motion blocked: no preflight path was provided.")
            return False
        try:
            iterator = iter(preview_path)
        except TypeError:
            self.log_error("Motion blocked: the preflight path is not iterable.")
            return False
        if iterator is preview_path:
            self.log_error(
                "Motion blocked: the preflight path is a one-shot iterator and cannot be verified before streaming."
            )
            return False
        try:
            point_count = len(preview_path)
        except TypeError:
            point_count = "unknown"
        if point_count == 0:
            self.log_error("Motion blocked: the preflight path contains no points.")
            return False
        try:
            valid = self.validate_trajectory_points(preview_path, f"Formal motion path ({point_count} points)")
        except Exception as exc:
            self.log_error(f"Motion blocked: preflight traversal failed: {exc}")
            return False
        if not valid:
            return False
        self.log_display.append(
            f"<font color='#98c379'>Preflight PASS: verified all {point_count} motion path points.</font>"
        )
        return True

    def _request_controller_diagnostics(self):
        if self.ser and self.ser.is_open:
            if not getattr(self, "inflight_lines", None):
                self.ser.write(b"?")

    def refresh_ports(self):
        self.port_combo.clear()
        ps = [p.device for p in serial.tools.list_ports.comports()]
        if ps:
            self.port_combo.addItems(ps)

    def toggle_serial(self):
        if self.ser is None or not self.ser.is_open:
            try:
                port = self.port_combo.currentText()
                if not port: return
                self.ser = SerialThreadTransport(port, 115200, self)
                self.ser.open()
                self.last_controller_rx_at = time.time()
                self.serial_failure_reported = False
                self.serial_status.setText("已连接")
                self.serial_status.setStyleSheet("color: green;")
                self.btn_connect.setText("断开")
                self.motion_preamble_needed = True
                self.controller_capabilities = None
                self.microstep_dirty = True
                self.apply_microstep_setting()
                self.heartbeat_timer.start(200)
            except Exception as e:
                self.log_error(f"连接失败: {e}")
        else:
            try:
                if hasattr(self, "stop_motion"):
                    self.stop_motion()
                self._force_laser_disarm()
                if self.ser and self.ser.is_open:
                    self.ser.flush()
                time.sleep(0.02)
            except Exception:
                pass
            self._reset_laser_task_ui()
            self.ser.close()
            self.ser = None
            self.last_controller_state = ""
            self.last_controller_rx_at = 0.0
            self.heartbeat_timer.stop()
            self.serial_status.setText("未连接")
            self.serial_status.setStyleSheet("color: gray;")
            self.btn_connect.setText("连接")

    def send_heartbeat(self):
        if self.ser and self.ser.is_open and not self.waiting_for_ack:
            self.heartbeat_count += 1
            if not getattr(self, "inflight_lines", None):
                self.ser.write(b"?")

    def load_motion_queue(self, path, append=False, send_path=None):
        """Stream a sampled preview path as lazy real Cartesian G-code."""
        if not path:
            self.log_error("No valid motion path was generated.")
            return
        self.emergency_paused = False
        if hasattr(self, "_set_emergency_button_paused"):
            self._set_emergency_button_paused(False)
        if not append:
            prepared = self._motion_path_with_current_connector(path, send_path)
            if prepared is None:
                return
            path, send_path = prepared
            laser_enabled = None
        else:
            laser_enabled = bool(getattr(self, "laser_task_active", False))
        self.log_display.append(
            f"<font color='#bbbbbb'>mode=grbl_stream selected append={int(bool(append))} "
            f"preview_points={len(path)}</font>"
        )
        self.load_motion_gcode_job(
            self._iter_path_gcode(path, laser_enabled),
            preview_path=path,
            append=append,
        )

    def _iter_path_gcode(self, path, laser_enabled=None):
        if laser_enabled is None:
            laser_enabled = bool(getattr(self, "laser_task_active", False))
        marking = bool(laser_enabled)
        for point in path:
            x, y = self.ui_to_mcu_xy(float(point[0]), float(point[1]))
            feed = max(1, int(round(float(point[2])))) if len(point) > 2 else 300
            silent = bool(point[3]) if len(point) > 3 else False
            wants_mark = bool(laser_enabled and not silent)
            if wants_mark != marking:
                if wants_mark:
                    yield f"M4 S{self._laser_s_word()}"
                else:
                    yield "M5"
                marking = wants_mark
            if silent:
                yield f"G0 X{x:.3f} Y{y:.3f}"
            else:
                yield f"G1 X{x:.3f} Y{y:.3f} F{feed}"

    def _motion_path_with_current_connector(self, path, send_path=None):
        if not path:
            return None
        first = path[0]
        start_x, start_y = float(self.cur_x), float(self.cur_y)
        first_x, first_y = float(first[0]), float(first[1])
        if ((first_x - start_x) * (first_x - start_x) + (first_y - start_y) * (first_y - start_y)) <= 0.0025:
            return path, send_path
        try:
            feed_mm_s = max(0.1, float(first[2]) / 60.0) if len(first) > 2 else 1.0
            connector = self.generate_linear_path(start_x, start_y, first_x, first_y, feed_mm_s, silent=True)
            if not connector:
                self.log_error(
                    f"无法规划到轨迹起点的连接段: 当前({start_x:.2f},{start_y:.2f}) -> 起点({first_x:.2f},{first_y:.2f})"
                )
                return None
            path = connector + list(path)
            if hasattr(self, "set_planned_preview"):
                self.set_planned_preview(path, getattr(self, "preview_label", "轨迹预览") or "轨迹预览")
            self.log_display.append(
                f"<font color='#bbbbbb'>自动补充到轨迹起点连接段: ({start_x:.1f},{start_y:.1f}) -> ({first_x:.1f},{first_y:.1f})</font>"
            )
            return path, None
        except Exception as exc:
            self.log_error(f"轨迹起点连接段规划失败: {exc}")
            return None

    def _set_sender_status(self, mode, **stats):
        self.sender_mode = str(mode)
        current = dict(getattr(self, "sender_stats", {}) or {})
        current.update(stats)
        self.sender_stats = current
        queued = current.get("queued", current.get("queued_lines", "--"))
        inflight = current.get("inflight", current.get("inflight_lines", "--"))
        free = current.get("planner_free", current.get("free", "--"))
        rx_free = current.get("rx_free", getattr(self, "rx_free_hint", "--"))
        step_count = current.get("step_segment_count", "--")
        step_free = current.get("step_segment_free", "--")
        low = current.get("segment_low_water", current.get("low_water", "--"))
        underrun = current.get("underrun", current.get("underrun_ticks", "--"))
        underrun_count = current.get("segment_underrun_count", "--")
        rate_limited = current.get("rate_limited_segments", "--")
        refill_gap = current.get("max_refill_gap_ms", "--")
        free_min = current.get("planner_free_min", getattr(self, "planner_free_min", "--"))
        if free_min is None:
            free_min = "--"
        ack_ms = current.get("avg_ack_ms", "--")
        status_text = (
            f"Sender: {self.sender_mode}  queued={queued} inflight={inflight} "
            f"free={free}/{rx_free} min={free_min} sq={step_count}/{step_free} "
            f"low={low} underrun={underrun}/{underrun_count} rl={rate_limited} gap={refill_gap}ms avg_ack={ack_ms}"
        )
        label = getattr(self, "lbl_sender_mode", None)
        if label is not None:
            label.setText(status_text)
        elif getattr(self, "ser", None) is not None and self.ser.is_open and hasattr(self, "serial_status"):
            self.serial_status.setText(status_text)

    def _clear_text_sender_state(self):
        self.inflight_lines = []
        self.inflight_bytes = 0
        self.waiting_for_ack = False
        self.last_sent_motion = None
        self.planner_free_min = None
        self.ack_timeout_count = 0
        self.idle_ack_stall_polls = 0

    def _begin_controller_reset(self, reason):
        self.controller_reset_pending = True
        self.controller_reset_reason = str(reason)
        self.controller_reset_started_at = time.time()
        self.controller_reset_generation = int(getattr(self, "controller_reset_generation", 0)) + 1

    def _finish_controller_reset(self, raw=None):
        if not getattr(self, "controller_reset_pending", False):
            return
        age = max(0.0, time.time() - float(getattr(self, "controller_reset_started_at", 0.0) or 0.0))
        reason = str(getattr(self, "controller_reset_reason", "") or "controller reset")
        self.controller_reset_pending = False
        self.controller_reset_reason = ""
        self.controller_reset_started_at = 0.0
        self.motion_preamble_needed = True
        self.motion_profile_sync_requested = False
        self.timeout_timer.stop()
        detail = f" status={raw}" if raw else ""
        self.log_display.append(
            f"<font color='#98c379'>Controller reset drain complete after {age:.1f}s: {reason}.{detail}</font>"
        )

    def _update_planner_free_hint(self, free):
        free = max(0, int(free))
        self.planner_free_hint = free
        previous = getattr(self, "planner_free_min", None)
        self.planner_free_min = free if previous is None else min(int(previous), free)
        return free

    def _text_sender_limits(self):
        # Grbl character-counting streaming owns one fixed outstanding-byte
        # window. Bf RX-free is a diagnostic snapshot, not a new credit grant.
        return 64, 224

    def _line_from_motion_item(self, motion_item):
        if isinstance(motion_item, str):
            return motion_item.strip(), "grbl_stream", motion_item

        tx, ty, feed_rate = motion_item[0], motion_item[1], motion_item[2]
        slt = motion_item[3] if len(motion_item) > 3 else False
        mcu_tx, mcu_ty = self.ui_to_mcu_xy(tx, ty)
        gcode_raw = build_g1_line(
            mcu_tx,
            mcu_ty,
            feed_rate,
            self.sent_point_id + 1,
            limit_checked=True,
        )

        if self.plot_mode_combo.currentText() == "通讯发送内容":
            self.cur_x, self.cur_y = tx, ty
            if slt:
                self.history_x, self.history_y = [tx], [ty]
            else:
                self.history_x.append(tx)
                self.history_y.append(ty)
            ik = self.inverse_kinematics(tx, ty)
            if ik and ik[0] is not None:
                self.update_plot(ik[0], ik[1])
        return gcode_raw, "gcode_stream", motion_item

    @staticmethod
    def _line_requires_homing(line):
        text = str(line).lstrip().upper()
        return text.startswith("$J=") or re.match(r"^G0?[0-3](?=[^0-9]|$)", text) is not None

    @staticmethod
    def _line_is_long_running(line):
        return str(line).strip().upper() in ("$H", "$HS")

    def _oldest_inflight_entry(self):
        inflight = getattr(self, "inflight_lines", None) or []
        return inflight[0] if inflight else None

    def _should_log_stream_progress(self, line_id):
        line_id = int(line_id)
        total = int(getattr(self, "total_task_points", 0) or 0)
        return (
            line_id <= self.STREAM_LOG_FIRST
            or line_id % self.STREAM_LOG_EVERY == 0
            or (total > 0 and line_id >= total)
        )

    def _restart_ack_timer(self):
        oldest = self._oldest_inflight_entry()
        if oldest is None:
            self.timeout_timer.stop()
            return
        delay = self.HOME_STATUS_POLL_MS if self._line_is_long_running(oldest.get("line", "")) else self.ACK_TIMEOUT_MS
        self.timeout_timer.start(delay)

    def _abort_stalled_stream(self, reason, reset_controller=True):
        oldest = self._oldest_inflight_entry()
        line = oldest.get("line", "") if oldest else ""
        age = max(0.0, time.time() - float(oldest.get("sent_at", time.time()))) if oldest else 0.0
        self.log_error(f"{reason}; oldest pending line age={age:.1f}s line={line!r}. Stream cleared.")
        queue = getattr(self, "point_queue", None)
        if hasattr(queue, "clear"):
            queue.clear()
        else:
            self.point_queue = []
        self._clear_text_sender_state()
        self.stream_waiting_buffer = False
        self.last_sent_motion = None
        self.active_preview_path = []
        self.motion_preamble_needed = True
        self.motion_profile_sync_requested = False
        self.timeout_timer.stop()
        if reset_controller:
            self._begin_controller_reset(reason)
        if hasattr(self, "_force_laser_disarm"):
            self._force_laser_disarm()
        if reset_controller and self.ser and self.ser.is_open:
            self.ser.write(b"\x18")
            self.ser.write(b"?")

    def _check_serial_transport_error(self):
        transport = getattr(self, "ser", None)
        error = getattr(transport, "error", None) if transport is not None else None
        if error is None:
            self.serial_failure_reported = False
            return False
        if not getattr(self, "serial_failure_reported", False):
            self.serial_failure_reported = True
            self._abort_stalled_stream(f"Serial worker stopped: {error}", reset_controller=False)
        return True

    def _fill_ascii_sender_window(self):
        if not self.ser or not self.ser.is_open:
            return False
        if not self.point_queue:
            return False
        inflight = getattr(self, "inflight_lines", None)
        if inflight is None:
            inflight = []
            self.inflight_lines = inflight
            self.inflight_bytes = 0

        max_lines, max_bytes = self._text_sender_limits()
        sent_any = False
        while self.point_queue and len(inflight) < max_lines:
            if inflight and self._line_is_long_running(inflight[0].get("line", "")):
                break
            motion_item = self.point_queue[0]
            gcode_raw, mode, motion_record = self._line_from_motion_item(motion_item)
            if not gcode_raw:
                self.point_queue.pop(0)
                continue
            long_running = self._line_is_long_running(gcode_raw)
            if long_running and inflight:
                break
            if not self.board_only_debug and self._line_requires_homing(gcode_raw):
                if self.home_sensor_triggered:
                    self.log_error("Home switch is active; motion stream stopped.")
                    self.point_queue.clear()
                    return True
                if not self.is_homed:
                    self.log_error("Not homed; run homing before motion.")
                    self.point_queue.clear()
                    return True
            line_bytes = len(gcode_raw.encode("ascii", errors="ignore")) + 1
            if inflight and (self.inflight_bytes + line_bytes > max_bytes):
                break

            self.point_queue.pop(0)
            self.sent_point_id += 1
            self.last_sent_motion = motion_record
            if not sent_any:
                self._write_motion_preamble()
            ts = self.get_timestamp()
            tag = "gcode"
            log_msg = (
                f"TX {ts} [{tag} {self.sent_point_id}/{self.total_task_points}] "
                f"len={line_bytes} line={gcode_raw}"
            )
            if self._should_log_stream_progress(self.sent_point_id):
                self.log_display.append(f"<font color='#ffffff'>{log_msg}</font>")
            self.ser.write((gcode_raw + "\n").encode("ascii"))
            entry = {
                "line": gcode_raw,
                "bytes": line_bytes,
                "sent_at": time.time(),
                "mode": mode,
                "id": self.sent_point_id,
                "motion": motion_record,
            }
            inflight.append(entry)
            self.inflight_bytes = int(getattr(self, "inflight_bytes", 0)) + line_bytes
            sent_any = True
            if long_running:
                break

        if inflight:
            self.waiting_for_ack = True
            self._restart_ack_timer()
            self._set_sender_status(
                inflight[0].get("mode", "gcode_stream"),
                queued_lines=len(self.point_queue),
                inflight_lines=len(inflight),
                inflight_bytes=self.inflight_bytes,
            )
        return True

    def on_microstep_changed(self):
        self.microstep_dirty = True
        if self.ser and self.ser.is_open and not self.waiting_for_ack and not self.point_queue:
            self.apply_microstep_setting()

    def apply_microstep_setting(self):
        try:
            ppr = int(self.microstep_combo.currentText())
        except Exception as exc:
            self.log_error(f"细分/脉冲参数错误: {exc}")
            return False
        if self.waiting_for_ack or self.point_queue:
            self.log_display.append("<font color='yellow'>PPR 参数将在当前队列结束后应用</font>")
            return False
        if not (self.ser and self.ser.is_open):
            self.microstep_dirty = True
            return False
        line = build_ppr_line(ppr)
        if self.load_gcode_job([line]):
            self.current_ppr = ppr
            self.microstep_dirty = False
            if hasattr(self, "_reset_jog_anchor"):
                self._reset_jog_anchor()
            if hasattr(self, "update_jog_pps_preview"):
                self.update_jog_pps_preview()
            return True
        return False

    def check_serial_feedback(self):
        if self._check_serial_transport_error():
            return
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not raw: return
                self.last_controller_rx_at = time.time()
                if self.ser.in_waiting > 0:
                    QTimer.singleShot(0, self.check_serial_feedback)

                # 1. 独立处理下位机健康监控反馈 (心跳 OK)
                if raw.startswith("OK HEARTBEAT"):
                    tick_match = re.search(r'tick=(\d+)', raw)
                    err_match = re.search(r'err=(\d+)', raw)
                    gbuf_match = re.search(r'gbuf=\d+,(\d+)', raw)
                    if tick_match: self.lbl_mcu_tick.setText(f"MCU时间: {tick_match.group(1)} ms")
                    if err_match: self.lbl_mcu_err.setText(f"错误码: {err_match.group(1)}")
                    if gbuf_match:
                        capacity = int(getattr(self, "mcu_planner_capacity", 48))
                        self.lbl_mcu_gbuf.setText(f"缓冲区占用: {gbuf_match.group(1)} / {capacity}")
                    # 心跳响应在此处终结，绝对不执行后续运动队列逻辑
                    return

                if raw.startswith("OK HOME_SENSOR"):
                    h1_match = re.search(r'h1=(\d+)', raw)
                    h2_match = re.search(r'h2=(\d+)', raw)
                    h1 = int(h1_match.group(1)) if h1_match else 0
                    h2 = int(h2_match.group(1)) if h2_match else 0
                    if not self.board_only_debug:
                        self.home_sensor_triggered = (h1 == 1 or h2 == 1)
                    if hasattr(self, "lbl_mcu_interp"):
                        self.lbl_mcu_interp.setText(f"HOME_SENSOR h={h1},{h2}")
                    self.log_display.append(f"<font color='#98c379'>RX {self.get_timestamp()} {raw}</font>")
                    return

                if raw.startswith("OK ") and not (getattr(self, "inflight_lines", []) or []):
                    if getattr(self, "controller_reset_pending", False):
                        self.log_display.append(
                            f"<font color='orange'>RX {self.get_timestamp()} [STALE_POST_RESET] {raw}</font>"
                        )
                        return
                    self.waiting_for_ack = False
                    self.timeout_timer.stop()
                    self.log_display.append(f"<font color='#98c379'>RX {self.get_timestamp()} [SYSTEM_OK] {raw}</font>")
                    return

                # 2. 处理运动指令 ACK；系统 OK 只记录，不推进运动队列。
                if raw.lower().startswith("ok"):
                    ts = self.get_timestamp()
                    inflight = getattr(self, "inflight_lines", []) or []
                    ack = parse_ok_ack(raw)

                    if getattr(self, "controller_reset_pending", False) and not inflight:
                        self.log_display.append(f"<font color='orange'>RX {ts} [STALE_POST_RESET] {raw}</font>")
                        return

                    if not ack.matched:
                        self.log_display.append(
                            f"<font color='orange'>RX {ts} [OUT_OF_BAND_ACK] {raw}</font>"
                        )
                        return

                    if not inflight:
                        self.log_display.append(f"<font color='orange'>RX {ts} [STALE_ACK] {raw}</font>")
                        return

                    expected_entry = inflight.pop(0)
                    expected_line = expected_entry["line"]
                    self.inflight_bytes = max(
                        0,
                        int(getattr(self, "inflight_bytes", 0)) - int(expected_entry.get("bytes", 0)),
                    )
                    sent_at = expected_entry.get("sent_at")
                    ack_id = expected_entry.get("id", self.sent_point_id)
                    ack_mode = expected_entry.get("mode", getattr(self, "sender_mode", "gcode_stream"))
                    if str(expected_line).lstrip().upper().startswith("$H"):
                        self._apply_home_state("done")
                    self.waiting_for_ack = bool(inflight)
                    if not self.waiting_for_ack:
                        self.timeout_timer.stop()
                    self.ack_timeout_count = 0
                    self.stream_waiting_buffer = False
                    self.last_sent_motion = None
                    if sent_at is not None:
                        ack_ms = max(0.0, (time.time() - sent_at) * 1000.0)
                        prev = getattr(self, "avg_ack_ms", None)
                        self.avg_ack_ms = ack_ms if prev is None else (prev * 0.8 + ack_ms * 0.2)
                        self._set_sender_status(
                            ack_mode,
                            avg_ack_ms=f"{self.avg_ack_ms:.1f}ms",
                            queued_lines=len(getattr(self, "point_queue", [])),
                            inflight_lines=len(inflight),
                            inflight_bytes=int(getattr(self, "inflight_bytes", 0)),
                        )

                    if self._should_log_stream_progress(ack_id):
                        self.log_display.append(
                            f"<font color='#ffffff'>RX {ts} [ACK {ack_id}/{self.total_task_points}] {raw}</font>"
                        )
                        self.log_display.append(f"<font color='#00ff99'>MATCH line=OK</font>")

                    # ACK 只确认接收/入队；真实运动反馈只使用状态帧 MPos:x,y。
                    # 只有匹配当前 G-code 的 ACK 才能推进队列，避免 OK ENABLE/OK ZERO 误触发点动发送。
                    self.process_queue()
                    return

                # 3. 处理主动推送的状态包 <...>
                if raw.startswith('<') and '>' in raw:
                    self._update_laser_status(raw)
                    mode_match = re.match(r'<([^|>]+)', raw)
                    if mode_match:
                        self.last_controller_state = mode_match.group(1).strip().lower()
                    if getattr(self, "velocity_monitor", None) is not None:
                        self.velocity_monitor.process_mcu_status(raw, getattr(self, "current_ppr", 6400))

                    bf_match = re.search(r'Bf:(\d+),(\d+)', raw)
                    if bf_match:
                        self.mcu_planner_free = int(bf_match.group(1))
                        self.rx_free_hint = int(bf_match.group(2))
                        self._update_planner_free_hint(self.mcu_planner_free)
                        capacity = int(getattr(self, "mcu_planner_capacity", 48))
                        self.lbl_mcu_gbuf.setText(f"Planner free: {self.mcu_planner_free} / {capacity}")
                        if self.stream_waiting_buffer and self.mcu_planner_free > 0 and not self.waiting_for_ack:
                            self.stream_waiting_buffer = False
                            QTimer.singleShot(50, self.process_queue)

                    q_match = re.search(r'Q:(\d+)', raw)
                    if bf_match and q_match:
                        self.mcu_planner_capacity = int(bf_match.group(1)) + int(q_match.group(1))
                    sq_match = re.search(r'Seg:(\d+),(\d+),(\d+),(\d+)', raw)
                    if sq_match:
                        self.last_segment_count = int(sq_match.group(1))
                        self._set_sender_status(
                            getattr(self, "sender_mode", "gcode_stream"),
                            step_segment_count=int(sq_match.group(1)),
                            step_segment_free=int(sq_match.group(2)),
                            segment_low_water=int(sq_match.group(3)),
                            segment_underrun_count=int(sq_match.group(4)),
                            rx_free=getattr(self, "rx_free_hint", "--"),
                        )
                    pf_match = re.search(r'Pf:(\d+)', raw)
                    if pf_match:
                        fault_count = int(pf_match.group(1))
                        previous_fault_count = getattr(self, "planner_fault_count", None)
                        self.planner_fault_count = fault_count
                        if previous_fault_count is not None and fault_count > int(previous_fault_count):
                            stream_active = bool(
                                self.waiting_for_ack
                                or self.point_queue
                                or getattr(self, "inflight_lines", None)
                                or str(getattr(self, "last_controller_state", "")).lower() in ("run", "hold")
                            )
                            if stream_active and not getattr(self, "controller_reset_pending", False):
                                self._abort_stalled_stream(
                                    f"Controller planner segment preparation fault count increased to {fault_count}"
                                )
                                return
                            self.log_display.append(
                                f"<font color='orange'>Planner preparation fault counter advanced while idle: "
                                f"{previous_fault_count} -> {fault_count}; stream was not cleared.</font>"
                            )
                    rl_match = re.search(r'Rl:(\d+)', raw)
                    if rl_match:
                        self.rate_limited_segment_count = int(rl_match.group(1))
                        self._set_sender_status(
                            getattr(self, "sender_mode", "gcode_stream"),
                            rate_limited_segments=self.rate_limited_segment_count,
                        )
                    pg_match = re.search(r'Pg:(\d+)', raw)
                    if pg_match:
                        self._set_sender_status(
                            getattr(self, "sender_mode", "gcode_stream"),
                            max_refill_gap_ms=int(pg_match.group(1)),
                        )
                    if q_match: self.lbl_mcu_queue.setText(f"队列负载(Q): {q_match.group(1)}")

                    hz_match = re.search(r'Hz:(\d+)', raw)
                    if hz_match and hasattr(self, "lbl_mcu_hz"):
                        self.lbl_mcu_hz.setText(f"控制频率: {hz_match.group(1)} Hz")

                    pulse_match = re.search(r'JPos:(-?\d+),(-?\d+)', raw)
                    a1_match = re.search(r'A1:(\d+),(\d+),([-?\d]+),([-?\d]+)', raw)
                    a2_match = re.search(r'A2:(\d+),(\d+),([-?\d]+),([-?\d]+)', raw)
                    if pulse_match:
                        self.feedback_p1 = int(pulse_match.group(1))
                        self.feedback_p2 = int(pulse_match.group(2))
                        if hasattr(self, "_feedback_xy_from_pulses"):
                            try:
                                self.cur_x, self.cur_y = self._feedback_xy_from_pulses(
                                    (self.feedback_p1, self.feedback_p2)
                                )
                            except Exception as exc:
                                self.log_error(f"P: 脉冲正解失败: {exc}")
                    if a1_match:
                        self.feedback_a1_pps = (int(a1_match.group(3)), int(a1_match.group(4)))
                    if a2_match:
                        self.feedback_a2_pps = (int(a2_match.group(3)), int(a2_match.group(4)))

                    h_match = re.search(r'H:(\d),(\d)', raw)
                    if h_match:
                        h1, h2 = int(h_match.group(1)), int(h_match.group(2))
                        if not self.board_only_debug:
                            self.home_sensor_triggered = (h1 == 1 or h2 == 1)

                    hs_match = re.search(r'HS:(\w+)', raw)
                    if hs_match:
                        self._apply_home_state(hs_match.group(1))

                    if getattr(self, "controller_reset_pending", False):
                        q_zero = re.search(r'\bQ:(\d+)', raw)
                        e_zero = re.search(r'\bE:(\d+)', raw)
                        seg_zero = re.search(r'\bSeg:(\d+),', raw)
                        if (
                            str(getattr(self, "last_controller_state", "") or "").lower() == "idle"
                            and q_zero and int(q_zero.group(1)) == 0
                            and e_zero and int(e_zero.group(1)) == 0
                            and seg_zero and int(seg_zero.group(1)) == 0
                            and not (getattr(self, "inflight_lines", []) or [])
                            and not getattr(self, "point_queue", [])
                        ):
                            self._finish_controller_reset(raw)

                    match = re.search(r'MPos:(-?[\d.]+),(-?[\d.]+)', raw)
                    if match:
                        rx, ry = self.mcu_to_ui_xy(float(match.group(1)), float(match.group(2)))
                        q1, q2 = self.inverse_kinematics(rx, ry)
                        if hasattr(self, "feedback_pose_label"):
                            self.feedback_pose_label.setText(f"回传末端: X={rx:.3f}, Y={ry:.3f}")
                        if hasattr(self, "feedback_joint_label"):
                            if q1 is None or q2 is None:
                                self.feedback_joint_label.setText("回传角度: M1=不可解, M2=不可解")
                            else:
                                self.feedback_joint_label.setText(f"回传角度: M1={q1:.2f} deg, M2={q2:.2f} deg")
                        if hasattr(self, "feedback_pulse_label"):
                            p1 = getattr(self, "feedback_p1", None)
                            p2 = getattr(self, "feedback_p2", None)
                            a1 = getattr(self, "feedback_a1_pps", (None, None))
                            a2 = getattr(self, "feedback_a2_pps", (None, None))
                            self.feedback_pulse_label.setText(
                                f"脉冲/PPS: P={p1 if p1 is not None else '--'},{p2 if p2 is not None else '--'}  "
                                f"A1={a1[0] if a1[0] is not None else '--'}/{a1[1] if a1[1] is not None else '--'}  "
                                f"A2={a2[0] if a2[0] is not None else '--'}/{a2[1] if a2[1] is not None else '--'}"
                            )
                        if hasattr(self, "append_feedback_point"):
                            self.append_feedback_point(rx, ry)
                        if getattr(self, "velocity_monitor", None) is not None:
                            self.velocity_monitor.process_new_data(f"X{rx:.3f} Y{ry:.3f}")
                        self.update_plot()
                    return

                # 4. 处理错误报警
                if raw.startswith("STAT"):
                    self._update_laser_status(raw)
                    if getattr(self, "velocity_monitor", None) is not None:
                        self.velocity_monitor.process_mcu_status(raw, getattr(self, "current_ppr", 6400))

                    tick_match = re.search(r't=(\d+)', raw)
                    err_match = re.search(r'\be=(\d+)', raw)
                    bf_match = re.search(r'\bbf=(\d+),(\d+)', raw)
                    q_match = re.search(r'\bq=(\d+)', raw)
                    mode_match = re.search(r'\bm=([A-Za-z]+)', raw)
                    if mode_match:
                        self.last_controller_state = mode_match.group(1).strip().lower()
                    pps_match = re.search(r'\bpps=(-?\d+),(-?\d+)', raw)
                    tgt_match = re.search(r'\btgt=(-?\d+),(-?\d+)', raw)
                    en_match = re.search(r'\ben=(\d+),(\d+)', raw)
                    home_match = re.search(r'\bh=(\d+),(\d+)', raw)
                    hs_match = re.search(r'\bhs=([A-Za-z0-9_]+)', raw)
                    he_match = re.search(r'\bhe=(\d+)', raw)
                    if tick_match:
                        self.lbl_mcu_tick.setText(f"MCU时间: {tick_match.group(1)} ms")
                    if err_match:
                        self.lbl_mcu_err.setText(f"错误码: {err_match.group(1)}")
                    if bf_match:
                        self.mcu_planner_free = int(bf_match.group(1))
                        self.rx_free_hint = int(bf_match.group(2))
                        if q_match:
                            self.mcu_planner_capacity = self.mcu_planner_free + int(q_match.group(1))
                        self._update_planner_free_hint(self.mcu_planner_free)
                        capacity = int(getattr(self, "mcu_planner_capacity", 48))
                        self.lbl_mcu_gbuf.setText(f"Planner free: {self.mcu_planner_free} / {capacity}")
                    sq_match = re.search(r'\bsq=(\d+),(\d+)', raw)
                    if sq_match:
                        self.last_segment_count = int(sq_match.group(1))
                    if home_match and not self.board_only_debug:
                        self.home_sensor_triggered = (home_match.group(1) == "1" or home_match.group(2) == "1")
                    if hs_match:
                        self._apply_home_state(hs_match.group(1))
                    if hasattr(self, "lbl_mcu_interp") and (hs_match or pps_match or home_match):
                        mode = mode_match.group(1) if mode_match else "--"
                        hs = hs_match.group(1) if hs_match else "--"
                        he = he_match.group(1) if he_match else "--"
                        en = f"{en_match.group(1)},{en_match.group(2)}" if en_match else "--,--"
                        pps = f"{pps_match.group(1)},{pps_match.group(2)}" if pps_match else "--,--"
                        tgt = f"{tgt_match.group(1)},{tgt_match.group(2)}" if tgt_match else "--,--"
                        h = f"{home_match.group(1)},{home_match.group(2)}" if home_match else "--,--"
                        self.lbl_mcu_interp.setText(
                            f"运动:{mode} hs={hs} he={he} en={en} pps={pps} tgt={tgt} h={h}"
                        )
                    self.log_display.append(f"<font color='#98c379'>RX {self.get_timestamp()} {raw}</font>")
                    return

                if "error:" in raw.lower() or raw.upper().startswith("ERR"):
                    failed_entry = None
                    inflight = getattr(self, "inflight_lines", []) or []
                    if (
                        raw.lower().startswith("error:15")
                        and inflight
                        and self._line_is_long_running(inflight[0].get("line", ""))
                    ):
                        self.log_display.append(
                            "<font color='orange'>Ignored stale error:15 while homing is pending; "
                            "$H/$HS cannot generate a planner path error. Querying status.</font>"
                        )
                        if self.ser and self.ser.is_open:
                            self.ser.write(b"?")
                        self._restart_ack_timer()
                        return
                    if inflight:
                        failed_entry = inflight.pop(0)
                        self.inflight_bytes = max(
                            0,
                            int(getattr(self, "inflight_bytes", 0)) - int(failed_entry.get("bytes", 0)),
                        )
                    if failed_entry and str(failed_entry.get("line", "")).lstrip().upper().startswith("$H"):
                        self._apply_home_state("error")
                    self.waiting_for_ack = bool(inflight)
                    if raw.lower().startswith("error:8"):
                        self.log_display.append(
                            "<font color='orange'>RX error:8，下位机 pending/buffer 忙；暂停发送、查询状态，不急停。</font>"
                        )
                        self.stream_waiting_buffer = True
                        if not self.waiting_for_ack:
                            self.timeout_timer.stop()
                        failed_motion = failed_entry.get("motion") if failed_entry else None
                        if failed_motion is not None:
                            self.point_queue.insert(0, failed_motion)
                            self.sent_point_id = max(0, self.sent_point_id - 1)
                        if self.ser and self.ser.is_open:
                            self.ser.write(b"?")
                        self._force_laser_disarm()
                        return
                    self.log_display.append(f"<font color='red'>控制器报警: {raw}</font>")
                    if self.board_only_debug and raw.lower().startswith("error:5"):
                        # 仅连接控制板调试时，error:5 常见原因是下位机已在回零/忙状态。
                        # 此时不自动急停，避免反复把控制器打入 ESTOP；接入真实 HOME 开关后关闭 board_only_debug。
                        self.is_homed = True
                        self.home_sensor_triggered = False
                        self.waiting_for_ack = False
                        self.timeout_timer.stop()
                        return
                    if raw.lower().startswith("error:15"):
                        if getattr(self, "controller_reset_pending", False) and not inflight:
                            self.log_display.append(
                                f"<font color='orange'>RX {self.get_timestamp()} [STALE_POST_RESET] {raw}</font>"
                            )
                            return
                        self.log_display.append(
                            "<font color='orange'>运动被下位机拒绝，正在执行软复位并等待 Idle/Q:0/Seg:0/E:0。</font>"
                        )
                        self._abort_stalled_stream("Controller rejected motion with error:15")
                        self._request_controller_diagnostics()
                        return
                    self.stop_motion()
                    return

                # 5. 其他杂项信息
                self.log_display.append(f"<font color='#98c379'>RX {self.get_timestamp()} {raw}</font>")
            except Exception as exc:
                self.log_error(f"Serial feedback processing failed: {exc}")

    def handle_timeout(self):
        if not self.waiting_for_ack:
            return
        if self._check_serial_transport_error():
            return
        if not self.ser or not self.ser.is_open:
            self._abort_stalled_stream("Serial link closed while waiting for controller ACK", reset_controller=False)
            return

        oldest = self._oldest_inflight_entry()
        if oldest is None:
            self._clear_text_sender_state()
            self.timeout_timer.stop()
            return

        self.ack_timeout_count += 1
        line = oldest.get("line", "")
        age = max(0.0, time.time() - float(oldest.get("sent_at", time.time())))
        if self._line_is_long_running(line):
            if self.ack_timeout_count == 1 or self.ack_timeout_count % 10 == 0:
                self.log_display.append(
                    f"<font color='#bbbbbb'>Homing in progress: waiting {age:.1f}s for {line} completion ACK.</font>"
                )
            self.ser.write(b"?")
            self.timeout_timer.start(self.HOME_STATUS_POLL_MS)
            return

        self.stream_waiting_buffer = True
        self.ser.write(b"?")
        state = str(getattr(self, "last_controller_state", "") or "").lower()
        last_rx_at = float(getattr(self, "last_controller_rx_at", 0.0) or 0.0)
        controller_age = max(0.0, time.time() - last_rx_at) if last_rx_at > 0.0 else 0.0
        planner_free = int(getattr(self, "mcu_planner_free", 0) or 0)
        segment_count = int(getattr(self, "last_segment_count", 0) or 0)
        capacity = int(getattr(self, "mcu_planner_capacity", 48) or 48)
        idle_empty = state == "idle" and planner_free >= capacity and segment_count == 0
        if idle_empty and age >= self.IDLE_ACK_STALL_LIMIT_S:
            self.idle_ack_stall_polls = int(getattr(self, "idle_ack_stall_polls", 0)) + 1
        else:
            self.idle_ack_stall_polls = 0

        if last_rx_at > 0.0 and controller_age >= self.CONTROLLER_SILENCE_LIMIT_S:
            self._abort_stalled_stream(
                f"Controller produced no feedback for {controller_age:.1f}s while an ACK was pending"
            )
            return
        if self.idle_ack_stall_polls >= self.IDLE_ACK_STALL_POLLS:
            self._abort_stalled_stream(
                "Controller is Idle with an empty planner/segment queue but a G-code ACK is still pending"
            )
            return

        if self.ack_timeout_count == 1 or self.ack_timeout_count % 10 == 0:
            self.log_display.append(
                f"<font color='orange'>Waiting for ok ({age:.1f}s, controller={state or 'unknown'}); "
                "queried status and did not resend G-code.</font>"
            )
        self.timeout_timer.start(self.ACK_RECHECK_MS)

    def process_gcode_stream(self):
        return self._fill_ascii_sender_window()

    def process_simulated_queue(self):
        if not self.point_queue or self.waiting_for_ack:
            return
        if (not self.board_only_debug) and self.home_sensor_triggered:
            self.log_error("Home sensor triggered; simulated queue stopped.")
            self.point_queue = []
            return
        if (not self.board_only_debug) and (not self.is_homed):
            self.log_error("Not homed; run homing before motion.")
            self.point_queue = []
            return

        motion_item = self.point_queue.pop(0)
        if isinstance(motion_item, str):
            self._set_sender_status("grbl_stream", queued_lines=len(self.point_queue), inflight_lines=0)
            QTimer.singleShot(10, self.process_queue)
            return

        tx, ty, feed_rate = motion_item[0], motion_item[1], motion_item[2]
        slt = motion_item[3] if len(motion_item) > 3 else False
        self._set_sender_status("simulated_queue", queued_lines=len(self.point_queue), inflight_lines=0)
        self.last_sent_motion = (tx, ty, feed_rate, slt)
        self.sent_point_id += 1

        prev_x, prev_y = self.cur_x, self.cur_y
        self.cur_x, self.cur_y = tx, ty
        if slt:
            self.history_x, self.history_y = [tx], [ty]
        else:
            self.history_x.append(tx)
            self.history_y.append(ty)

        import math as _math
        dx = tx - prev_x
        dy = ty - prev_y
        dist = _math.hypot(dx, dy)
        feed_mm_s = feed_rate / 60.0
        dt = dist / feed_mm_s if feed_mm_s > 0.001 else self.dt

        monitor = getattr(self, "velocity_monitor", None)
        if monitor is not None and hasattr(monitor, "process_tcp_point"):
            monitor.process_tcp_point(tx, ty, dt)

        ik = self.inverse_kinematics(tx, ty)
        if ik and ik[0] is not None:
            self.update_plot(ik[0], ik[1])
        QTimer.singleShot(10, self.process_queue)

    def process_queue(self):
        if getattr(self, "emergency_paused", False):
            return
        self._flush_pending_laser_power()
        if self.ser is not None:
            if self._check_serial_transport_error():
                return
            if self.ser.is_open:
                if not self.point_queue:
                    return
                self.process_gcode_stream()
            return
        self.process_simulated_queue()
