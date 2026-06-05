import re
import time

import serial
import serial.tools.list_ports
from PySide6.QtCore import QTimer

from .serial_protocol import build_g1_line, build_ppr_line, parse_ok_ack


class ScaraSerialMixin:
    def ui_to_mcu_xy(self, x, y):
        return x - self.L0 * 0.5, y

    def mcu_to_ui_xy(self, x, y):
        return x + self.L0 * 0.5, y

    def _write_motion_preamble(self):
        if not self.ser or not self.ser.is_open or not self.motion_preamble_needed:
            return
        for cmd in ("CLEAR_ERROR", "ENABLE 1"):
            self.ser.write((cmd + "\n").encode('ascii'))
            self.log_display.append(
                f"<font color='#bbbbbb'>TX {self.get_timestamp()} [MOTION_PREP] {cmd}</font>"
            )
        self.motion_preamble_needed = False

    def _request_controller_diagnostics(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b"ERRORS\n?\n")

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
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.serial_status.setText("已连接")
                self.serial_status.setStyleSheet("color: green;")
                self.btn_connect.setText("断开")
                self.motion_preamble_needed = True
                self.microstep_dirty = True
                self.apply_microstep_setting()
                self.heartbeat_timer.start(200) 
            except Exception as e:
                self.log_error(f"连接失败: {e}")
        else:
            self.ser.close()
            self.heartbeat_timer.stop()
            self.serial_status.setText("未连接")
            self.serial_status.setStyleSheet("color: gray;")
            self.btn_connect.setText("连接")

    def send_heartbeat(self):
        if self.ser and self.ser.is_open and not self.waiting_for_ack:
            self.heartbeat_count += 1
            self.ser.write(f"HEARTBEAT {self.heartbeat_count}\n".encode('ascii'))

    def load_motion_queue(self, path, append=False):
        # 统一装载轨迹，避免各个按钮重复维护计数器。
        if not path:
            self.log_error("未生成有效轨迹")
            return
        if getattr(self, "microstep_dirty", False) and self.ser and self.ser.is_open:
            self.apply_microstep_setting()
        if append and (self.waiting_for_ack or self.point_queue):
            self.point_queue.extend(path)
            self.total_task_points += len(path)
        else:
            self.sent_point_id = 0
            self.total_task_points = len(path)
            self.task_start_time = time.time()
            self.point_queue = path
        self.process_queue()

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
        if self.send_ascii_line(line, "PPR"):
            self.current_ppr = ppr
            self.microstep_dirty = False
            self.send_ascii_line("PARAMS", "PARAMS")
            return True
        return False

    def check_serial_feedback(self):
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not raw: return

                # 1. 独立处理下位机健康监控反馈 (心跳 OK)
                if raw.startswith("OK HEARTBEAT"):
                    tick_match = re.search(r'tick=(\d+)', raw)
                    err_match = re.search(r'err=(\d+)', raw)
                    gbuf_match = re.search(r'gbuf=\d+,(\d+)', raw)
                    if tick_match: self.lbl_mcu_tick.setText(f"MCU时间: {tick_match.group(1)} ms")
                    if err_match: self.lbl_mcu_err.setText(f"错误码: {err_match.group(1)}")
                    if gbuf_match: self.lbl_mcu_gbuf.setText(f"缓冲区占用: {gbuf_match.group(1)} / 32")
                    # 心跳响应在此处终结，绝对不执行后续运动队列逻辑
                    return 

                # 2. 处理运动指令 ACK；系统 OK 只记录，不推进运动队列。
                if raw.lower().startswith("ok"):
                    ts = self.get_timestamp()
                    ack = parse_ok_ack(raw, self.last_sent_package.strip())

                    if not (ack.rx_checksum and ack.rx_line):
                        self.log_display.append(f"<font color='#98c379'>RX {ts} [OK] {raw}</font>")
                        return

                    if not ack.matched:
                        self.log_display.append(
                            f"<font color='orange'>RX {ts} [OUT_OF_BAND_ACK] {raw}</font>"
                        )
                        self.log_display.append(
                            f"<font color='orange'>MISMATCH expected_cs={ack.expected_checksum} rx_cs={ack.rx_checksum} expected_line={self.last_sent_package.strip()}</font>"
                        )
                        return

                    if not self.waiting_for_ack:
                        self.log_display.append(f"<font color='orange'>RX {ts} [STALE_ACK] {raw}</font>")
                        return

                    self.timeout_timer.stop()
                    self.waiting_for_ack = False
                    self.ack_timeout_count = 0
                    self.stream_waiting_buffer = False
                    self.last_sent_motion = None

                    self.log_display.append(f"<font color='#ffffff'>RX {ts} [ACK {self.sent_point_id}/{self.total_task_points}] {raw}</font>")
                    self.log_display.append(f"<font color='#00ff99'>MATCH cs={ack.expected_checksum} line=OK</font>")
                    
                    # ACK line 是下位机回显的目标命令，不是真实运动反馈；真实反馈只使用状态帧 M:x,y。
                    # 只有匹配当前 G-code 的 ACK 才能推进队列，避免 OK ENABLE/OK ZERO 误触发点动发送。
                    self.process_queue()
                    return

                # 3. 处理主动推送的状态包 <...>
                if raw.startswith('<') and '>' in raw:
                    bf_match = re.search(r'Bf:(\d+),(\d+)', raw)
                    if bf_match:
                        self.mcu_planner_free = int(bf_match.group(2))
                        self.lbl_mcu_gbuf.setText(f"Planner free: {self.mcu_planner_free} / 32")
                        if self.stream_waiting_buffer and self.mcu_planner_free > 0 and not self.waiting_for_ack:
                            self.stream_waiting_buffer = False
                            QTimer.singleShot(50, self.process_queue)

                    q_match = re.search(r'Q:(\d+)', raw)
                    if q_match: self.lbl_mcu_queue.setText(f"队列负载(Q): {q_match.group(1)}")

                    pulse_match = re.search(r'P:([-?\d]+),([-?\d]+)', raw)
                    a1_match = re.search(r'A1:(\d+),(\d+),([-?\d]+),([-?\d]+)', raw)
                    a2_match = re.search(r'A2:(\d+),(\d+),([-?\d]+),([-?\d]+)', raw)
                    if pulse_match:
                        self.feedback_p1 = int(pulse_match.group(1))
                        self.feedback_p2 = int(pulse_match.group(2))
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
                    if hs_match and hs_match.group(1).lower() == "done": 
                        self.is_homed = True
                    
                    match = re.search(r'M:([-?\d.]+),([-?\d.]+)', raw)
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
                        if self.plot_mode_combo.currentText() == "通讯接收内容":
                            self.cur_x, self.cur_y = rx, ry
                            ik = self.inverse_kinematics(rx, ry)
                            if ik and ik[0] is not None:
                                self.update_plot(ik[0], ik[1])
                        else:
                            self.update_plot()
                    return

                # 4. 处理错误报警
                if "error:" in raw.lower():
                    if raw.lower().startswith("error:8"):
                        self.log_display.append(
                            "<font color='orange'>RX error:8，下位机 pending/buffer 忙；暂停发送、查询状态，不急停。</font>"
                        )
                        self.stream_waiting_buffer = True
                        self.waiting_for_ack = False
                        self.timeout_timer.stop()
                        if self.last_sent_motion is not None:
                            self.point_queue.insert(0, self.last_sent_motion)
                            self.last_sent_motion = None
                            self.sent_point_id = max(0, self.sent_point_id - 1)
                        if self.ser and self.ser.is_open:
                            self.ser.write(b"?\n")
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
                        self.log_display.append(
                            "<font color='orange'>运动被下位机拒绝，暂停队列并查询 ERRORS/状态；不自动急停。</font>"
                        )
                        self.point_queue = []
                        self.waiting_for_ack = False
                        self.timeout_timer.stop()
                        self.last_sent_motion = None
                        self.motion_preamble_needed = True
                        self._request_controller_diagnostics()
                        return
                    self.emergency_stop()
                    return

                # 5. 其他杂项信息
                self.log_display.append(f"<font color='#98c379'>RX {self.get_timestamp()} {raw}</font>")
            except Exception as e: 
                pass

    def handle_timeout(self):
        if self.waiting_for_ack and self.ser and self.ser.is_open:
            self.ack_timeout_count += 1
            self.log_display.append(
                f"<font color='orange'>等待 ok 超时 {self.ack_timeout_count} 次，查询状态，不重发 G-code</font>"
            )
            self.stream_waiting_buffer = True
            self.ser.write(b"?\n")
            self.timeout_timer.start(1500) 

    def process_queue(self):
        if not self.point_queue or self.waiting_for_ack: return
        if (not self.board_only_debug) and self.home_sensor_triggered:
            self.log_error("传感器触发，轨迹终止")
            self.point_queue = []
            return
        if (not self.board_only_debug) and (not self.is_homed) and "$H" not in self.last_sent_package:
             self.log_error("未回零，请先执行寻原点")
             self.point_queue = []
             return
        tx, ty, feed_rate, slt = self.point_queue.pop(0)
        self.last_sent_motion = (tx, ty, feed_rate, slt)
        self.sent_point_id += 1
        mcu_tx, mcu_ty = self.ui_to_mcu_xy(tx, ty)
        gcode_raw = build_g1_line(mcu_tx, mcu_ty, feed_rate, self.sent_point_id, limit_checked=True)
        gcode_line = gcode_raw + "\n"
        self.last_sent_cs = self.calculate_checksum(gcode_raw)
        self.last_sent_package = gcode_raw
        if self.plot_mode_combo.currentText() == "通讯发送内容":
            self.cur_x, self.cur_y = tx, ty
            if slt: self.history_x, self.history_y = [tx], [ty]
            else: self.history_x.append(tx); self.history_y.append(ty)
            ik = self.inverse_kinematics(tx, ty)
            if ik and ik[0] is not None: self.update_plot(ik[0], ik[1])
        ts = self.get_timestamp()
        log_msg = f"TX {ts} [point {self.sent_point_id}/{self.total_task_points}] cs={self.last_sent_cs} len={len(gcode_line)} line={gcode_raw}"
        self.log_display.append(f"<font color='#ffffff'>{log_msg}</font>")
        if self.ser and self.ser.is_open:
            self._write_motion_preamble()
            self.ser.write(gcode_line.encode('ascii'))
            self.waiting_for_ack = True  
            self.timeout_timer.start(1000) 
        else:
            # ---- ??????????? / ???????? dt ----
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
