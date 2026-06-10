from pathlib import Path
import math
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from SCARA_UI.communication.motion_senders import (
    ASCII_LEGACY_G1_SENDER,
    BUFFERED_BINARY_SENDER,
    HOST_TIMED_SEGMENT_SENDER,
    select_motion_sender,
)
from SCARA_UI.communication.binary_trajectory_protocol import (
    FLAG_HOST_TIMED,
    TYPE_ACK,
    TYPE_BEGIN,
    build_frame,
    joint_deg_to_pulse,
)
from SCARA_UI.communication.serial_mixin import ScaraSerialMixin
from SCARA_UI.communication.serial_protocol import checksum
from SCARA_UI.motion.motion_mixin import HostTimedSegment, ScaraMotionMixin


class DummySerial:
    def __init__(self, is_open=True):
        self.is_open = is_open
        self.writes = []
        self.rx_lines = []
        self.rx_bytes = bytearray()

    def write(self, data):
        self.writes.append(bytes(data))

    @property
    def in_waiting(self):
        return len(self.rx_lines)

    def readline(self):
        return self.rx_lines.pop(0).encode("ascii")

    def read(self, size=1):
        if not self.rx_bytes:
            return b""
        take = min(int(size), len(self.rx_bytes))
        data = bytes(self.rx_bytes[:take])
        del self.rx_bytes[:take]
        return data


class DummyTimer:
    def __init__(self):
        self.started = []

    def start(self, delay):
        self.started.append(int(delay))

    def stop(self):
        pass


class DummyLog:
    def __init__(self):
        self.items = []

    def append(self, item):
        self.items.append(str(item))


class DummyText:
    def __init__(self, value):
        self.value = str(value)

    def currentText(self):
        return self.value


class SenderWindowDummy(ScaraSerialMixin):
    def __init__(self):
        self.ser = DummySerial()
        self.point_queue = [f"G1 A{i} B{i} T100 ;ID={i} HOST=1" for i in range(12)]
        self.waiting_for_ack = False
        self.board_only_debug = True
        self.home_sensor_triggered = False
        self.is_homed = True
        self.last_sent_package = ""
        self.last_sent_motion = None
        self.sent_point_id = 0
        self.total_task_points = len(self.point_queue)
        self.mcu_planner_free = 32
        self.motion_preamble_needed = False
        self.inflight_lines = []
        self.inflight_bytes = 0
        self.log_display = DummyLog()
        self.timeout_timer = DummyTimer()
        self.sender_stats = {}

    def calculate_checksum(self, line):
        return f"{sum(line.encode('ascii')) & 0xFF:02X}"

    def get_timestamp(self):
        return "test"


class SelectorDummy:
    def __init__(self, serial_open=True):
        self.ser = DummySerial(serial_open) if serial_open is not None else None
        self.host_timed_segment_mode = False
        self.waiting_for_ack = False
        self.point_queue = []


class TimedUploadDummy(ScaraMotionMixin):
    def __init__(self, text_mode=False):
        self.ser = DummySerial()
        self.waiting_for_ack = False
        self.point_queue = []
        self.host_timed_segment_mode = bool(text_mode)
        self.log_display = DummyLog()
        self.binary_points = None
        self.text_segments = None
        self.published = None

    def log_error(self, message):
        raise AssertionError(message)

    def _upload_binary_points(self, points, label=""):
        self.binary_points = list(points)
        return True

    def _queue_gcode_segments(self, segments, label=""):
        self.text_segments = list(segments)
        return True

    def _publish_commanded_segments(self, segments):
        self.published = list(segments)


class TimedPathDummy(TimedUploadDummy):
    def __init__(self):
        super().__init__(text_mode=False)
        from SCARA_UI.core.kinematics import FiveBarKinematics
        from SCARA_UI.trajectory.look_ahead import LookAheadPlanner

        self.kinematics = FiveBarKinematics()
        self.path_planner = LookAheadPlanner(accel_mm_s2=100.0)
        self.current_ppr = 3200
        self.cur_x, self.cur_y = self.kinematics.forward(90.0, 90.0)
        self.feedback_p1, self.feedback_p2 = self._joint_deg_to_pulse(
            *self.inverse_kinematics(self.cur_x, self.cur_y)
        )
        self.binary_motion_active = False
        self.jog_target_xy = None
        self.jog_target_pulses = None
        self.jog_pending_start_pulses = None
        self.jog_pending_target_pulses = None
        self.jog_roundtrip_origin_pulses = None

    def inverse_kinematics(self, x, y):
        return self.kinematics.inverse(x, y)


class FeedbackStateDummy(ScaraSerialMixin, ScaraMotionMixin):
    def __init__(self, plot_mode):
        from SCARA_UI.core.kinematics import FiveBarKinematics

        self.ser = DummySerial()
        self.kinematics = FiveBarKinematics()
        self.current_ppr = 3200
        self.L0 = self.kinematics.config.base_distance
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.plot_mode_combo = DummyText(plot_mode)
        self.velocity_monitor = None
        self.board_only_debug = True
        self.binary_motion_active = False
        self.jog_pending_target_pulses = None
        self.log_display = DummyLog()
        self.plot_updates = 0

    def update_plot(self, *args, **kwargs):
        self.plot_updates += 1

    def log_error(self, message):
        raise AssertionError(message)


def assert_true(value, message):
    if not value:
        raise AssertionError(message)


def check_selection():
    owner = SelectorDummy(serial_open=True)
    assert_true(
        select_motion_sender(owner, append=False, send_path=[(1, 2)]) is BUFFERED_BINARY_SENDER,
        "normal connected trajectory must select buffered binary",
    )

    owner.host_timed_segment_mode = True
    assert_true(
        select_motion_sender(owner, append=False, send_path=[(1, 2)]) is HOST_TIMED_SEGMENT_SENDER,
        "host timed mode must be explicit",
    )

    owner.host_timed_segment_mode = False
    assert_true(
        select_motion_sender(owner, append=True, send_path=[(1, 2)]) is ASCII_LEGACY_G1_SENDER,
        "append mode must use the streaming ASCII queue",
    )

    owner.waiting_for_ack = True
    assert_true(
        select_motion_sender(owner, append=False, send_path=[(1, 2)]) is ASCII_LEGACY_G1_SENDER,
        "busy text sender must not start binary upload",
    )

    owner = SelectorDummy(serial_open=None)
    assert_true(
        select_motion_sender(owner, append=False, send_path=[(1, 2)]) is ASCII_LEGACY_G1_SENDER,
        "offline preview must use the simulated ASCII queue",
    )


def check_window_accounting():
    owner = SenderWindowDummy()
    owner.process_queue()

    assert_true(len(owner.inflight_lines) > 1, "sender regressed to stop-and-wait")
    assert_true(len(owner.inflight_lines) <= 6, "sender exceeded line window")
    assert_true(owner.inflight_bytes <= 180, "sender exceeded RX byte budget")
    assert_true(owner.waiting_for_ack, "sender must track outstanding ACKs")
    assert_true(
        owner.inflight_bytes == sum(item["bytes"] for item in owner.inflight_lines),
        "inflight byte accounting mismatch",
    )
    assert_true(len(owner.ser.writes) == len(owner.inflight_lines), "serial write/inflight mismatch")

    constrained = SenderWindowDummy()
    constrained.rx_free_hint = 100
    constrained.process_queue()
    assert_true(constrained.inflight_bytes <= 84, "sender ignored RX byte budget")


def check_ack_window_progress():
    owner = SenderWindowDummy()
    owner.process_queue()
    initial_inflight = list(owner.inflight_lines)
    initial_remaining = len(owner.point_queue)
    assert_true(len(initial_inflight) > 1, "ACK test requires a populated send window")

    first = initial_inflight[0]["line"]
    owner.ser.rx_lines.append(f"ok seq=1 cs={checksum(first)} line={first}\n")
    owner.check_serial_feedback()

    assert_true(
        all(item["line"] != first for item in owner.inflight_lines),
        "matching ACK did not release the oldest inflight line",
    )
    assert_true(
        len(owner.point_queue) < initial_remaining,
        "matching ACK did not refill the sender window",
    )
    assert_true(
        owner.inflight_bytes == sum(item["bytes"] for item in owner.inflight_lines),
        "ACK release broke inflight byte accounting",
    )

    writes_before_heartbeat = len(owner.ser.writes)
    owner.send_heartbeat()
    assert_true(
        len(owner.ser.writes) == writes_before_heartbeat,
        "heartbeat competed with an active motion send window",
    )


def check_timed_jog_transport():
    segments = [
        HostTimedSegment(10, 0, 100),
        HostTimedSegment(20, 0, 80, flags=1),
    ]
    owner = TimedUploadDummy(text_mode=False)
    assert_true(owner._upload_host_segments(segments, "jog"), "timed binary upload failed")
    assert_true(owner.text_segments is None, "normal timed jog regressed to text G-code")
    assert_true(len(owner.binary_points) == len(segments), "timed binary segment count mismatch")
    assert_true(
        all((point.flags & FLAG_HOST_TIMED) != 0 for point in owner.binary_points),
        "timed binary points are missing the host-timed flag",
    )
    assert_true(
        [point.v_dom_pps for point in owner.binary_points] == [100, 80],
        "timed binary duration ticks were not encoded",
    )

    experimental = TimedUploadDummy(text_mode=True)
    assert_true(experimental._upload_host_segments(segments, "jog"), "experimental text upload failed")
    assert_true(experimental.text_segments == segments, "explicit host-timed text mode was not honored")
    assert_true(experimental.binary_points is None, "experimental text mode also started binary upload")


def check_timed_path_segmentation():
    owner = TimedPathDummy()
    sx, sy = owner.cur_x, owner.cur_y
    path = owner.generate_linear_path(sx, sy, sx + 20.0, sy, 20.0)
    segments = owner.build_host_segments_from_path(path, start_xy=(sx, sy), label="test")
    assert_true(segments, "timed path generated no segments")
    assert_true(len(segments) < 300, "timed path regressed to one binary point per motor pulse")
    assert_true(
        all((segment.flags & owner.BINARY_FLAG_HOST_SEGMENT) != 0 for segment in segments),
        "timed path contains a segment without the host-timed flag",
    )
    assert_true((segments[-1].flags & owner.BINARY_FLAG_EXACT_STOP) != 0, "timed path does not stop at final point")

    expected_time = 0.0
    previous_xy = (sx, sy)
    previous_speed = 0.0
    for point in path:
        speed = max(0.0, float(point[2]) / 60.0)
        distance = math.hypot(float(point[0]) - previous_xy[0], float(point[1]) - previous_xy[1])
        average_speed = 0.5 * (previous_speed + speed)
        if distance > 1e-6:
            expected_time += distance / max(0.1, average_speed)
        previous_xy = (float(point[0]), float(point[1]))
        previous_speed = speed
    actual_time = sum(int(segment.duration_ticks) for segment in segments) / owner.HOST_DDA_HZ
    assert_true(
        abs(actual_time - expected_time) <= 0.02,
        f"timed path duration drifted: actual={actual_time:.6f}s expected={expected_time:.6f}s",
    )

    q1, q2 = owner.inverse_kinematics(float(path[-1][0]), float(path[-1][1]))
    expected_target = owner._joint_deg_to_pulse(q1, q2)
    actual_target = (int(segments[-1].p1_abs), int(segments[-1].p2_abs))
    assert_true(actual_target == expected_target, "timed path final pulse target drifted")

    last = owner._joint_deg_to_pulse(*owner.inverse_kinematics(sx, sy))
    speeds = []
    for segment in segments:
        dominant = max(abs(int(segment.p1_abs) - last[0]), abs(int(segment.p2_abs) - last[1]))
        assert_true(dominant <= int(segment.duration_ticks), "timed path requests more than one DDA event per tick")
        speeds.append(dominant * owner.HOST_DDA_HZ / int(segment.duration_ticks))
        last = (int(segment.p1_abs), int(segment.p2_abs))
    assert_true(
        max(abs(current - previous) for previous, current in zip(speeds, speeds[1:])) <= 250.0,
        "timed path contains a large adjacent slice speed jump",
    )


def segment_line_error(owner, start_pulses, segments, line_start, line_end):
    max_error = 0.0
    previous = tuple(int(value) for value in start_pulses)
    for segment in segments:
        target = (int(segment.p1_abs), int(segment.p2_abs))
        dp1 = target[0] - previous[0]
        dp2 = target[1] - previous[1]
        events = max(abs(dp1), abs(dp2), 1)
        for event in range(1, events + 1):
            pulses = (
                int(round(previous[0] + dp1 * event / events)),
                int(round(previous[1] + dp2 * event / events)),
            )
            xy = owner._binary_xy_from_pulse(pulses[0], pulses[1], owner.current_ppr)
            max_error = max(max_error, owner._point_to_line_error(xy, line_start, line_end))
        previous = target
    return max_error


def check_cartesian_jog_roundtrip():
    owner = TimedPathDummy()
    points = ((75.0, 220.0), (90.0, 145.0), (50.0, 170.0), (75.0, 135.0))
    deltas = ((-10.0, 0.0), (10.0, 0.0), (0.0, -10.0), (0.0, 10.0))
    max_line_error = 0.0

    for point in points:
        origin = owner._binary_pulse_from_xy(point, owner.current_ppr)
        assert_true(origin is not None, f"test point has no pulse solution: {point}")
        for delta in deltas:
            owner.feedback_p1, owner.feedback_p2 = origin
            owner._reset_jog_anchor()
            start_xy, start_pulses = owner._ensure_jog_anchor()
            target_xy = (start_xy[0] + delta[0], start_xy[1] + delta[1])
            assert_true(owner.check_workspace_safety(*target_xy), f"jog target is unsafe: {target_xy}")

            outbound = owner._build_cartesian_host_segments(
                start_xy,
                target_xy,
                20.0,
                100.0,
                start_pulses=start_pulses,
            )
            assert_true(outbound, f"outbound jog generated no segments: point={point} delta={delta}")
            target_pulses = (int(outbound[-1].p1_abs), int(outbound[-1].p2_abs))
            max_line_error = max(
                max_line_error,
                segment_line_error(owner, start_pulses, outbound, start_xy, target_xy),
            )

            owner._commit_jog_target(target_xy, target_pulses, start_pulses)
            owner.feedback_p1, owner.feedback_p2 = target_pulses
            owner._check_jog_completion_from_feedback()
            assert_true(owner.jog_pending_target_pulses is None, "jog completion did not clear pending target")
            anchored_xy, anchored_pulses = owner._ensure_jog_anchor()
            assert_true(anchored_xy == target_xy, "feedback refresh replaced the commanded jog anchor")
            assert_true(anchored_pulses == target_pulses, "feedback refresh replaced the target pulses")

            return_xy = (anchored_xy[0] - delta[0], anchored_xy[1] - delta[1])
            inbound = owner._build_cartesian_host_segments(
                anchored_xy,
                return_xy,
                20.0,
                100.0,
                start_pulses=anchored_pulses,
            )
            assert_true(inbound, f"return jog generated no segments: point={point} delta={delta}")
            return_pulses = (int(inbound[-1].p1_abs), int(inbound[-1].p2_abs))
            max_line_error = max(
                max_line_error,
                segment_line_error(owner, anchored_pulses, inbound, anchored_xy, return_xy),
            )
            owner._commit_jog_target(return_xy, return_pulses, anchored_pulses)
            owner.feedback_p1, owner.feedback_p2 = return_pulses
            owner._check_jog_completion_from_feedback()
            assert_true(
                return_pulses == origin,
                f"jog roundtrip did not close: point={point} delta={delta} "
                f"origin={origin} return={return_pulses}",
            )
            assert_true(
                any("JOG_ROUNDTRIP_CLOSED" in item for item in owner.log_display.items),
                "jog roundtrip closure was not logged",
            )

    # At the lower workspace edge, a single 3200-PPR pulse can move the tool
    # more than 0.3 mm. Keep the current configuration bounded, then verify
    # that the requested 0.3 mm tolerance is achievable at a supported PPR.
    assert_true(
        max_line_error <= 0.8,
        f"Cartesian jog cross-track error exceeds the 3200-PPR quantization bound: {max_line_error:.6f}mm",
    )

    owner = TimedPathDummy()
    owner.current_ppr = 12800
    high_resolution_error = 0.0
    for point in points:
        origin = owner._binary_pulse_from_xy(point, owner.current_ppr)
        owner.feedback_p1, owner.feedback_p2 = origin
        owner._reset_jog_anchor()
        start_xy, start_pulses = owner._ensure_jog_anchor()
        for delta in deltas:
            target_xy = (start_xy[0] + delta[0], start_xy[1] + delta[1])
            segments = owner._build_cartesian_host_segments(
                start_xy,
                target_xy,
                20.0,
                100.0,
                start_pulses=start_pulses,
            )
            high_resolution_error = max(
                high_resolution_error,
                segment_line_error(owner, start_pulses, segments, start_xy, target_xy),
            )
    assert_true(
        high_resolution_error <= 0.3,
        f"Cartesian jog cross-track error exceeds 0.3mm at 12800 PPR: {high_resolution_error:.6f}mm",
    )


def check_feedback_mode_does_not_change_motion_state():
    feedback_pulses = (97, -97)
    states = []
    for mode in ("通讯发送内容", "通讯接收内容"):
        owner = FeedbackStateDummy(mode)
        owner.ser.rx_lines.append(
            f"<P:{feedback_pulses[0]},{feedback_pulses[1]}|M:999.000,999.000>\n"
        )
        owner.check_serial_feedback()
        expected = owner._feedback_xy_from_pulses(feedback_pulses)
        assert_true(
            math.hypot(owner.cur_x - expected[0], owner.cur_y - expected[1]) < 1e-9,
            f"plot mode {mode} allowed M: display data to override P: motion state",
        )
        assert_true(owner.plot_updates == 1, f"plot mode {mode} did not refresh the plot")
        states.append((owner.cur_x, owner.cur_y))
    assert_true(states[0] == states[1], "plot mode changed the authoritative motion state")


def check_single_rounding_pulse_conversion():
    ppr = 32000
    zero_deg = 2.251 * 180.0 / 3.141592653589793
    one_pulse_deg = 360.0 / ppr
    p0, _ = joint_deg_to_pulse(zero_deg, zero_deg, ppr)
    p1, _ = joint_deg_to_pulse(zero_deg + one_pulse_deg, zero_deg, ppr)
    previous = p0
    jumps = []
    for index in range(1, 40):
        current, _ = joint_deg_to_pulse(zero_deg + index * one_pulse_deg, zero_deg, ppr)
        jumps.append(current - previous)
        previous = current
    full_rev, _ = joint_deg_to_pulse(zero_deg + 360.0, zero_deg, ppr)

    assert_true(p0 == 0 and p1 == 1, "joint conversion cannot resolve one-pulse angle increments")
    assert_true(all(jump == 1 for jump in jumps), "joint conversion still contains multi-pulse mrad quantization")
    assert_true(full_rev == ppr, "joint conversion uses an approximate revolution constant")


def check_stop_clears_all_sender_layers():
    owner = SenderWindowDummy()
    owner.process_queue()
    owner.binary_motion_active = True
    owner.binary_stream_active = True
    owner.binary_stream_points = [1, 2, 3]
    owner.active_binary_send_path = [(1, 2)]
    owner.active_preview_path = [(1, 2)]
    owner.stream_waiting_buffer = True
    owner.emergency_resume_path = [(1, 2)]
    owner.emergency_paused = False
    ScaraMotionMixin.stop_motion(owner)
    assert_true(not owner.point_queue, "stop did not clear host pending queue")
    assert_true(not owner.inflight_lines, "stop did not clear inflight ASCII window")
    assert_true(not owner.binary_stream_active, "stop did not stop binary refill timer state")
    assert_true(not owner.binary_motion_active, "stop did not clear binary active state")
    assert_true(not owner.active_binary_send_path, "stop did not release active binary path")
    assert_true(owner.ser.writes[-1] == b"STOP\n", "stop did not send the controller buffer-clear command")


def check_capability_handshake():
    owner = SenderWindowDummy()
    owner.ser.rx_lines.append(
        "OK HOSTCAP host_plan=1 gcode_abt=1 binary_traj=1 binary_timed=1 dda=1 hz=10000\n"
    )
    capabilities = owner._query_host_capabilities()
    assert_true(capabilities.get("binary_traj") == "1", "HOSTCAP missed binary trajectory support")
    assert_true(capabilities.get("binary_timed") == "1", "HOSTCAP missed timed binary support")
    assert_true(owner.ser.writes[-1] == b"HOSTCAP\n", "HOSTCAP handshake was not sent")


def check_binary_ack_statistics():
    owner = SenderWindowDummy()
    owner._pump_ui_events = lambda: None
    owner.ser.rx_bytes.extend(build_frame(TYPE_ACK, 7, bytes((TYPE_BEGIN, 0))))
    frame = owner._send_binary_frame_expect_ack(TYPE_BEGIN, 7, b"")
    assert_true(frame.frame_type == TYPE_ACK, "binary ACK frame was not parsed")
    assert_true(getattr(owner, "avg_ack_ms", None) is not None, "binary ACK latency was not recorded")
    assert_true(getattr(owner, "sender_mode", "") == "binary_buffered", "binary ACK did not update sender mode")


def main():
    check_selection()
    check_window_accounting()
    check_ack_window_progress()
    check_timed_jog_transport()
    check_timed_path_segmentation()
    check_cartesian_jog_roundtrip()
    check_feedback_mode_does_not_change_motion_state()
    check_single_rounding_pulse_conversion()
    check_stop_clears_all_sender_layers()
    check_capability_handshake()
    check_binary_ack_statistics()
    print("SENDER_STRATEGY_CHECK PASS")


if __name__ == "__main__":
    main()
