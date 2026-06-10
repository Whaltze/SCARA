from pathlib import Path
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

    def inverse_kinematics(self, x, y):
        return self.kinematics.inverse(x, y)

    def _joint_deg_to_pulse_float(self, q1, q2):
        p1 = ((q1 * 3.141592653589793 / 180.0 * 1000.0 - self.BINARY_ZERO_MRAD[0]) * self.current_ppr) / self.BINARY_MRAD_PER_REV
        p2 = ((q2 * 3.141592653589793 / 180.0 * 1000.0 - self.BINARY_ZERO_MRAD[1]) * self.current_ppr) / self.BINARY_MRAD_PER_REV
        return p1, p2


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
    assert_true((segments[-1].flags & owner.BINARY_FLAG_EXACT_STOP) != 0, "timed path does not stop at final point")


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
    check_stop_clears_all_sender_layers()
    check_capability_handshake()
    check_binary_ack_statistics()
    print("SENDER_STRATEGY_CHECK PASS")


if __name__ == "__main__":
    main()
