from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from SCARA_UI.communication.motion_senders import (
    CountedCommandStream,
    GcodeJob,
    GRBL_GCODE_SENDER,
    select_motion_sender,
)
from SCARA_UI.communication.serial_mixin import ScaraSerialMixin
from SCARA_UI.communication.serial_protocol import parse_ok_ack


def check_bounded_lazy_job():
    generated = []

    def commands():
        for index in range(100000):
            generated.append(index)
            yield f"G1 X{index}"

    job = GcodeJob(commands())
    assert len(job) == 64
    assert len(generated) == 64
    for _ in range(1000):
        job.pop(0)
    assert len(job) == 64
    assert len(generated) == 1064


def check_counted_stream_stays_lazy():
    generated = []

    def commands():
        for index in range(100000):
            generated.append(index)
            yield f"G1 X{index}"

    source = CountedCommandStream(commands(), 100000)
    assert len(source) == 100000
    assert not generated
    job = GcodeJob(source)
    assert len(job) == 64
    assert len(generated) == 64


def check_single_sender():
    assert select_motion_sender(object()) is GRBL_GCODE_SENDER


def check_standard_ack():
    assert parse_ok_ack("ok").matched
    assert not parse_ok_ack("OK STATUS").matched
    assert not parse_ok_ack("ok cs=00 line=G1 X1").matched


def check_formal_homing_job():
    class Owner:
        waiting_for_ack = False
        point_queue = []
        motion_preamble_needed = True
        laser_task_active = False
        active_preview_path = []

        def _sender_now(self):
            return 0.0

        def _clear_text_sender_state(self):
            self.waiting_for_ack = False

        def _set_sender_status(self, *_args, **_kwargs):
            return None

        def process_queue(self):
            return None

    owner = Owner()
    GRBL_GCODE_SENDER.send(owner, ["$H"])
    assert [owner.point_queue.pop(0) for _ in range(3)] == ["$X", "M17", "$H"]
    assert not owner.point_queue


def check_appended_motion_keeps_formal_preamble():
    class Owner:
        waiting_for_ack = False
        point_queue = []
        motion_preamble_needed = True
        laser_preamble_needed = False
        laser_task_active = False
        active_preview_path = []

        def _sender_now(self):
            return 0.0

        def _clear_text_sender_state(self):
            self.waiting_for_ack = False

        def _set_sender_status(self, *_args, **_kwargs):
            return None

        def process_queue(self):
            return None

        def _laser_s_word(self):
            return 250

    owner = Owner()
    GRBL_GCODE_SENDER.send(owner, ["$100=6400 $101=6400"])
    owner.waiting_for_ack = True
    owner.motion_preamble_needed = True
    owner.laser_preamble_needed = True
    owner.laser_task_active = True
    GRBL_GCODE_SENDER.send(owner, ["G1 X1 Y2 F1200"], append=True)
    lines = []
    while owner.point_queue:
        lines.append(owner.point_queue.pop(0))
    assert lines == [
        "$X",
        "M17",
        "$100=6400 $101=6400",
        "$X",
        "M17",
        "M4 S250",
        "G1 X1 Y2 F1200",
        "M5",
    ]


def check_motion_profile_precedes_laser_and_geometry():
    class Owner:
        waiting_for_ack = False
        point_queue = []
        motion_preamble_needed = True
        motion_profile_sync_requested = True
        laser_preamble_needed = True
        laser_task_active = True
        active_preview_path = []

        def _sender_now(self):
            return 0.0

        def _clear_text_sender_state(self):
            self.waiting_for_ack = False

        def _set_sender_status(self, *_args, **_kwargs):
            return None

        def process_queue(self):
            return None

        def _laser_s_word(self):
            return 250

        def _motion_profile_preamble(self):
            return ("$110=1200", "$120=80", "$11=20")

    owner = Owner()
    GRBL_GCODE_SENDER.send(owner, ["G1 X1 Y2 F1200"])
    lines = []
    while owner.point_queue:
        lines.append(owner.point_queue.pop(0))
    assert lines == [
        "$X",
        "M17",
        "$110=1200",
        "$120=80",
        "$11=20",
        "M4 S250",
        "G1 X1 Y2 F1200",
        "M5",
    ]


def check_motion_line_classification():
    requires_home = ScaraSerialMixin._line_requires_homing
    long_running = ScaraSerialMixin._line_is_long_running
    assert requires_home("$J=G91 X1 F300")
    assert requires_home("G1 X1 Y2")
    assert requires_home("G1X1Y2")
    assert requires_home("G03 X1 Y2 I0 J1")
    assert not requires_home("$H")
    assert not requires_home("G20")
    assert not requires_home("G21")
    assert not requires_home("G90")
    assert long_running("$H")
    assert long_running(" $hs ")
    assert not long_running("$X")
    assert not long_running("G1 X1")


def check_fixed_character_counting_window():
    owner = object.__new__(ScaraSerialMixin)
    owner.rx_free_hint = 1
    owner.planner_free_hint = 0
    assert owner._text_sender_limits() == (64, 224)


def check_segment_event_fit_converges():
    """A dense candidate is shortened, not rejected by a PPS-delta gate."""
    ticks = 50
    distance = 1.0
    events_per_mm = 250.0
    for _ in range(24):
        events = round(events_per_mm * distance)
        if events <= ticks:
            break
        distance *= 0.95 * ticks / events
    assert events <= ticks
    assert distance > 0.0


def check_stream_log_sampling():
    owner = object.__new__(ScaraSerialMixin)
    owner.total_task_points = 100000
    assert owner._should_log_stream_progress(1)
    assert owner._should_log_stream_progress(20)
    assert not owner._should_log_stream_progress(21)
    assert owner._should_log_stream_progress(100)
    assert owner._should_log_stream_progress(100000)


def main():
    check_bounded_lazy_job()
    check_counted_stream_stays_lazy()
    check_single_sender()
    check_standard_ack()
    check_formal_homing_job()
    check_appended_motion_keeps_formal_preamble()
    check_motion_profile_precedes_laser_and_geometry()
    check_motion_line_classification()
    check_fixed_character_counting_window()
    check_segment_event_fit_converges()
    check_stream_log_sampling()
    print("SENDER_STRATEGY_CHECK PASS")


if __name__ == "__main__":
    main()
