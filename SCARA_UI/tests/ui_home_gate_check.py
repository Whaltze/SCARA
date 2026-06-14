"""UI-level homing gate regression checks.

The real window is instantiated offscreen and driven through the same button
slots a user clicks. A fake serial port captures writes so this test can prove
that homing status unlocks jog/trajectory transmission without moving hardware.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from collections import deque


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


class FakeSerial:
    def __init__(self):
        self.is_open = True
        self.writes = []
        self._rx = deque()

    @property
    def in_waiting(self):
        return len(self._rx[0]) if self._rx else 0

    def queue_line(self, line: str):
        self._rx.append((line.rstrip("\n") + "\n").encode("ascii"))

    def readline(self):
        if not self._rx:
            return b""
        return self._rx.popleft()

    def write(self, data):
        if isinstance(data, str):
            data = data.encode("ascii")
        self.writes.append(bytes(data))
        return len(data)

    def flush(self):
        return None

    def close(self):
        self.is_open = False

    def clear_writes(self):
        self.writes.clear()

    def text(self):
        return "".join(item.decode("ascii", errors="ignore") for item in self.writes)


def stop_threads(window) -> None:
    for name in ("cam_thread", "img_proc_thread"):
        thread = getattr(window, name, None)
        if thread is None:
            continue
        if hasattr(thread, "running"):
            thread.running = False
        if hasattr(thread, "quit"):
            thread.quit()
        if hasattr(thread, "wait"):
            thread.wait(1000)


def pump(app, window, cycles: int = 1):
    for _ in range(cycles):
        app.processEvents()
        window.check_serial_feedback()
        app.processEvents()


def assert_no_motion_write(serial: FakeSerial):
    text = serial.text()
    for marker in ("$J=", "G0 ", "G1 ", "G2 ", "G3 "):
        assert marker not in text, text


def reset_sender(window, serial: FakeSerial):
    if hasattr(window, "_clear_text_sender_state"):
        window._clear_text_sender_state()
    window.point_queue = []
    window.waiting_for_ack = False
    window.stream_waiting_buffer = False
    window.last_sent_motion = None
    window.motion_preamble_needed = False
    window.controller_reset_pending = False
    window.controller_reset_reason = ""
    window.controller_reset_started_at = 0.0
    window.microstep_dirty = False
    window.cur_x = 75.0
    window.cur_y = 220.0
    window.feedback_p1 = 0
    window.feedback_p2 = 0
    serial.clear_writes()


def queue_status(serial: FakeSerial, state: str = "Run", pf=None, rl=0, pg=0):
    diagnostics = "" if pf is None else f"|Pf:{pf}|Rl:{rl}|Pg:{pg}"
    serial.queue_line(
        f"<{state}|MPos:0.000,220.000|JPos:0,0|FS:1200,0|Bf:48,256|Q:0|E:0|"
        f"Seg:0,16,0,0{diagnostics}|H:0,0|HS:Done|A1:1,0,0,0|A2:1,0,0,0|Lz:0,0,0,10>"
    )


def drain_fake_controller(app, window, serial: FakeSerial, label: str):
    """Ack every sent line, with status frames interleaved like the controller."""
    for _ in range(1000):
        while serial.in_waiting:
            pump(app, window)

        inflight = getattr(window, "inflight_lines", []) or []
        queued = getattr(window, "point_queue", []) or []
        if not inflight and not queued and not getattr(window, "waiting_for_ack", False):
            queue_status(serial, "Idle")
            pump(app, window)
            return

        if inflight:
            queue_status(serial, "Run")
            serial.queue_line("ok")
            pump(app, window, cycles=2)
            continue

        if queued:
            window.process_queue()
            pump(app, window)
            continue

    raise AssertionError(
        f"{label} did not drain; queued={len(getattr(window, 'point_queue', []) or [])} "
        f"inflight={len(getattr(window, 'inflight_lines', []) or [])} "
        f"waiting={getattr(window, 'waiting_for_ack', None)} writes={serial.text()!r}"
    )


def assert_click_roundtrip(app, window, serial: FakeSerial, click, expected: str, label: str):
    reset_sender(window, serial)
    click()
    pump(app, window)
    text = serial.text()
    assert expected in text, f"{label} did not write {expected!r}; writes={text!r}"
    drain_fake_controller(app, window, serial, label)


def set_combo_prefix(combo, prefix: str):
    for index in range(combo.count()):
        if combo.itemText(index).startswith(prefix):
            combo.setCurrentIndex(index)
            return
    raise AssertionError(f"combo item starting with {prefix!r} not found")


def main() -> int:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from SCARA_UI.ui.main_window import FiveBarSerialGUI

    app = QApplication.instance() or QApplication([])
    window = FiveBarSerialGUI()
    serial = FakeSerial()
    window.ser = serial

    try:
        window.board_only_debug = False
        window.cur_x = 75.0
        window.cur_y = 220.0
        window.feedback_p1 = 0
        window.feedback_p2 = 0
        window.current_ppr = 6400
        window.motion_preamble_needed = False
        window.microstep_dirty = False
        window.hw_speed_input.setText("20")
        window.jog_step_input.setText("10")
        window.plot_mode_combo.setCurrentText("通讯接收内容")

        reset_sender(window, serial)
        window.planner_fault_count = None
        queue_status(serial, "Idle", pf=4, rl=3, pg=2)
        pump(app, window)
        assert window.planner_fault_count == 4
        assert window.rate_limited_segment_count == 3
        assert b"\x18" not in serial.writes
        queue_status(serial, "Idle", pf=5, rl=4, pg=3)
        pump(app, window)
        assert window.planner_fault_count == 5
        assert b"\x18" not in serial.writes

        reset_sender(window, serial)
        window.inflight_lines = [
            {
                "line": "G1 X1.000 Y2.000 F1200",
                "bytes": 24,
                "sent_at": time.time(),
                "mode": "gcode_stream",
                "id": 1,
                "motion": "G1 X1.000 Y2.000 F1200",
            }
        ]
        window.inflight_bytes = 24
        window.waiting_for_ack = True
        queue_status(serial, "Run", pf=6, rl=4, pg=3)
        pump(app, window)
        assert b"\x18" in serial.writes
        assert window.controller_reset_pending is True
        queue_status(serial, "Idle", pf=6, rl=4, pg=3)
        pump(app, window)
        assert window.controller_reset_pending is False

        window.is_homed = False
        window.btns["UP"].click()
        pump(app, window)
        assert_no_motion_write(serial)

        reset_sender(window, serial)
        window.is_homed = False
        window.btn_home_real.click()
        pump(app, window)
        assert "$X\n" in serial.text()
        assert "M17\n" in serial.text()
        assert "$H\n" not in serial.text()
        assert [entry["line"] for entry in window.inflight_lines] == ["$X", "M17"]
        serial.queue_line("ok")
        pump(app, window, cycles=2)
        assert "$H\n" not in serial.text()
        assert [entry["line"] for entry in window.inflight_lines] == ["M17"]
        serial.queue_line("ok")
        pump(app, window, cycles=2)
        assert "$H\n" in serial.text()
        assert [entry["line"] for entry in window.inflight_lines] == ["$H"]
        assert window.is_homed is False
        home_writes_before_timeout = len(serial.writes)
        window.handle_timeout()
        assert serial.writes[-1] == b"?"
        assert len(serial.writes) == home_writes_before_timeout + 1
        assert [entry["line"] for entry in window.inflight_lines] == ["$H"]
        serial.queue_line("error:15")
        pump(app, window)
        assert [entry["line"] for entry in window.inflight_lines] == ["$H"]
        assert b"\x18" not in serial.writes
        queue_status(serial, "Idle")
        pump(app, window)
        assert window.is_homed is True
        serial.queue_line("ok")
        pump(app, window)
        assert not window.inflight_lines
        assert not window.waiting_for_ack

        serial.queue_line(
            "STAT t=1 m=IDLE e=0 p=0,0 r=0,0 en=1,1 pps=0,0 tgt=0,0 "
            "wd=0 idle=0 rxov=0 txd=0 txq=0 h=0,0 hs=Done he=0 "
            "bf=48,256 q=0 sq=0,16 low=0 und=0 prep=0 done=0 hz=10000 ic=100"
        )
        pump(app, window)
        assert window.is_homed is True

        serial.clear_writes()
        window.btns["UP"].click()
        pump(app, window)
        assert "$J=G91 X0.000 Y10.000" in serial.text(), serial.text()

        reset_sender(window, serial)
        window.is_homed = True
        accepted = window.load_motion_gcode_job(
            ["G1 X925.000 Y1000.000 F1200"],
            preview_path=[(1000.0, 1000.0, 1200.0, False)],
        )
        assert accepted is False
        assert_no_motion_write(serial)
        assert not window.waiting_for_ack
        assert not window.point_queue

        reset_sender(window, serial)
        window.is_homed = True
        window.last_controller_state = "run"
        accepted = window.load_motion_gcode_job(
            ["G1 X0.000 Y220.000 F1200"],
            preview_path=[(75.0, 220.0, 1200.0, False)],
        )
        assert accepted is False
        assert_no_motion_write(serial)
        window.last_controller_state = "idle"

        reset_sender(window, serial)
        window.is_homed = False
        serial.queue_line("ok")
        pump(app, window)
        assert window.is_homed is False
        serial.queue_line(
            "<Idle|MPos:0.000,220.000|JPos:0,0|FS:1200,0|Bf:48,256|Q:0|E:0|"
            "Seg:0,16,0,0|H:0,0|HS:Done|A1:1,0,0,0|A2:1,0,0,0|Lz:0,0,0,10>"
        )
        pump(app, window)
        assert window.is_homed is True

        serial.clear_writes()
        window.btn_stop_motion.click()
        assert b"\x18" in serial.writes
        assert b"?" in serial.writes
        assert window.controller_reset_pending is True
        accepted = window.load_motion_gcode_job(
            ["G1 X0.000 Y220.000 F1200"],
            preview_path=[(75.0, 220.0, 1200.0, False)],
        )
        assert accepted is False
        serial.queue_line("error:15")
        pump(app, window)
        assert window.controller_reset_pending is True
        queue_status(serial, "Idle")
        pump(app, window)
        assert window.controller_reset_pending is False

        jog_cases = [
            ("UP", "$J=G91 X0.000 Y10.000"),
            ("DOWN", "$J=G91 X0.000 Y-10.000"),
            ("LEFT", "$J=G91 X-10.000 Y0.000"),
            ("RIGHT", "$J=G91 X10.000 Y0.000"),
        ]
        for key, expected in jog_cases:
            assert_click_roundtrip(
                app,
                window,
                serial,
                lambda key=key: window.btns[key].click(),
                expected,
                f"jog {key}",
            )

        motor_cases = [
            ("M1_POS", "G0 "),
            ("M1_NEG", "G0 "),
            ("M2_POS", "G0 "),
            ("M2_NEG", "G0 "),
        ]
        for key, expected in motor_cases:
            assert_click_roundtrip(
                app,
                window,
                serial,
                lambda key=key: window.motor_btns[key].click(),
                expected,
                f"motor {key}",
            )

        window.target_x.setText("85")
        window.target_y.setText("225")
        set_combo_prefix(window.mode_combo, "G1")
        assert_click_roundtrip(app, window, serial, lambda: window.plan_trajectory(silent=False), "G1 ", "G1 trajectory")

        window.target_x.setText("90")
        window.target_y.setText("230")
        window.radius_r.setText("30")
        set_combo_prefix(window.mode_combo, "G2")
        assert_click_roundtrip(app, window, serial, lambda: window.plan_trajectory(silent=False), "G2 ", "G2 trajectory")

        window.target_x.setText("80")
        window.target_y.setText("235")
        window.radius_r.setText("35")
        set_combo_prefix(window.mode_combo, "G3")
        assert_click_roundtrip(app, window, serial, lambda: window.plan_trajectory(silent=False), "G3 ", "G3 trajectory")

        window.car_start_x.setText("75")
        window.car_start_y.setText("200")
        assert_click_roundtrip(app, window, serial, window.plan_car_path, "G0 ", "car1 trajectory")
        assert_click_roundtrip(app, window, serial, window.plan_car2_path, "G0 ", "car2 trajectory")

        reset_sender(window, serial)
        window.hw_speed_input.setText("12.5")
        window.hw_accel_input.setText("75")
        window.handwriting_pad.normalized_strokes = lambda: [
            [(0.45, 0.50), (0.50, 0.50), (0.50, 0.45)],
            [(0.55, 0.45), (0.60, 0.45)],
        ]
        window.plan_handwriting_path()
        drain_fake_controller(app, window, serial, "handwriting geometry")
        writing = serial.text()
        assert "$110=750\n" in writing
        assert "$120=75\n" in writing
        assert "$11=20\n" in writing
        assert "G4 P0.001\n" in writing
        assert "G0 " in writing
        writing_geometry = [
            line for line in writing.splitlines() if line.startswith(("G1 ", "G2 ", "G3 "))
        ]
        assert writing_geometry
        assert all("F750" in line for line in writing_geometry), writing_geometry

        reset_sender(window, serial)
        window.is_homed = True
        window.inflight_lines = [
            {
                "line": "G1 X1.000 Y2.000 F1200",
                "bytes": 24,
                "sent_at": time.time() - 10.0,
                "mode": "gcode_stream",
                "id": 1,
                "motion": "G1 X1.000 Y2.000 F1200",
            }
        ]
        window.inflight_bytes = 24
        window.waiting_for_ack = True
        window.ack_timeout_count = 20
        window.idle_ack_stall_polls = 0
        window.last_controller_rx_at = time.time()
        window.last_controller_state = "run"
        window.mcu_planner_free = 0
        window.last_segment_count = 16
        serial.clear_writes()
        window.handle_timeout()
        assert b"?" in serial.writes
        assert b"\x18" not in serial.writes
        assert window.inflight_lines
        assert window.waiting_for_ack

        window.ack_timeout_count = 1
        window.idle_ack_stall_polls = 1
        window.last_controller_rx_at = time.time()
        window.last_controller_state = "idle"
        window.mcu_planner_free = window.mcu_planner_capacity
        window.last_segment_count = 0
        serial.clear_writes()
        window.handle_timeout()
        assert b"?" in serial.writes
        assert b"\x18" in serial.writes
        assert not window.inflight_lines
        assert not window.waiting_for_ack

        print("UI_HOME_GATE_CHECK PASS")
        return 0
    finally:
        stop_threads(window)
        window.close()
        app.processEvents()


if __name__ == "__main__":
    raise SystemExit(main())
