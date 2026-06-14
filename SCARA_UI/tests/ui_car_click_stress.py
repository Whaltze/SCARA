"""Run the car trajectory through the real Qt UI control path.

This test intentionally instantiates the main window, fills the same widgets a
user edits, clicks the same car button slots, and lets ``serial_mixin`` stream
the generated GRBL/G-code job. It is a UI-level stress test, not a hand-written
protocol sender.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SCARA UI car click stress")
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--shape", choices=("car1", "car2"), default="car1")
    parser.add_argument("--feed-mm-s", type=float, default=20.0)
    parser.add_argument("--car-x", type=float, default=75.0)
    parser.add_argument("--car-y", type=float, default=200.0)
    parser.add_argument("--max-error-mm", type=float, default=0.6)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--csv-path", default="")
    parser.add_argument("--use-zero", action="store_true", help="Use UI serial ZERO prep instead of clicking simulated homing.")
    parser.add_argument("--show", action="store_true", help="Show the window instead of running Qt offscreen.")
    return parser.parse_args()


def pump(app, window, seconds: float, poll_status: bool = False) -> None:
    deadline = time.monotonic() + max(0.0, seconds)
    next_status = 0.0
    while time.monotonic() < deadline:
        app.processEvents()
        if poll_status and window.ser and window.ser.is_open and time.monotonic() >= next_status:
            try:
                window.ser.write(b"?")
            except Exception:
                pass
            next_status = time.monotonic() + 0.12
        time.sleep(0.01)


def wait_for_serial(window, app, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents()
        if window.ser and window.ser.is_open:
            return
        time.sleep(0.02)
    raise TimeoutError("UI did not open serial port")


def wait_for_motion_done(window, app, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    stable_idle_since = None
    while time.monotonic() < deadline:
        pump(app, window, 0.08, poll_status=True)
        stats = window.feedback_error_tracker.stats()
        queued = bool(getattr(window, "point_queue", []))
        waiting = bool(getattr(window, "waiting_for_ack", False))
        if stats.count >= 3 and not queued and not waiting:
            if stable_idle_since is None:
                stable_idle_since = time.monotonic()
            elif time.monotonic() - stable_idle_since >= 1.0:
                return
        else:
            stable_idle_since = None
    raise TimeoutError("UI car motion did not finish before timeout")


def export_csv(path: str, samples) -> None:
    if not path:
        return
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(("index", "feedback_x", "feedback_y", "expected_x", "expected_y", "err_x", "err_y", "err_norm", "segment"))
        for index, sample in enumerate(samples, start=1):
            writer.writerow(
                (
                    index,
                    f"{sample.feedback_x:.6f}",
                    f"{sample.feedback_y:.6f}",
                    f"{sample.expected_x:.6f}",
                    f"{sample.expected_y:.6f}",
                    f"{sample.err_x:.6f}",
                    f"{sample.err_y:.6f}",
                    f"{sample.err_norm:.6f}",
                    sample.segment_index,
                )
            )


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


def main() -> int:
    args = parse_args()
    if not args.show:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from SCARA_UI.ui.main_window import FiveBarSerialGUI

    app = QApplication.instance() or QApplication([])
    window = FiveBarSerialGUI()
    if args.show:
        window.show()

    try:
        if window.port_combo.findText(args.port) < 0:
            window.port_combo.addItem(args.port)
        window.port_combo.setCurrentText(args.port)
        window.hw_speed_input.setText(f"{args.feed_mm_s:.6f}")
        window.car_start_x.setText(f"{args.car_x:.6f}")
        window.car_start_y.setText(f"{args.car_y:.6f}")
        window.plot_mode_combo.setCurrentIndex(1)

        window.btn_connect.click()
        wait_for_serial(window, app, 5.0)
        pump(app, window, 0.4)

        if args.use_zero:
            window.send_ascii_line("STOP", "UI_TEST_PREP")
            window.send_ascii_line("CLEAR_ERROR", "UI_TEST_PREP")
            window.send_ascii_line("ZERO", "UI_TEST_PREP")
            window.send_ascii_line("ENABLE 1", "UI_TEST_PREP")
        else:
            window.btn_reset_home.click()
        pump(app, window, 1.2, poll_status=True)

        before_log = window.log_display.toPlainText()
        if args.shape == "car1":
            window.plan_car_path()
        else:
            window.plan_car2_path()
        pump(app, window, 0.2)

        preview_count = len(getattr(window, "active_preview_path", []) or [])
        send_count = int(getattr(window, "sent_point_id", 0))
        if preview_count == 0:
            preview_count = len(getattr(window, "preview_x", []) or [])

        wait_for_motion_done(window, app, args.timeout_s)
        stats = window.feedback_error_tracker.stats()
        export_csv(args.csv_path, window.feedback_error_tracker.samples)

        print(
            "UI_CAR_CLICK shape={} preview={} send={} samples={} max={:.4f} rms={:.4f}".format(
                args.shape,
                preview_count,
                send_count,
                stats.count,
                stats.max_norm,
                stats.rms_norm,
            )
        )
        if args.csv_path:
            print(f"CSV {args.csv_path}")
        if stats.count < 3:
            print("UI_CAR_CLICK FAIL: too few feedback samples")
            return 1
        if stats.max_norm > args.max_error_mm:
            print(f"UI_CAR_CLICK FAIL: max error {stats.max_norm:.4f}mm > {args.max_error_mm:.4f}mm")
            return 1
        if "控制器报警" in window.log_display.toPlainText()[len(before_log) :]:
            print("UI_CAR_CLICK FAIL: controller error logged")
            return 1
        print("UI_CAR_CLICK PASS")
        return 0
    finally:
        try:
            if window.ser and window.ser.is_open:
                window.send_ascii_line("ENABLE 0", "UI_TEST_DONE")
                pump(app, window, 0.2)
                window.ser.close()
        finally:
            stop_threads(window)
            window.close()
            app.processEvents()


if __name__ == "__main__":
    raise SystemExit(main())
