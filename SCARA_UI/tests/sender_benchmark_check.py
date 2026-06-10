from pathlib import Path
import math
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from SCARA_UI.communication.motion_senders import BUFFERED_BINARY_SENDER, select_motion_sender
from SCARA_UI.tests.trajectory_planner_check import DummyUi


class OpenSerial:
    is_open = True


class SelectorOwner:
    def __init__(self):
        self.ser = OpenSerial()
        self.host_timed_segment_mode = False
        self.waiting_for_ack = False
        self.point_queue = []


def build_cases(ui):
    start = (75.0, 220.0)
    jog_preview = ui.generate_linear_path(*start, 75.0, 230.0, 20.0)
    jog_send = ui._build_cartesian_host_segments(start, (75.0, 230.0), 20.0, 100.0)

    line_preview = ui.generate_linear_path(*start, 175.0, 220.0, 20.0)
    line_send = ui.generate_binary_line_targets(start, (175.0, 220.0), 20.0)

    car_preview, car_send = ui.generate_geometry_motion(ui.build_car1_segments(75.0, 200.0), 20.0, label="car1")

    strokes = ui.handwriting_strokes_to_robot([[(0.1, 0.8), (0.3, 0.2), (0.7, 0.6)]])
    handwriting_preview = ui.generate_stroke_path(strokes, 20.0, label="handwriting")
    handwriting_send = ui.generate_binary_send_from_path(handwriting_preview, 20.0)

    return (
        ("jog_10mm", jog_preview, jog_send),
        ("line_100mm", line_preview, line_send),
        ("car_outline", car_preview, car_send),
        ("handwriting", handwriting_preview, handwriting_send),
    )


def main():
    owner = SelectorOwner()
    if select_motion_sender(owner, append=False, send_path=[(1, 2)]) is not BUFFERED_BINARY_SENDER:
        raise AssertionError("ordinary benchmark path does not select buffered binary")

    ui = DummyUi()
    print("case,preview_points,binary_points,binary_chunks,ascii_stop_wait_lines,reduction")
    for name, preview, send_path in build_cases(ui):
        if not preview or not send_path:
            raise AssertionError(f"{name} generated an empty path")
        binary_chunks = math.ceil(len(send_path) / 20.0)
        ascii_lines = len(preview)
        reduction = 1.0 - (binary_chunks / max(1, ascii_lines))
        if binary_chunks > ascii_lines:
            raise AssertionError(f"{name} binary chunk count exceeds ASCII line count")
        print(f"{name},{ascii_lines},{len(send_path)},{binary_chunks},{ascii_lines},{reduction:.3f}")
    print("SENDER_BENCHMARK_CHECK PASS")


if __name__ == "__main__":
    main()
