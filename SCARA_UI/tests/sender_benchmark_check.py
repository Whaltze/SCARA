from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from SCARA_UI.communication.motion_senders import GcodeJob


def main():
    generated = 0

    def commands():
        nonlocal generated
        for index in range(100000):
            generated += 1
            yield f"G1 X{index % 100} Y{(index * 3) % 100} F1200"

    job = GcodeJob(commands(), max_pending=64)
    peak = len(job)
    consumed = 0
    while job:
        job.pop(0)
        consumed += 1
        peak = max(peak, len(job))
    assert consumed == 100000
    assert generated == 100000
    assert peak <= 64
    print(f"SENDER_BENCHMARK_CHECK PASS commands={consumed} peak_pending={peak}")


if __name__ == "__main__":
    main()
