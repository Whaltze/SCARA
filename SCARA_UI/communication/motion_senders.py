"""GRBL character-counting motion sender.

The controller owns look-ahead and segment preparation. The UI only streams
real Cartesian G-code and keeps at most the configured send window in flight.
"""

from collections import deque
from itertools import chain


class CountedCommandStream:
    """One-shot lazy command stream with a known total for UI progress."""

    def __init__(self, commands, count):
        self._commands = iter(commands)
        self._count = max(0, int(count))

    def __iter__(self):
        return self

    def __next__(self):
        return next(self._commands)

    def __len__(self):
        return self._count


class GcodeJob:
    """Bounded lazy command source used by the GRBL sender."""

    def __init__(self, commands=(), max_pending=64):
        self._sources = deque([iter(commands)])
        self._pending = deque()
        self._max_pending = max(1, int(max_pending))
        self._fill()

    def _fill(self):
        while len(self._pending) < self._max_pending and self._sources:
            try:
                self._pending.append(next(self._sources[0]))
            except StopIteration:
                self._sources.popleft()

    def __bool__(self):
        self._fill()
        return bool(self._pending)

    def __len__(self):
        self._fill()
        return len(self._pending)

    def __getitem__(self, index):
        self._fill()
        if index == 0:
            return self._pending[0]
        if index == -1:
            return self._pending[-1]
        raise IndexError("GcodeJob only supports its buffered head and tail")

    def pop(self, index=0):
        if index != 0:
            raise IndexError("GcodeJob only pops from the head")
        self._fill()
        item = self._pending.popleft()
        self._fill()
        return item

    def insert(self, index, item):
        if index != 0:
            raise IndexError("GcodeJob only inserts at the head")
        self._pending.appendleft(item)

    def extend(self, commands):
        self._sources.append(iter(commands))
        self._fill()

    def clear(self):
        self._sources.clear()
        self._pending.clear()


def _job_commands(owner, source, include_preamble):
    preamble = []
    epilogue = []
    if include_preamble and getattr(owner, "motion_preamble_needed", False):
        preamble.extend(("$X", "M17"))
        owner.motion_preamble_needed = False
    if include_preamble and getattr(owner, "motion_profile_sync_requested", False):
        preamble.extend(getattr(owner, "_motion_profile_preamble", lambda: ())())
        owner.motion_profile_sync_requested = False
    if include_preamble and getattr(owner, "laser_task_active", False):
        power = int(getattr(owner, "_laser_s_word", lambda: 200)())
        preamble.append(f"M4 S{power}")
        epilogue.append("M5")
        owner.laser_preamble_needed = False
    return chain(preamble, source, epilogue)


class GrblGcodeSender:
    mode = "grbl_stream"

    def send(self, owner, path, *, append=False, send_path=None):
        source = path
        if append and (owner.waiting_for_ack or owner.point_queue):
            include_preamble = bool(
                getattr(owner, "motion_preamble_needed", False)
                or getattr(owner, "laser_preamble_needed", False)
                or getattr(owner, "motion_profile_sync_requested", False)
            )
            owner.point_queue.extend(_job_commands(owner, source, include_preamble=include_preamble))
            try:
                owner.total_task_points += len(source)
            except TypeError:
                pass
        else:
            owner.sent_point_id = 0
            try:
                owner.total_task_points = len(source)
            except TypeError:
                owner.total_task_points = 0
            owner.task_start_time = owner._sender_now()
            owner._clear_text_sender_state()
            owner.point_queue = GcodeJob(_job_commands(owner, source, include_preamble=True))
        owner._set_sender_status(self.mode, queued_lines=len(owner.point_queue), inflight_lines=0)
        owner.process_queue()
        return True


GRBL_GCODE_SENDER = GrblGcodeSender()


def select_motion_sender(owner, *, append=False, send_path=None):
    return GRBL_GCODE_SENDER
