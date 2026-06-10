"""Motion sender strategies.

The UI planner produces paths. These strategies decide how an already-planned
path is transported without mixing transport-specific state into the dispatcher.
"""


class MotionSender:
    mode = "unknown"

    def send(self, owner, path, *, append=False, send_path=None):
        raise NotImplementedError


class HostTimedSegmentSender(MotionSender):
    mode = "host_timed_segment"

    def send(self, owner, path, *, append=False, send_path=None):
        if append or owner.waiting_for_ack or owner.point_queue:
            owner.log_error("Motion queue is busy; stop or wait before loading a host-timed trajectory.")
            return False
        try:
            segments = owner.build_host_segments_from_path(
                path,
                start_xy=(owner.cur_x, owner.cur_y),
                label="host timed trajectory",
            )
        except Exception as exc:
            owner.log_error(f"Host-timed segment planning failed: {exc}")
            return False
        owner.active_preview_path = list(path)
        return bool(owner._queue_gcode_segments(segments, label="host timed trajectory"))


class BufferedBinarySender(MotionSender):
    mode = "binary_buffered"

    def send(self, owner, path, *, append=False, send_path=None):
        if append or not send_path:
            return False
        if owner.waiting_for_ack or owner.point_queue:
            return False

        start_x, start_y = owner.cur_x, owner.cur_y
        owner.active_binary_send_path = list(send_path)
        owner.active_preview_path = list(path)
        if not owner._upload_binary_motion(send_path, preview_path=path):
            owner.active_binary_send_path = []
            return False

        owner.sent_point_id = len(send_path)
        owner.total_task_points = len(send_path)
        owner.task_start_time = owner._sender_now()
        owner.point_queue = []
        owner._clear_text_sender_state()
        if owner.plot_mode_combo.currentText() == "通讯发送内容":
            owner.history_x = [float(start_x)]
            owner.history_y = [float(start_y)]
            owner.update_plot(force=True)
        return True


class AsciiLegacyG1Sender(MotionSender):
    mode = "gcode_stream"

    def send(self, owner, path, *, append=False, send_path=None):
        if append and (owner.waiting_for_ack or owner.point_queue):
            owner.point_queue.extend(path)
            owner.total_task_points += len(path)
        else:
            owner.sent_point_id = 0
            owner.total_task_points = len(path)
            owner.task_start_time = owner._sender_now()
            owner._clear_text_sender_state()
            owner.point_queue = list(path)
        owner.active_preview_path = list(path)
        owner._set_sender_status(self.mode, queued_lines=len(owner.point_queue), inflight_lines=0)
        owner.process_queue()
        return True


HOST_TIMED_SEGMENT_SENDER = HostTimedSegmentSender()
BUFFERED_BINARY_SENDER = BufferedBinarySender()
ASCII_LEGACY_G1_SENDER = AsciiLegacyG1Sender()


def select_motion_sender(owner, *, append=False, send_path=None):
    serial_open = bool(owner.ser and owner.ser.is_open)
    if serial_open and bool(getattr(owner, "host_timed_segment_mode", False)):
        return HOST_TIMED_SEGMENT_SENDER
    if (
        serial_open
        and send_path
        and not append
        and not owner.waiting_for_ack
        and not owner.point_queue
    ):
        return BUFFERED_BINARY_SENDER
    return ASCII_LEGACY_G1_SENDER
