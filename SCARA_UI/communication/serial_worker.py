"""QThread-backed serial transport with non-blocking UI-facing methods."""

from queue import Empty, Queue
from threading import Event

import serial
from PySide6.QtCore import QThread


class SerialThreadTransport(QThread):
    def __init__(self, port, baudrate=115200, parent=None):
        super().__init__(parent)
        self.port = str(port)
        self.baudrate = int(baudrate)
        self._tx = Queue()
        self._rx = Queue()
        self._stop_event = Event()
        self._open_event = Event()
        self._serial = None
        self.error = None

    @property
    def is_open(self):
        return self._open_event.is_set() and self.error is None and not self._stop_event.is_set()

    @property
    def in_waiting(self):
        return self._rx.qsize()

    def open(self, timeout_s=1.5):
        self.start()
        if not self._open_event.wait(float(timeout_s)):
            raise TimeoutError(f"serial open timeout: {self.port}")
        if self.error is not None:
            raise self.error

    def write(self, data):
        payload = bytes(data)
        if not self.is_open:
            raise serial.SerialException("serial transport is not open")
        self._tx.put(payload)
        return len(payload)

    def readline(self):
        try:
            return self._rx.get_nowait()
        except Empty:
            return b""

    def flush(self):
        return None

    def close(self):
        self._stop_event.set()
        self.wait(1500)

    def run(self):
        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.02, write_timeout=0.2)
            self._open_event.set()
            while not self._stop_event.is_set():
                try:
                    while True:
                        self._serial.write(self._tx.get_nowait())
                except Empty:
                    pass
                line = self._serial.readline()
                if line:
                    self._rx.put(bytes(line))
        except Exception as exc:
            self.error = exc
            self._open_event.set()
        finally:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
            self._stop_event.set()
