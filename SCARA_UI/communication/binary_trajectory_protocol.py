"""Binary joint trajectory helpers for SCARA_F103.

The MCU protocol is:
    A5 5A | ver u8 | type u8 | seq u16le | len u16le | payload | crc16le

Trajectory point payload entries are:
    int32 p1_abs, int32 p2_abs, uint16 v_dom_pps, uint16 flags
"""

from dataclasses import dataclass
import math
import struct
from typing import Iterable, List, Sequence, Tuple


SOF = b"\xA5\x5A"
VERSION = 1

TYPE_HELLO = 0x01
TYPE_BEGIN = 0x10
TYPE_CHUNK = 0x11
TYPE_VALIDATE = 0x12
TYPE_RUN = 0x13
TYPE_ABORT = 0x14
TYPE_STATUS = 0x15

TYPE_ACK = 0x80
TYPE_NACK = 0x81
TYPE_STATUS_RSP = 0x82

MRAD_PER_REV = 6283
DEFAULT_ZERO_MRAD = (2251, 890)
FLAG_EXACT_STOP = 0x0001
FLAG_CARTESIAN_LINE = 0x0002
FLAG_HOST_TIMED = 0x0004


@dataclass
class BinaryJointPoint:
    p1_abs: int
    p2_abs: int
    v_dom_pps: int
    flags: int = 0


@dataclass
class BinaryFrame:
    version: int
    frame_type: int
    seq: int
    payload: bytes


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = ((crc >> 1) ^ 0xA001) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc


def build_frame(frame_type: int, seq: int, payload: bytes = b"") -> bytes:
    header = struct.pack("<BBHH", VERSION, int(frame_type) & 0xFF, int(seq) & 0xFFFF, len(payload))
    body = header + payload
    return SOF + body + struct.pack("<H", crc16(body))


def parse_frame(data: bytes) -> BinaryFrame:
    if len(data) < 10 or data[:2] != SOF:
        raise ValueError("invalid binary frame header")
    version, frame_type, seq, length = struct.unpack_from("<BBHH", data, 2)
    expected_len = 2 + 6 + length + 2
    if len(data) != expected_len:
        raise ValueError(f"invalid frame length: got {len(data)}, expected {expected_len}")
    payload = data[8 : 8 + length]
    rx_crc = struct.unpack_from("<H", data, 8 + length)[0]
    calc_crc = crc16(data[2 : 8 + length])
    if rx_crc != calc_crc:
        raise ValueError(f"crc mismatch: rx={rx_crc:04X}, calc={calc_crc:04X}")
    return BinaryFrame(version=version, frame_type=frame_type, seq=seq, payload=payload)


def build_begin_payload(total_points: int) -> bytes:
    return struct.pack("<I", int(total_points))


def build_chunk_payload(points: Sequence[BinaryJointPoint]) -> bytes:
    out = bytearray()
    for point in points:
        out.extend(
            struct.pack(
                "<iiHH",
                int(point.p1_abs),
                int(point.p2_abs),
                max(1, min(65535, int(point.v_dom_pps))),
                int(point.flags) & 0xFFFF,
            )
        )
    return bytes(out)


def joint_deg_to_pulse(theta1_deg: float, theta2_deg: float, ppr: int, zero_mrad=DEFAULT_ZERO_MRAD) -> Tuple[int, int]:
    """关节角度转绝对脉冲。

    参数调节：
    - ppr：驱动器细分后的每圈脉冲数。PPR 越高，末端量化误差越小，但同样速度下 PPS 越高。
      当前 UI 默认 3200，是为了让小车/直线轨迹离线误差稳定压到 0.5mm 内。
    - zero_mrad：电机软件零点，必须和固件里的零点保持一致，否则轨迹会整体偏移。

    这里使用四舍五入而不是截断，避免所有关键点系统性偏向同一侧。
    """
    theta1_mrad = int(round(math.radians(theta1_deg) * 1000.0))
    theta2_mrad = int(round(math.radians(theta2_deg) * 1000.0))
    p1 = int(round(((theta1_mrad - int(zero_mrad[0])) * int(ppr)) / MRAD_PER_REV))
    p2 = int(round(((theta2_mrad - int(zero_mrad[1])) * int(ppr)) / MRAD_PER_REV))
    return p1, p2


def path_to_joint_points(
    path: Iterable[Tuple[float, float, float, bool]],
    kinematics,
    ppr: int,
    start_xy: Tuple[float, float] = None,
    min_pps: int = 16,
    max_pps: int = 10000,
) -> List[BinaryJointPoint]:
    """Convert UI path points to MCU binary joint trajectory points.

    ``path`` entries are the existing UI tuples: x_mm, y_mm, feed_mm_min, silent.
    ``kinematics`` must provide the current ``inverse(x, y) -> q1_deg, q2_deg`` method.

    调节说明：
    - min_pps：最低主导轴速度，太低会导致很短线段启动拖沓。
    - max_pps：最高主导轴速度，必须不超过固件和驱动器可稳定输出的 PPS。
    - feed_mm_min：来自 UI 轨迹速度，函数会根据相邻关键点距离换算为主导轴 PPS。
    """
    result: List[BinaryJointPoint] = []
    prev_pulse = None
    prev_xy = None
    if start_xy is not None:
        sx, sy = float(start_xy[0]), float(start_xy[1])
        q1, q2 = kinematics.inverse(sx, sy)
        if q1 is None or q2 is None:
            raise ValueError(f"unreachable start point: X={sx:.3f}, Y={sy:.3f}")
        prev_pulse = joint_deg_to_pulse(q1, q2, ppr)
        prev_xy = (sx, sy)
    for point in path:
        x, y, feed_mm_min = point[0], point[1], point[2]
        flags = int(point[4]) if len(point) > 4 else 0
        x = float(x)
        y = float(y)
        feed_mm_min = float(feed_mm_min)
        q1, q2 = kinematics.inverse(float(x), float(y))
        if q1 is None or q2 is None:
            raise ValueError(f"unreachable path point: X={x:.3f}, Y={y:.3f}")
        p1, p2 = joint_deg_to_pulse(q1, q2, ppr)
        if flags & FLAG_CARTESIAN_LINE:
            base = float(getattr(getattr(kinematics, "config", None), "base_distance", 0.0))
            x_um = int(round((x - base * 0.5) * 1000.0))
            y_um = int(round(y * 1000.0))
            result.append(BinaryJointPoint(x_um, y_um, max(1, min(65535, int(round(feed_mm_min)))), flags=flags))
            prev_pulse = (p1, p2)
            prev_xy = (x, y)
            continue
        if prev_pulse is None:
            v_dom = max(min_pps, min(max_pps, int(feed_mm_min / 60.0 * 30.0)))
        else:
            dp1 = abs(p1 - prev_pulse[0])
            dp2 = abs(p2 - prev_pulse[1])
            dom = max(dp1, dp2)
            if dom == 0:
                prev_xy = (x, y)
                continue
            if prev_xy is not None:
                distance = math.hypot(x - prev_xy[0], y - prev_xy[1])
                feed_mm_s = max(0.1, feed_mm_min / 60.0)
                if distance > 1e-6:
                    duration = distance / feed_mm_s
                    v_dom = int(math.ceil(dom / max(duration, 1e-6)))
                else:
                    v_dom = int(feed_mm_s * 30.0)
            else:
                v_dom = int(max(dom, 1) * 20)
            v_dom = max(min_pps, min(max_pps, v_dom))
        result.append(BinaryJointPoint(p1, p2, v_dom, flags=flags))
        prev_pulse = (p1, p2)
        prev_xy = (x, y)
    return result
