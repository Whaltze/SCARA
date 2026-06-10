"""Binary joint trajectory helpers for SCARA_F103.

The MCU protocol is:
    A5 5A | ver u8 | type u8 | seq u16le | len u16le | payload | crc16le

Trajectory point payload entries are:
    int32 p1_abs, int32 p2_abs, uint16 v_dom_pps, uint16 flags
"""

from dataclasses import dataclass
import math
import struct
from typing import Iterable, List, Tuple, Sequence, Any

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

RAD_PER_REV = 2.0 * math.pi
DEFAULT_ZERO_RAD = (2.251, 0.890)
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


def joint_deg_to_pulse(theta1_deg: float, theta2_deg: float, ppr: int, zero_rad=DEFAULT_ZERO_RAD) -> Tuple[int, int]:
    """关节角度转绝对脉冲。

    参数调节：
    - ppr：驱动器细分后的每圈脉冲数。PPR 越高，末端量化误差越小，但同样速度下 PPS 越高。
      当前 UI 默认 3200，是为了让小车/直线轨迹离线误差稳定压到 0.5mm 内。
    - zero_rad：电机软件零点弧度值，必须和固件里的零点保持一致，否则轨迹会整体偏移。

    全程保留浮点精度，只在生成最终绝对脉冲时舍入一次。
    """
    scale = float(int(ppr)) / RAD_PER_REV
    p1 = int(round((math.radians(float(theta1_deg)) - float(zero_rad[0])) * scale))
    p2 = int(round((math.radians(float(theta2_deg)) - float(zero_rad[1])) * scale))
    return p1, p2


def path_to_joint_points(
    path: Iterable[Any],  # 放宽类型检查，兼容 Tuple 和 PlannerPoint 对象
    kinematics,
    ppr: int,
    start_xy: Tuple[float, float] = None,
    min_pps: int = 16,
    max_pps: int = 10000,
) -> List[BinaryJointPoint]:
    """Convert UI path points to MCU binary joint trajectory points.

    支持接收传统的 UI 轨迹元组，或者由 LookAheadPlanner 输出的带真实 dt 时间的 PlannerPoint 对象。
    此函数将强制启用下位机的时间片同步模式 (HOST_TIMED)，从而实现 0 延迟、完全平滑的轨迹执行。
    
    参数调节：
    - ppr：驱动器细分后的每圈脉冲数。
    - start_xy：上下文起点，用于计算第一段插补的增量。
    """
    result: List[BinaryJointPoint] = []
    prev_pulse = None
    prev_xy = None
    
    # 1. 建立初始坐标上下文
    if start_xy is not None:
        sx, sy = float(start_xy[0]), float(start_xy[1])
        q1, q2 = kinematics.inverse(sx, sy)
        if q1 is None or q2 is None:
            raise ValueError(f"unreachable start point: X={sx:.3f}, Y={sy:.3f}")
        prev_pulse = joint_deg_to_pulse(q1, q2, ppr)
        prev_xy = (sx, sy)
        
    for point in path:
        # 2. 兼容对象的智能解包 (修复了原来重复解包覆盖的 Bug)
        if hasattr(point, 'x'):
            # 输入是 PlannerPoint 对象
            x = float(point.x)
            y = float(point.y)
            feed_mm_min = float(point.feed_mm_min)
            flags = getattr(point, 'flags', 0)
            dt = getattr(point, 'dt', 0.02)  # 获取上位机前瞻算好的真实耗时
        else:
            # 输入是旧版元组 (Tuple)
            x = float(point[0])
            y = float(point[1])
            feed_mm_min = float(point[2])
            flags = int(point[4]) if len(point) > 4 else 0
            dt = float(point[5]) if len(point) > 5 else 0.02
            
        # 3. 计算逆向运动学得到关节角度
        q1, q2 = kinematics.inverse(x, y)
        if q1 is None or q2 is None:
            raise ValueError(f"unreachable path point: X={x:.3f}, Y={y:.3f}")
            
        # 4. 转为绝对脉冲 (全程浮点运算，仅在此处做四舍五入，消除累积误差)
        p1, p2 = joint_deg_to_pulse(q1, q2, ppr)
        
        # 5. 笛卡尔直线模式 (交由下位机做逆解)，保持不变
        if flags & FLAG_CARTESIAN_LINE:
            base = float(getattr(getattr(kinematics, "config", None), "base_distance", 0.0))
            x_um = int(round((x - base * 0.5) * 1000.0))
            y_um = int(round(y * 1000.0))
            result.append(BinaryJointPoint(x_um, y_um, max(1, min(65535, int(round(feed_mm_min)))), flags=flags))
            prev_pulse = (p1, p2)
            prev_xy = (x, y)
            continue
            
        # =======================================================
        # 6. 核心优化：强制启用 HOST_TIMED 时间片模式，屏蔽下位机加减速
        # =======================================================
        flags |= FLAG_HOST_TIMED 
        
        if prev_pulse is None:
            # 如果是第一点且没有起点的上下文，假设用 20ms 完成 (10kHz * 0.02s)
            duration_ticks = int(0.02 * 10000)
        else:
            dp1 = abs(p1 - prev_pulse[0])
            dp2 = abs(p2 - prev_pulse[1])
            # 如果目标脉冲和上一个点一模一样，直接忽略该点，避免产生 0 距离微小段
            if max(dp1, dp2) == 0:
                prev_xy = (x, y)
                continue
                
            if dt > 0.0001:
                # 核心机制：将 LookAheadPlanner 给定的秒数 dt，直接映射为 10000Hz 下的 Timer Ticks
                duration_ticks = int(round(dt * 10000.0)) 
            else:
                # 容错：如果没获取到 dt，根据两点距离和设定速度反推耗时
                distance = math.hypot(x - prev_xy[0], y - prev_xy[1])
                feed_mm_s = max(0.1, feed_mm_min / 60.0)
                duration_ticks = int(round((distance / feed_mm_s) * 10000.0))
                
        # 限制时间范围：最低 1 拍，最高 65535 拍 (大约 6.55 秒，受限于协议 uint16)
        duration_ticks = max(1, min(65535, duration_ticks))
        
        # 将算好的 duration_ticks 填入第三个参数。
        # 在 HOST_TIMED 模式下，下位机会将该参数解释为 duration_ticks 而不是 v_dom_pps
        result.append(BinaryJointPoint(p1, p2, duration_ticks, flags=flags))
        
        # 更新上一刻状态
        prev_pulse = (p1, p2)
        prev_xy = (x, y)
        
    return result
