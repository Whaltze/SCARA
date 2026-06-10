"""SCARA_F103 串口协议工具。

这里只处理文本协议的校验、G-code 组帧和 ACK 回显解析，不直接操作 UI。
"""

from dataclasses import dataclass
import re
from typing import Optional


@dataclass
class AckResult:
    """下位机 ok 回显解析结果。"""

    raw: str
    rx_checksum: Optional[str] = None
    rx_line: Optional[str] = None
    matched: bool = False
    expected_checksum: str = ""


def checksum(line: str) -> str:
    """8 位 ASCII 累加校验，和下位机 ok seq/cs/line 回显保持一致。"""
    text = line.strip()
    return f"{sum(text.encode('ascii', errors='ignore')) & 0xFF:02X}"


def build_g1_line(x: float, y: float, feed_mm_min: float, point_id: int, limit_checked: bool = True) -> str:
    """生成一条上位机规划后的 G1 文本指令。"""
    lim = 1 if limit_checked else 0
    return f"G1 X{x:.3f} Y{y:.3f} F{feed_mm_min:.0f} ;ID={point_id} LIM={lim}"


def build_g1_timed_line(p1_abs: int, p2_abs: int, duration_ticks: int, point_id: int = 0) -> str:
    """Build a host-planned joint timed segment."""
    p1_abs = int(p1_abs)
    p2_abs = int(p2_abs)
    duration_ticks = int(duration_ticks)
    suffix = f" ;ID={int(point_id)} HOST=1" if point_id else " ;HOST=1"
    return f"G1 A{p1_abs} B{p2_abs} T{duration_ticks}{suffix}"


def build_ppr_line(ppr1: int, ppr2: int = None) -> str:
    """Build the firmware command that matches the UI pulses/rev selection."""
    ppr1 = int(ppr1)
    ppr2 = ppr1 if ppr2 is None else int(ppr2)
    return f"$100={ppr1} $101={ppr2}"


def parse_ok_ack(raw: str, expected_line: str) -> AckResult:
    """解析下位机 ok 回显，并检查 cs 和 line 是否和上位机最近发送一致。"""
    cs_match = re.search(r"cs=([0-9A-F]{2})", raw)
    line_match = re.search(r"line=(.*)", raw)
    expected = checksum(expected_line)
    result = AckResult(raw=raw, expected_checksum=expected)
    if cs_match:
        result.rx_checksum = cs_match.group(1)
    if line_match:
        result.rx_line = line_match.group(1).strip()
    result.matched = (
        result.rx_checksum is not None
        and result.rx_line is not None
        and result.rx_checksum.upper() == expected
        and result.rx_line == expected_line.strip()
    )
    return result
