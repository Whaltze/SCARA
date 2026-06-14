"""Helpers for the standard GRBL text protocol."""

from dataclasses import dataclass


@dataclass
class AckResult:
    """Parsed standard GRBL command acknowledgement."""

    raw: str
    matched: bool = False


def build_g1_line(
    x: float,
    y: float,
    feed_mm_min: float,
    point_id: int,
    limit_checked: bool = True,
    laser_mark: bool = False,
    laser_prep: bool = False,
) -> str:
    """Build one Cartesian G1 command."""
    lim = 1 if limit_checked else 0
    return f"G1 X{x:.3f} Y{y:.3f} F{feed_mm_min:.0f} ;ID={point_id} LIM={lim}"


def build_ppr_line(ppr1: int, ppr2: int = None) -> str:
    """Build the firmware command that matches the UI pulses/rev selection."""
    ppr1 = int(ppr1)
    ppr2 = ppr1 if ppr2 is None else int(ppr2)
    return f"$100={ppr1} $101={ppr2}"


def parse_ok_ack(raw: str) -> AckResult:
    """Accept only the standard GRBL command acknowledgement."""
    return AckResult(raw=raw, matched=raw.strip().lower() == "ok")
