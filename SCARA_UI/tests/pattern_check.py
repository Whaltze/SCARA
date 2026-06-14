"""自定义图案（SVG 线条 / 图片点阵）离线验证脚本。

直接运行：``python SCARA_UI/tests/pattern_check.py``（需 robot 环境，含 PySide6 + OpenCV）。
不依赖串口或 GUI，验证三件事：
  1. SVG → build_svg_outline_strokes 产出非空笔画，且全部落在画布安全框内（缩放居中正确）；
  2. 图片 → build_halftone_dots 产出打点，且落在点阵安全框内（Y 翻转 / 居中正确）；
  3. _halftone_commands 产出的点阵 G-code 结构正确（首条 M5、含 M4 S 出光与 G4 P 驻点停留）。
"""

from pathlib import Path
import os
import sys
import tempfile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from SCARA_UI.motion.motion_mixin import ScaraMotionMixin


class PatternOwner(ScaraMotionMixin):
    """只补 build_*/_halftone_commands 路径上用到的两个跨 mixin 方法。"""

    def _laser_s_word(self):
        return 200

    def ui_to_mcu_xy(self, x, y):
        return x - 75.0, y


def _in_box(values, center, span, tol=0.5):
    return (center - span / 2.0 - tol) <= min(values) and max(values) <= (center + span / 2.0 + tol)


def main():
    owner = PatternOwner()
    patterns = ROOT / "SCARA_UI" / "trajectory" / "patterns"
    cx, cy = owner.DRAW_CENTER_X, owner.DRAW_CENTER_Y

    # 1. SVG 线条
    svgs = sorted(patterns.glob("*.svg"))
    assert svgs, "patterns/ 下没有示例 .svg"
    strokes = owner.build_svg_outline_strokes(str(svgs[0]))
    assert strokes, "SVG 未解析出笔画"
    xs = [x for stroke in strokes for x, _ in stroke]
    ys = [y for stroke in strokes for _, y in stroke]
    assert _in_box(xs, cx, owner.PATTERN_MAX_WIDTH_MM), "SVG X 超出画布安全框"
    assert _in_box(ys, cy, owner.PATTERN_MAX_HEIGHT_MM), "SVG Y 超出画布安全框"
    print(f"[SVG] {svgs[0].name}: {len(strokes)} 笔 {len(xs)} 点  "
          f"X[{min(xs):.1f},{max(xs):.1f}] Y[{min(ys):.1f},{max(ys):.1f}]  OK")

    # 2. 图片点阵（临时生成一张黑色实心圆测试图）
    img = np.full((120, 160), 255, np.uint8)
    cv2.circle(img, (80, 60), 45, 0, -1)
    with tempfile.TemporaryDirectory() as td:
        fp = os.path.join(td, "halftone_test.png")
        cv2.imwrite(fp, img)
        dots = owner.build_halftone_dots(fp)
    assert dots, "点阵未产出打点"
    dxs = [d[0] for d in dots]
    dys = [d[1] for d in dots]
    assert _in_box(dxs, cx, owner.HALFTONE_MAX_WIDTH_MM), "点阵 X 超出安全框"
    assert _in_box(dys, cy, owner.HALFTONE_MAX_HEIGHT_MM), "点阵 Y 超出安全框"
    print(f"[点阵] {len(dots)} 打点  X[{min(dxs):.1f},{max(dxs):.1f}] Y[{min(dys):.1f},{max(dys):.1f}]  OK")

    # 3. 点阵 G-code 结构
    gcode = list(owner._halftone_commands(dots[:3]))
    assert gcode[0] == "M5", "首条应为 M5（保证移到首点不出光）"
    assert any(s.startswith("M4 S") for s in gcode), "缺少出光指令 M4 S"
    assert any(s.startswith("G4 P") for s in gcode), "缺少驻点停留 G4 P"
    assert any(s.startswith("G0 ") for s in gcode), "缺少移动指令 G0"
    print(f"[G-code] 前 9 条: {gcode[:9]}  OK")

    print("ALL OK")


if __name__ == "__main__":
    main()
