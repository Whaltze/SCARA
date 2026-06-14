"""轻量 SVG → QPainterPath 解析器。

仅依赖标准库 ``xml.etree`` 与 ``PySide6.QtGui``，不引入第三方 SVG 库
（环境中无 svgelements/svgpathtools；QtSvg 只能光栅化、拿不到矢量路径）。

用途：把矢量线条图（卡通人物等）解析成一组 ``QPainterPath``，交给上位机
现成的轮廓→折线管线（``ScaraMotionMixin._qt_path_contours``）转成绘制轨迹。

覆盖范围（足以处理 Inkscape / Illustrator 导出的描边线条图）：

- ``<path d=...>``：``M/m L/l H/h V/v C/c S/s Q/q T/t Z/z``；``A/a`` 椭圆弧用
  线段近似（已足够，Inkscape 多已把弧转成 C）。
- 基本图元：``<line> <polyline> <polygon> <rect> <circle> <ellipse>``。
- ``transform``：``translate/scale/rotate/matrix/skewX/skewY``，``<g>`` 嵌套逐层累积。

返回的每个 ``QPainterPath`` 都已应用其累积 ``transform``，处于 SVG 用户坐标系
（Y 轴向下）。下游负责等比缩放、居中与 Y 翻转，故此处忽略 viewBox 的绝对尺度与样式。
"""

import math
import re
import xml.etree.ElementTree as ET

from PySide6.QtGui import QPainterPath, QTransform

# path d 的 token：命令字母，或一个数字（含小数 / 科学计数 / 正负号）。
_PATH_TOKEN_RE = re.compile(r"[MmLlHhVvCcSsQqTtAaZz]|[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?")
_NUMBER_RE = re.compile(r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?")
_TRANSFORM_RE = re.compile(r"(matrix|translate|scale|rotate|skewX|skewY)\s*\(([^)]*)\)")
_PATH_CMDS = "MmLlHhVvCcSsQqTtAaZz"
_SHAPE_TAGS = ("rect", "circle", "ellipse", "line", "polyline", "polygon")

# 仿射矩阵用 (a, b, c, d, e, f) 表示，作用于点：x' = a*x + c*y + e, y' = b*x + d*y + f。
# 这正是 SVG ``matrix(a b c d e f)`` 的定义，也与 Qt 的 QTransform(a,b,c,d,e,f) 行向量约定一致。
_IDENTITY = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def _strip_ns(tag):
    """去掉 ``{namespace}`` 前缀，返回纯标签名。"""
    return tag.split("}", 1)[1] if "}" in tag else tag


def _floats(text):
    return [float(v) for v in _NUMBER_RE.findall(text or "")]


def _mat_mul(m1, m2):
    """仿射组合：返回“先应用 m2、再应用 m1”的矩阵。"""
    a1, b1, c1, d1, e1, f1 = m1
    a2, b2, c2, d2, e2, f2 = m2
    return (
        a1 * a2 + c1 * b2,
        b1 * a2 + d1 * b2,
        a1 * c2 + c1 * d2,
        b1 * c2 + d1 * d2,
        a1 * e2 + c1 * f2 + e1,
        b1 * e2 + d1 * f2 + f1,
    )


def _parse_transform(text):
    """把 SVG ``transform`` 属性解析为 (a,b,c,d,e,f)。列表从左到右组合：M = T1∘T2∘…。"""
    m = _IDENTITY
    if not text:
        return m
    for name, arg in _TRANSFORM_RE.findall(text):
        v = _floats(arg)
        if name == "matrix" and len(v) >= 6:
            t = (v[0], v[1], v[2], v[3], v[4], v[5])
        elif name == "translate":
            tx = v[0] if v else 0.0
            ty = v[1] if len(v) > 1 else 0.0
            t = (1.0, 0.0, 0.0, 1.0, tx, ty)
        elif name == "scale":
            sx = v[0] if v else 1.0
            sy = v[1] if len(v) > 1 else sx
            t = (sx, 0.0, 0.0, sy, 0.0, 0.0)
        elif name == "rotate":
            ang = math.radians(v[0]) if v else 0.0
            cos_a, sin_a = math.cos(ang), math.sin(ang)
            rot = (cos_a, sin_a, -sin_a, cos_a, 0.0, 0.0)
            if len(v) >= 3:  # 绕 (cx,cy) 旋转
                cx, cy = v[1], v[2]
                t = _mat_mul((1.0, 0.0, 0.0, 1.0, cx, cy),
                             _mat_mul(rot, (1.0, 0.0, 0.0, 1.0, -cx, -cy)))
            else:
                t = rot
        elif name == "skewX":
            t = (1.0, 0.0, math.tan(math.radians(v[0] if v else 0.0)), 1.0, 0.0, 0.0)
        elif name == "skewY":
            t = (1.0, math.tan(math.radians(v[0] if v else 0.0)), 0.0, 1.0, 0.0, 0.0)
        else:
            continue
        m = _mat_mul(m, t)
    return m


def _arc_to(path, x1, y1, rx, ry, phi_deg, large_arc, sweep, x2, y2):
    """SVG endpoint 弧 → 线段近似（标准 endpoint→center 换算，每约 10° 一段）。"""
    if rx == 0.0 or ry == 0.0 or (x1 == x2 and y1 == y2):
        path.lineTo(x2, y2)
        return
    rx, ry = abs(rx), abs(ry)
    phi = math.radians(phi_deg)
    cosp, sinp = math.cos(phi), math.sin(phi)
    dx, dy = (x1 - x2) / 2.0, (y1 - y2) / 2.0
    x1p = cosp * dx + sinp * dy
    y1p = -sinp * dx + cosp * dy
    lam = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry)
    if lam > 1.0:  # 半径过小，按规范放大
        s = math.sqrt(lam)
        rx, ry = rx * s, ry * s
    den = rx * rx * y1p * y1p + ry * ry * x1p * x1p
    num = rx * rx * ry * ry - den
    sign = -1.0 if bool(large_arc) == bool(sweep) else 1.0
    co = sign * math.sqrt(max(0.0, num / den)) if den else 0.0
    cxp = co * rx * y1p / ry
    cyp = -co * ry * x1p / rx
    cx = cosp * cxp - sinp * cyp + (x1 + x2) / 2.0
    cy = sinp * cxp + cosp * cyp + (y1 + y2) / 2.0

    def _angle(ux, uy, vx, vy):
        dot = ux * vx + uy * vy
        ln = math.hypot(ux, uy) * math.hypot(vx, vy)
        ang = math.acos(max(-1.0, min(1.0, dot / ln))) if ln else 0.0
        return -ang if (ux * vy - uy * vx) < 0 else ang

    ux, uy = (x1p - cxp) / rx, (y1p - cyp) / ry
    vx, vy = (-x1p - cxp) / rx, (-y1p - cyp) / ry
    theta1 = _angle(1.0, 0.0, ux, uy)
    dtheta = _angle(ux, uy, vx, vy)
    if not sweep and dtheta > 0:
        dtheta -= 2.0 * math.pi
    elif sweep and dtheta < 0:
        dtheta += 2.0 * math.pi
    segs = max(2, int(abs(dtheta) / (math.pi / 18.0)))
    for k in range(1, segs + 1):
        t = theta1 + dtheta * k / segs
        ex = cosp * rx * math.cos(t) - sinp * ry * math.sin(t) + cx
        ey = sinp * rx * math.cos(t) + cosp * ry * math.sin(t) + cy
        path.lineTo(ex, ey)


def _parse_path_d(d):
    """解析 ``<path>`` 的 ``d`` 属性为 QPainterPath（元素局部坐标）。"""
    path = QPainterPath()
    tokens = _PATH_TOKEN_RE.findall(d or "")
    n = len(tokens)
    i = 0
    cx = cy = 0.0       # 当前点
    sx = sy = 0.0       # 子路径起点（Z 回到此处）
    prev_cmd = ""
    last_c2 = None      # 上一条三次曲线的第二控制点（S 命令反射用）
    last_q = None       # 上一条二次曲线的控制点（T 命令反射用）

    def num():
        nonlocal i
        v = float(tokens[i])
        i += 1
        return v

    while i < n:
        tok = tokens[i]
        if tok in _PATH_CMDS:
            cmd = tok
            i += 1
        else:
            cmd = prev_cmd  # 隐式重复
            if cmd == "M":
                cmd = "L"
            elif cmd == "m":
                cmd = "l"
        if not cmd:
            i += 1
            continue
        rel = cmd.islower()
        c = cmd.upper()
        if c == "M":
            x, y = num(), num()
            if rel:
                x, y = x + cx, y + cy
            cx, cy = x, y
            sx, sy = x, y
            path.moveTo(cx, cy)
        elif c == "L":
            x, y = num(), num()
            if rel:
                x, y = x + cx, y + cy
            cx, cy = x, y
            path.lineTo(cx, cy)
        elif c == "H":
            x = num()
            cx = x + cx if rel else x
            path.lineTo(cx, cy)
        elif c == "V":
            y = num()
            cy = y + cy if rel else y
            path.lineTo(cx, cy)
        elif c == "C":
            x1, y1, x2, y2, x, y = num(), num(), num(), num(), num(), num()
            if rel:
                x1, y1, x2, y2, x, y = x1 + cx, y1 + cy, x2 + cx, y2 + cy, x + cx, y + cy
            path.cubicTo(x1, y1, x2, y2, x, y)
            last_c2, (cx, cy) = (x2, y2), (x, y)
        elif c == "S":
            x2, y2, x, y = num(), num(), num(), num()
            if rel:
                x2, y2, x, y = x2 + cx, y2 + cy, x + cx, y + cy
            if prev_cmd.upper() in ("C", "S") and last_c2 is not None:
                x1, y1 = 2.0 * cx - last_c2[0], 2.0 * cy - last_c2[1]
            else:
                x1, y1 = cx, cy
            path.cubicTo(x1, y1, x2, y2, x, y)
            last_c2, (cx, cy) = (x2, y2), (x, y)
        elif c == "Q":
            x1, y1, x, y = num(), num(), num(), num()
            if rel:
                x1, y1, x, y = x1 + cx, y1 + cy, x + cx, y + cy
            path.quadTo(x1, y1, x, y)
            last_q, (cx, cy) = (x1, y1), (x, y)
        elif c == "T":
            x, y = num(), num()
            if rel:
                x, y = x + cx, y + cy
            if prev_cmd.upper() in ("Q", "T") and last_q is not None:
                x1, y1 = 2.0 * cx - last_q[0], 2.0 * cy - last_q[1]
            else:
                x1, y1 = cx, cy
            path.quadTo(x1, y1, x, y)
            last_q, (cx, cy) = (x1, y1), (x, y)
        elif c == "A":
            rx, ry, rot, large, sweep, x, y = (num(), num(), num(), num(), num(), num(), num())
            if rel:
                x, y = x + cx, y + cy
            _arc_to(path, cx, cy, rx, ry, rot, large, sweep, x, y)
            cx, cy = x, y
        elif c == "Z":
            path.closeSubpath()
            cx, cy = sx, sy
        prev_cmd = cmd
    return path


def _shape_path(tag, attr):
    """把基本图元元素转成 QPainterPath（元素局部坐标）。"""
    p = QPainterPath()
    g = lambda k, default=0.0: float(attr.get(k, default))
    if tag == "rect":
        w, h = g("width"), g("height")
        if w > 0 and h > 0:
            p.addRect(g("x"), g("y"), w, h)
    elif tag == "circle":
        r = g("r")
        if r > 0:
            p.addEllipse(g("cx") - r, g("cy") - r, 2.0 * r, 2.0 * r)
    elif tag == "ellipse":
        rx, ry = g("rx"), g("ry")
        if rx > 0 and ry > 0:
            p.addEllipse(g("cx") - rx, g("cy") - ry, 2.0 * rx, 2.0 * ry)
    elif tag == "line":
        p.moveTo(g("x1"), g("y1"))
        p.lineTo(g("x2"), g("y2"))
    elif tag in ("polyline", "polygon"):
        pts = _floats(attr.get("points", ""))
        if len(pts) >= 4:
            p.moveTo(pts[0], pts[1])
            for k in range(2, len(pts) - 1, 2):
                p.lineTo(pts[k], pts[k + 1])
            if tag == "polygon":
                p.closeSubpath()
    return p


def _apply(matrix, path):
    a, b, c, d, e, f = matrix
    return QTransform(a, b, c, d, e, f).map(path)


def load_svg_paths(svg_file):
    """解析 SVG 文件，返回一组 QPainterPath（已应用各自 transform，SVG 用户坐标系）。

    解析失败（文件损坏 / 非 SVG）会抛 ``xml.etree.ElementTree.ParseError`` 等异常，
    由调用方捕获并提示。
    """
    root = ET.parse(svg_file).getroot()
    paths = []

    def walk(element, ctm):
        matrix = _mat_mul(ctm, _parse_transform(element.get("transform", "")))
        tag = _strip_ns(element.tag)
        if tag == "path":
            sub = _parse_path_d(element.get("d", ""))
            if not sub.isEmpty():
                paths.append(_apply(matrix, sub))
        elif tag in _SHAPE_TAGS:
            sub = _shape_path(tag, element.attrib)
            if not sub.isEmpty():
                paths.append(_apply(matrix, sub))
        for child in element:  # 递归 g/svg/a 等容器
            walk(child, matrix)

    walk(root, _IDENTITY)
    return paths
