# -*- coding: utf-8 -*-
"""参考黑线：弧长参数化的平面路径。

实物代码只消费 8 路模拟量，不包含地图。这里构造一条带直道、缓弯、
S 弯、急弯和直角弯的中心线，供传感器模型计算到线的距离，也供作图。

曲率 κ > 0 表示向左（逆时针）转弯。直角弯用半径 0.05 m 的 90° 圆弧，
曲率从直道的 0 突变到 20 m⁻¹，用来表现“曲率突变”，而不是把 PID 调到能过弯。
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Segment:
    kind: str
    name: str
    length: float
    kappa: float
    s0: float = 0.0
    x0: float = 0.0
    y0: float = 0.0
    h0: float = 0.0

    @property
    def s1(self):
        return self.s0 + self.length

    @property
    def radius(self):
        if abs(self.kappa) < 1e-12:
            return float("inf")
        return 1.0 / abs(self.kappa)


def pose_on_segment(x0, y0, h0, kappa, s_local):
    """段内弧长 ``s_local`` 处的位姿。直线或圆弧都有解析式。"""
    if abs(kappa) < 1e-12:
        return (
            x0 + s_local * np.cos(h0),
            y0 + s_local * np.sin(h0),
            h0,
        )
    cx = x0 - np.sin(h0) / kappa
    cy = y0 + np.cos(h0) / kappa
    phi = kappa * s_local
    v0x = x0 - cx
    v0y = y0 - cy
    c = np.cos(phi)
    sn = np.sin(phi)
    return (
        cx + c * v0x - sn * v0y,
        cy + sn * v0x + c * v0y,
        h0 + phi,
    )


class ReferencePath:
    """由若干直线/圆弧段串成的路径，并预采样成折线以便投影。"""

    def __init__(self, segments, ds=0.002):
        if not segments:
            raise ValueError("路径至少需要一段")
        self.ds = float(ds)
        self.segments = self._chain(segments)
        self.total_length = self.segments[-1].s1
        self._sample()

    @staticmethod
    def _chain(segments):
        x, y, h = 0.0, 0.0, 0.0
        s0 = 0.0
        chained = []
        for seg in segments:
            if seg.length <= 0:
                raise ValueError("段长度必须为正")
            chained.append(
                Segment(
                    kind=seg.kind,
                    name=seg.name,
                    length=float(seg.length),
                    kappa=float(seg.kappa),
                    s0=s0,
                    x0=x,
                    y0=y,
                    h0=h,
                )
            )
            x, y, h = pose_on_segment(x, y, h, seg.kappa, seg.length)
            s0 += float(seg.length)
        return chained

    def _sample(self):
        s_list, x_list, y_list, h_list, k_list, idx_list = [], [], [], [], [], []
        for i, seg in enumerate(self.segments):
            n = max(1, int(np.round(seg.length / self.ds)))
            local = np.linspace(0.0, seg.length, n + 1)
            if i > 0:
                local = local[1:]
            for s_local in local:
                x, y, h = pose_on_segment(seg.x0, seg.y0, seg.h0, seg.kappa, float(s_local))
                s_list.append(seg.s0 + float(s_local))
                x_list.append(x)
                y_list.append(y)
                h_list.append(h)
                k_list.append(seg.kappa)
                idx_list.append(i)
        self.s = np.asarray(s_list, dtype=float)
        self.x = np.asarray(x_list, dtype=float)
        self.y = np.asarray(y_list, dtype=float)
        self.heading = np.asarray(h_list, dtype=float)
        self.curvature = np.asarray(k_list, dtype=float)
        self.sample_segment = np.asarray(idx_list, dtype=int)

    def pose(self, s):
        """弧长 s 处的解析位姿，返回 x, y, heading, kappa。"""
        s = float(np.clip(s, 0.0, self.total_length))
        for seg in self.segments:
            if s <= seg.s1 + 1e-9:
                local = min(seg.length, max(0.0, s - seg.s0))
                x, y, h = pose_on_segment(seg.x0, seg.y0, seg.h0, seg.kappa, local)
                return x, y, h, seg.kappa
        seg = self.segments[-1]
        x, y, h = pose_on_segment(seg.x0, seg.y0, seg.h0, seg.kappa, seg.length)
        return x, y, h, seg.kappa

    def segment_index_at(self, s):
        s = float(s)
        for i, seg in enumerate(self.segments):
            if s <= seg.s1 + 1e-9:
                return i
        return len(self.segments) - 1

    def project(self, x, y, hint_s=None):
        """把点投影到中心线折线上。

        ``hint_s`` 给定时只在该弧长附近搜索，避免 S 弯折返后吸附到另一支。
        返回字典中：

        - ``d_left``：点在路径左侧为正（沿切向的左法向）
        - ``euclid``：到中心线的欧氏距离，传感器模拟量用这个距离
        """
        x = float(x)
        y = float(y)
        n = len(self.s)
        if hint_s is None:
            i0, i1 = 0, n
        else:
            i0 = int(np.searchsorted(self.s, float(hint_s) - 0.30, side="left"))
            i1 = int(np.searchsorted(self.s, float(hint_s) + 0.80, side="right"))
            i0 = max(0, min(i0, n - 1))
            i1 = max(i0 + 1, min(i1, n))
        window_x = self.x[i0:i1]
        window_y = self.y[i0:i1]
        d2 = (window_x - x) ** 2 + (window_y - y) ** 2
        j = i0 + int(np.argmin(d2))
        return self._project_vertex(j, x, y)

    def _project_vertex(self, j, x, y):
        best = None
        for a, b in ((j - 1, j), (j, j + 1)):
            if a < 0 or b >= len(self.s):
                continue
            cand = self._project_edge(a, b, x, y)
            if best is None or cand["dist2"] < best["dist2"]:
                best = cand
        if best is None:
            raise RuntimeError("路径采样点不足以投影")
        return best

    def _project_edge(self, a, b, x, y):
        ax, ay = float(self.x[a]), float(self.y[a])
        bx, by = float(self.x[b]), float(self.y[b])
        vx, vy = bx - ax, by - ay
        length2 = vx * vx + vy * vy
        if length2 < 1e-16:
            t = 0.0
        else:
            t = ((x - ax) * vx + (y - ay) * vy) / length2
            t = float(min(1.0, max(0.0, t)))
        px = ax + t * vx
        py = ay + t * vy
        dx = x - px
        dy = y - py
        s = float(self.s[a] + t * (self.s[b] - self.s[a]))
        heading = float(self.heading[a] + t * (self.heading[b] - self.heading[a]))
        kappa = float(self.curvature[a] if t < 0.5 else self.curvature[b])
        d_left = float(-np.sin(heading) * dx + np.cos(heading) * dy)
        dist2 = dx * dx + dy * dy
        seg_i = int(self.sample_segment[b] if t > 0.999 else self.sample_segment[a])
        return {
            "s": s,
            "x": px,
            "y": py,
            "heading": heading,
            "curvature": kappa,
            "d_left": d_left,
            "euclid": float(np.sqrt(dist2)),
            "dist2": float(dist2),
            "segment_index": seg_i,
            "clamped_end": t <= 0.0 or t >= 1.0,
        }

    def intervals(self, kind):
        """同一 kind 的各段弧长区间 ``(s0, s1, name)``。"""
        return [(seg.s0, seg.s1, seg.name) for seg in self.segments if seg.kind == kind]

    def describe(self):
        rows = []
        for seg in self.segments:
            rows.append(
                {
                    "name": seg.name,
                    "kind": seg.kind,
                    "s0": seg.s0,
                    "s1": seg.s1,
                    "length": seg.length,
                    "kappa": seg.kappa,
                    "radius": seg.radius,
                    "heading0_deg": float(np.degrees(seg.h0)),
                }
            )
        return rows


def build_demo_path(ds=0.002):
    """验收用路径：直道、缓弯 R=0.8、S 弯、急弯 R=0.25、直角弯 R=0.05。

    路径是开放的。直角弯放在急弯之后，并在两者之间留一段直道，
    这样急弯上的偏差和直角弯上的丢线可以分开看。
    """
    deg = np.pi / 180.0
    specs = [
        Segment("straight", "直道1", 1.20, 0.0),
        Segment("gentle", "缓弯", 0.80 * 90.0 * deg, 1.0 / 0.80),
        Segment("straight", "直道2", 0.70, 0.0),
        Segment("s", "S弯1", 0.60 * 45.0 * deg, -1.0 / 0.60),
        Segment("s", "S弯2", 0.60 * 90.0 * deg, 1.0 / 0.60),
        Segment("s", "S弯3", 0.60 * 45.0 * deg, -1.0 / 0.60),
        Segment("straight", "直道3", 0.55, 0.0),
        Segment("sharp", "急弯", 0.25 * 90.0 * deg, 1.0 / 0.25),
        Segment("straight", "直道4", 0.50, 0.0),
        # 半径 2.5 cm 的 90° 圆弧：曲率从 0 突变到 40 m⁻¹，用来表示胶带直角弯。
        # 4 cm 以上的圆角，这套运动学有时还能靠 1.5 倍 u 拐回来。
        Segment("corner", "直角弯", 0.025 * 90.0 * deg, 1.0 / 0.025),
        Segment("straight", "直道5", 2.90, 0.0),
    ]
    return ReferencePath(specs, ds=ds)
