# -*- coding: utf-8 -*-
"""误差曲线、轨迹对比、控制量和 GIF。

图上的急弯 / 直角弯色带与 ``simulate.time_spans`` 一致：
色带是车正在对应路段（直角弯再含前视）且还没明显离线的时间。
"""

import os

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

from core.car_model import sensor_world_positions
from config.params import CONFIG


def configure_font():
    """优先用微软雅黑。失败时调用方改用英文标签。"""
    candidates = [
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\msyhbd.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\simsun.ttc",
    ]
    for path in candidates:
        if not os.path.isfile(path):
            continue
        try:
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
            matplotlib.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            matplotlib.rcParams["font.family"] = "sans-serif"
            matplotlib.rcParams["axes.unicode_minus"] = False
            return True
        except (OSError, RuntimeError, ValueError):
            continue
    matplotlib.rcParams["axes.unicode_minus"] = False
    return False


def _labels(use_zh):
    if use_zh:
        return {
            "err_title": "传感器加权误差（增量式 PID 的输入）",
            "err_y": "error（传感器坐标）",
            "dist_title": "车轴到黑线中心的距离",
            "dist_y": "距离 / cm",
            "time": "时间 / s",
            "sharp": "急弯 R=0.25 m",
            "corner": "直角弯",
            "thr": "急弯切换阈值 ±2",
            "lost": "丢线，Σweight = 0",
            "traj_title": "参考路径与实际轨迹",
            "path": "参考路径",
            "actual": "实际轨迹",
            "start": "起点",
            "lost_pt": "开始丢线",
            "x": "x / m",
            "y": "y / m",
            "pwm_title": "左右轮 PWM",
            "pwm_y": "PWM",
            "left": "左轮",
            "right": "右轮",
            "u_title": "PID 输出 u_k 与送往电机的 u",
            "u_y": "控制量",
            "u_pid": "u_k（未乘 1.5）",
            "u_cmd": "u（急弯时 ×1.5 后再限幅）",
            "base_title": "基础速度 base_speed",
            "base_y": "base_speed",
            "gif_title": "增量式 PID 循迹仿真",
            "mode_sharp": "急弯模式",
            "mode_norm": "普通模式",
            "lost_txt": "丢线",
            "lost_body": "error 被置 0",
        }
    return {
        "err_title": "Sensor-weighted error (incremental PID input)",
        "err_y": "error (sensor units)",
        "dist_title": "Distance from axle to line center",
        "dist_y": "distance / cm",
        "time": "time / s",
        "sharp": "sharp turn R=0.25 m",
        "corner": "right-angle corner",
        "thr": "mode switch at ±2",
        "lost": "line lost, sum(weight)=0",
        "traj_title": "Reference path and actual trajectory",
        "path": "reference",
        "actual": "actual",
        "start": "start",
        "lost_pt": "line lost",
        "x": "x / m",
        "y": "y / m",
        "pwm_title": "Left / right PWM",
        "pwm_y": "PWM",
        "left": "left",
        "right": "right",
        "u_title": "PID output u_k and motor command u",
        "u_y": "command",
        "u_pid": "u_k (before x1.5)",
        "u_cmd": "u (x1.5 in sharp mode, then clipped)",
        "base_title": "Base speed",
        "base_y": "base_speed",
        "gif_title": "Incremental PID line tracking",
        "mode_sharp": "sharp mode",
        "mode_norm": "normal",
        "lost_txt": "line lost",
        "lost_body": "error forced to 0",
        }


def _shade(ax, spans, color, label):
    labeled = False
    for t0, t1 in spans:
        ax.axvspan(
            t0,
            t1,
            color=color,
            alpha=0.28,
            zorder=0,
            label=label if not labeled else None,
        )
        labeled = True


def _finite(result):
    return np.isfinite(result["error"])


def save_error_plot(result, metrics, path, labels):
    t = result["t"]
    err = result["error"]
    valid = _finite(result)
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 6.4), sharex=True, constrained_layout=True)

    ax = axes[0]
    _shade(ax, metrics["spans"]["sharp"], "#f4a261", labels["sharp"])
    _shade(ax, metrics["spans"]["corner"], "#e76f51", labels["corner"])
    ax.plot(t[valid], err[valid], color="#1d3557", lw=1.1, label="error")
    ax.axhline(2.0, color="#6c757d", ls="--", lw=0.9, label=labels["thr"])
    ax.axhline(-2.0, color="#6c757d", ls="--", lw=0.9)
    lost = valid & (result["sum_weight"] <= 0.0)
    if np.any(lost):
        ax.scatter(
            t[lost],
            err[lost],
            s=12,
            c="#c0392b",
            zorder=5,
            label=labels["lost"],
        )
    ax.set_ylabel(labels["err_y"])
    ax.set_title(labels["err_title"])
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8, ncol=2)

    ax = axes[1]
    _shade(ax, metrics["spans"]["sharp"], "#f4a261", labels["sharp"])
    _shade(ax, metrics["spans"]["corner"], "#e76f51", labels["corner"])
    ax.plot(t, result["dist"] * 100.0, color="#1d3557", lw=1.1)
    ax.set_xlabel(labels["time"])
    ax.set_ylabel(labels["dist_y"])
    ax.set_title(labels["dist_title"])
    ax.grid(True, alpha=0.3)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _amplify_index(result, metrics):
    """优先标直角弯上第一次把 u 乘出约 1.5 倍、且增量看得清的采样。

    S 弯上偶发会有单拍 work_n>=3，u 只有十几，乘 1.5 以后仍然很小。
    那一拍也是源码判据，但图上用来指认的是直角弯里持续约 0.2 s 的放大。
    """
    u_pid = result["u_pid"]
    u_cmd = result["u_cmd"]
    t = result["t"]

    def good(i):
        if not result["sharp"][i] or not np.isfinite(u_pid[i]) or abs(u_pid[i]) < 30.0:
            return False
        ratio = u_cmd[i] / u_pid[i]
        return 1.35 <= ratio <= 1.65

    spans = metrics.get("spans", {}).get("corner", [])
    if spans:
        t0, t1 = spans[0]
        for i, ti in enumerate(t):
            if t0 - 0.3 <= ti <= t1 and good(i):
                return i
    for i in range(len(t)):
        if good(i):
            return i
    return None


def save_control_plot(result, metrics, path, labels):
    t = result["t"]
    valid = _finite(result)
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 7.2), sharex=True, constrained_layout=True)
    for ax in axes:
        _shade(ax, metrics["spans"]["sharp"], "#f4a261", labels["sharp"])
        _shade(ax, metrics["spans"]["corner"], "#e76f51", labels["corner"])

    axes[0].step(t[valid], result["left_pwm"][valid], where="post", color="#1d3557", label=labels["left"])
    axes[0].step(t[valid], result["right_pwm"][valid], where="post", color="#e09f3e", label=labels["right"])
    axes[0].set_ylabel(labels["pwm_y"])
    axes[0].set_title(labels["pwm_title"])
    axes[0].set_ylim(-5, 270)
    axes[0].legend(loc="upper left", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t[valid], result["u_pid"][valid], color="#1d3557", lw=1.1, label=labels["u_pid"])
    axes[1].plot(t[valid], result["u_cmd"][valid], color="#c0392b", lw=1.1, label=labels["u_cmd"])
    axes[1].set_ylabel(labels["u_y"])
    axes[1].set_title(labels["u_title"])
    axes[1].legend(loc="upper left", fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].step(
        t[valid], result["base_speed"][valid], where="post", color="#1d3557", lw=1.4
    )
    axes[2].set_ylabel(labels["base_y"])
    axes[2].set_xlabel(labels["time"])
    axes[2].set_title(labels["base_title"])
    axes[2].set_ylim(40, 120)
    axes[2].grid(True, alpha=0.3)

    idx = _amplify_index(result, metrics)
    if idx is not None:
        xtext = float(t[idx]) - 6.2
        note = f"u {result['u_pid'][idx]:.1f} → {result['u_cmd'][idx]:.1f}（×1.5），base 60"
        axes[1].annotate(
            note,
            xy=(t[idx], result["u_cmd"][idx]),
            xytext=(xtext, 55),
            fontsize=8,
            color="#c0392b",
            arrowprops={"arrowstyle": "->", "color": "#c0392b", "lw": 0.8},
        )
        axes[2].annotate(
            "100 → 60",
            xy=(t[idx], 60),
            xytext=(xtext, 82),
            fontsize=8,
            color="#c0392b",
            arrowprops={"arrowstyle": "->", "color": "#c0392b", "lw": 0.8},
        )
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _car_triangle(x, y, theta, length=0.16, width=0.11):
    local = np.array(
        [
            [length * 0.62, 0.0],
            [-length * 0.38, width / 2.0],
            [-length * 0.38, -width / 2.0],
        ]
    )
    c, s = np.cos(theta), np.sin(theta)
    wx = x + local[:, 0] * c + local[:, 1] * (-s)
    wy = y + local[:, 0] * s + local[:, 1] * c
    return np.column_stack([wx, wy])


def save_trajectory_plot(result, path_file, labels):
    ref = result["path"]
    fig, ax = plt.subplots(figsize=(8.2, 7.2), constrained_layout=True)
    ax.plot(ref.x, ref.y, color="#b0b0b0", lw=5.0, solid_capstyle="round", label=labels["path"], zorder=1)
    labeled_sharp = False
    labeled_corner = False
    for seg in ref.segments:
        m = (ref.s >= seg.s0) & (ref.s <= seg.s1)
        if seg.kind == "sharp":
            ax.plot(
                ref.x[m],
                ref.y[m],
                color="#e67e22",
                lw=5.5,
                solid_capstyle="round",
                zorder=2,
                label=labels["sharp"] if not labeled_sharp else None,
            )
            labeled_sharp = True
        elif seg.kind == "corner":
            ax.plot(
                ref.x[m],
                ref.y[m],
                color="#c0392b",
                lw=6.0,
                solid_capstyle="round",
                zorder=2,
                label=labels["corner"] if not labeled_corner else None,
            )
            labeled_corner = True
            cx, cy, _, _ = ref.pose(seg.s0)
            ax.scatter([cx], [cy], c="#c0392b", s=28, zorder=3)
    ax.plot(
        result["x"],
        result["y"],
        color="#1d3557",
        lw=1.5,
        label=labels["actual"],
        zorder=4,
    )
    ax.scatter(result["x"][0], result["y"][0], c="#2a9d8f", s=36, zorder=5, label=labels["start"])
    lost = _finite(result) & (result["sum_weight"] <= 0.0)
    if np.any(lost):
        i = int(np.argmax(lost))
        ax.scatter(
            result["x"][i],
            result["y"][i],
            marker="x",
            c="#c0392b",
            s=48,
            zorder=6,
            label=labels["lost_pt"],
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(labels["x"])
    ax.set_ylabel(labels["y"])
    ax.set_title(labels["traj_title"])
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.savefig(path_file, dpi=140)
    plt.close(fig)


def save_animation(result, path_file, labels, cfg=None):
    cfg = CONFIG if cfg is None else cfg
    ref = result["path"]
    t = result["t"]
    stride = max(1, int(round(0.1 / float(cfg["dt"]))))
    frames = list(range(0, len(t), stride))
    if frames[-1] != len(t) - 1:
        frames.append(len(t) - 1)

    fig, ax = plt.subplots(figsize=(8.0, 6.6))
    xs = np.concatenate([ref.x, result["x"]])
    ys = np.concatenate([ref.y, result["y"]])
    pad = 0.28
    ax.set_xlim(float(xs.min()) - pad, float(xs.max()) + pad)
    ax.set_ylim(float(ys.min()) - pad, float(ys.max()) + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(labels["x"])
    ax.set_ylabel(labels["y"])
    ax.set_title(labels["gif_title"])
    ax.grid(True, alpha=0.25)
    ax.plot(ref.x, ref.y, color="#c8c8c8", lw=4.5, solid_capstyle="round", zorder=1)
    for seg in ref.segments:
        m = (ref.s >= seg.s0) & (ref.s <= seg.s1)
        if seg.kind == "sharp":
            ax.plot(ref.x[m], ref.y[m], color="#e67e22", lw=4.5, solid_capstyle="round", zorder=2)
        elif seg.kind == "corner":
            ax.plot(ref.x[m], ref.y[m], color="#c0392b", lw=5.0, solid_capstyle="round", zorder=2)

    (traj,) = ax.plot([], [], color="#1d3557", lw=1.5, zorder=3)
    (bar,) = ax.plot([], [], color="#333333", lw=1.0, zorder=4)
    poly = Polygon([[0, 0], [0, 0], [0, 0]], closed=True, fc="#1d3557", ec="black", lw=0.5, zorder=5)
    ax.add_patch(poly)
    scat = ax.scatter(
        [],
        [],
        c=[],
        cmap="inferno",
        vmin=0,
        vmax=8000,
        s=28,
        zorder=6,
    )
    text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"boxstyle": "round", "fc": "white", "alpha": 0.88, "ec": "#dddddd"},
    )

    def update(frame):
        x = float(result["x"][frame])
        y = float(result["y"][frame])
        theta = float(result["theta"][frame])
        traj.set_data(result["x"][: frame + 1], result["y"][: frame + 1])
        poly.set_xy(_car_triangle(x, y, theta))
        pts = sensor_world_positions(x, y, theta, cfg)
        bar.set_data([pts[0, 0], pts[-1, 0]], [pts[0, 1], pts[-1, 1]])
        analogs = result["analogs"][frame]
        if not np.all(np.isfinite(analogs)):
            analogs = np.zeros(8)
        scat.set_offsets(pts)
        scat.set_array(np.asarray(analogs, dtype=float))
        if np.isfinite(result["error"][frame]):
            mode = labels["mode_sharp"] if result["sharp"][frame] else labels["mode_norm"]
            lost = result["sum_weight"][frame] <= 0.0
            lost_s = f"  {labels['lost_txt']}" if lost else ""
            text.set_text(
                f"t = {result['t'][frame]:.2f} s\n"
                f"error = {result['error'][frame]:+.2f}{lost_s}\n"
                f"PWM L/R = {result['left_pwm'][frame]:.0f} / {result['right_pwm'][frame]:.0f}\n"
                f"base = {result['base_speed'][frame]:.0f}  {mode}"
            )
        else:
            text.set_text(
                f"t = {result['t'][frame]:.2f} s\n"
                f"{labels['lost_txt']}\n"
                f"{labels['lost_body']}"
            )
        return traj, poly, scat, text

    anim = FuncAnimation(fig, update, frames=frames, interval=100, blit=False)
    anim.save(path_file, writer=PillowWriter(fps=10), dpi=90)
    plt.close(fig)


def render_all(result, metrics, out_dir, cfg=None, use_zh=True):
    cfg = CONFIG if cfg is None else cfg
    labels = _labels(use_zh)
    save_error_plot(result, metrics, os.path.join(out_dir, "error_vs_time.png"), labels)
    save_trajectory_plot(result, os.path.join(out_dir, "trajectory_compare.png"), labels)
    save_control_plot(result, metrics, os.path.join(out_dir, "control_signals.png"), labels)
    save_animation(result, os.path.join(out_dir, "simulation.gif"), labels, cfg)
