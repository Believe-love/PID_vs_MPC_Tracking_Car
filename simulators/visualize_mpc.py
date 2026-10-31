# -*- coding: utf-8 -*-
"""MPC 专属图表与 GIF。坐标轴范围、路径线宽与颜色尽量与 PID 仿真一致。"""

import os

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

from core.car_model import sensor_world_positions
from config.params import CONFIG, MPC_CONFIG


def configure_font():
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
            "err_title": "横向误差 e_y（传感器换算到米）",
            "err_y": "e_y / m",
            "dist_title": "车轴到黑线中心的距离",
            "dist_y": "距离 / cm",
            "time": "时间 / s",
            "sharp": "急弯 R=0.25 m",
            "corner": "直角弯",
            "thr": "软约束 ±e_y_max",
            "lost": "丢线，Σweight = 0",
            "traj_title": "参考路径与 MPC 实际轨迹",
            "path": "参考路径",
            "actual": "MPC 轨迹",
            "start": "起点",
            "lost_pt": "开始丢线",
            "x": "x / m",
            "y": "y / m",
            "pwm_title": "左右轮 PWM",
            "pwm_y": "PWM",
            "left": "左轮",
            "right": "右轮",
            "omega_title": "角速度：omega_cmd / omega_r / delta_u",
            "omega_y": "rad/s",
            "omega_cmd": "omega_cmd",
            "omega_r": "omega_r",
            "delta_u": "delta_u",
            "kappa_mark": "参考曲率开始增大",
            "gif_title": "线性 MPC 循迹仿真",
            "mode_sharp": "低速参考段",
            "mode_norm": "高速参考段",
            "lost_txt": "丢线",
            "lost_body": "error 被置 0",
        }
    return {
        "err_title": "Lateral error e_y (sensor units → m)",
        "err_y": "e_y / m",
        "dist_title": "Distance from axle to line center",
        "dist_y": "distance / cm",
        "time": "time / s",
        "sharp": "sharp turn R=0.25 m",
        "corner": "right-angle corner",
        "thr": "soft bound ±e_y_max",
        "lost": "line lost, sum(weight)=0",
        "traj_title": "Reference path vs MPC trajectory",
        "path": "reference",
        "actual": "MPC",
        "start": "start",
        "lost_pt": "line lost",
        "x": "x / m",
        "y": "y / m",
        "pwm_title": "Left / right PWM",
        "pwm_y": "PWM",
        "left": "left",
        "right": "right",
        "omega_title": "omega_cmd / omega_r / delta_u",
        "omega_y": "rad/s",
        "omega_cmd": "omega_cmd",
        "omega_r": "omega_r",
        "delta_u": "delta_u",
        "kappa_mark": "curvature rise",
        "gif_title": "Linear MPC line tracking",
        "mode_sharp": "low v_r",
        "mode_norm": "high v_r",
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


def save_error_plot(result, metrics, path, labels, mpc_cfg=None):
    mpc_cfg = MPC_CONFIG if mpc_cfg is None else mpc_cfg
    t = result["t"]
    e_y = result["e_y"]
    valid = _finite(result)
    e_y_max = float(metrics.get("e_y_max", mpc_cfg["e_y_max"]))
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 6.4), sharex=True, constrained_layout=True)

    ax = axes[0]
    _shade(ax, metrics["spans"]["sharp"], "#f4a261", labels["sharp"])
    _shade(ax, metrics["spans"]["corner"], "#e76f51", labels["corner"])
    ax.plot(t[valid], e_y[valid], color="#1d3557", lw=1.1, label="e_y")
    ax.axhline(e_y_max, color="#6c757d", ls="--", lw=0.9, label=labels["thr"])
    ax.axhline(-e_y_max, color="#6c757d", ls="--", lw=0.9)
    lost = valid & (result["sum_weight"] <= 0.0)
    if np.any(lost):
        ax.scatter(
            t[lost],
            e_y[lost],
            s=12,
            c="#c0392b",
            zorder=5,
            label=labels["lost"],
        )
    ax.set_ylabel(labels["err_y"])
    ax.set_title(labels["err_title"])
    # 与 PID 误差图量级对照：传感器 ±4 单位 ≈ ±0.0686 m，略放宽
    ax.set_ylim(-0.08, 0.08)
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


def save_control_plot(result, metrics, path, labels):
    t = result["t"]
    valid = _finite(result)
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 7.8), sharex=True, constrained_layout=True)
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

    axes[1].plot(t[valid], result["omega_cmd"][valid], color="#1d3557", lw=1.1, label=labels["omega_cmd"])
    axes[1].plot(t[valid], result["omega_r"][valid], color="#2a9d8f", lw=1.1, ls="--", label=labels["omega_r"])
    axes[1].plot(t[valid], result["delta_u"][valid], color="#c0392b", lw=1.0, alpha=0.85, label=labels["delta_u"])
    axes[1].set_ylabel(labels["omega_y"])
    axes[1].set_title(labels["omega_title"])
    axes[1].legend(loc="upper left", fontsize=8, ncol=3)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t[valid], result["kappa_r"][valid], color="#1d3557", lw=1.2)
    axes[2].set_ylabel("kappa_r / 1/m")
    axes[2].set_xlabel(labels["time"])
    axes[2].set_title("参考路径曲率 kappa_r")
    axes[2].grid(True, alpha=0.3)

    # 标出急弯曲率上升时刻，检查预打角
    t_rise = metrics.get("curvature_rise_t_sharp")
    if t_rise is not None:
        for ax in axes:
            ax.axvline(t_rise, color="#9b59b6", ls=":", lw=1.2, zorder=3)
        axes[1].annotate(
            labels["kappa_mark"],
            xy=(t_rise, 0.0),
            xytext=(t_rise - 4.5, axes[1].get_ylim()[1] * 0.55 if axes[1].get_ylim()[1] != 0 else 1.0),
            fontsize=8,
            color="#9b59b6",
            arrowprops={"arrowstyle": "->", "color": "#9b59b6", "lw": 0.8},
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
    scat = ax.scatter([], [], c=[], cmap="inferno", vmin=0, vmax=8000, s=28, zorder=6)
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
                f"e_y = {result['e_y'][frame]:+.4f} m{lost_s}\n"
                f"e_θ = {result['e_theta'][frame]:+.3f} rad\n"
                f"ω_cmd = {result['omega_cmd'][frame]:+.3f}\n"
                f"PWM L/R = {result['left_pwm'][frame]:.0f} / {result['right_pwm'][frame]:.0f}\n"
                f"{mode}"
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
    save_error_plot(result, metrics, os.path.join(out_dir, "mpc_error_vs_time.png"), labels)
    save_trajectory_plot(result, os.path.join(out_dir, "mpc_trajectory.png"), labels)
    save_control_plot(result, metrics, os.path.join(out_dir, "mpc_control_signals.png"), labels)
    save_animation(result, os.path.join(out_dir, "mpc_simulation.gif"), labels, cfg)
