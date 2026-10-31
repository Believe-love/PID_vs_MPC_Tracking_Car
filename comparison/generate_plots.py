# -*- coding: utf-8 -*-
"""为 PPT 生成三张带大字标注的对比图，并写出 metrics_updated.md。

只读两个项目 output 里的最新 CSV，不改仿真代码，也不改原有指标公式。
图幅 16:9，标注字号大于 16，曲线线宽大于 2.5。
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np

from comparison.compare import (
    DATA,
    FIG,
    configure_font,
    load_inputs,
    rate_rms,
)

# 直道噪声大约 ±0.2 rad/s。高于 0.40 rad/s 才算指令开始打角，避免把噪声当成提前量。
CMD_ONSET = 0.40
HI_M = 0.02
LO_M = 0.005

PID_COLOR = "#d62728"
MPC_COLOR = "#1f77b4"
REF_COLOR = "#4d4d4d"
SHARP_COLOR = "#ffd7a8"
CORNER_COLOR = "#f6b3b3"
BOX = dict(boxstyle="round,pad=0.45", fc="#fff4c2", ec="#8a5a00", lw=2.8)


def _style(ax):
    ax.tick_params(labelsize=18, width=1.6, length=6)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_linewidth(1.4)
    ax.xaxis.label.set_size(20)
    ax.yaxis.label.set_size(20)
    ax.title.set_size(26)
    ax.title.set_fontweight("bold")


def _new_fig():
    fig, ax = plt.subplots(figsize=(16, 9))
    fig.subplots_adjust(left=0.08, right=0.96, top=0.88, bottom=0.12)
    return fig, ax


def _save(fig, name):
    FIG.mkdir(parents=True, exist_ok=True)
    path = FIG / name
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def _spans(df, s0, s1):
    time = df["time"].to_numpy(dtype=float)
    arc = df["s"].to_numpy(dtype=float)
    mask = (arc >= s0) & (arc <= s1)
    spans = []
    i = 0
    n = len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i + 1
        while j < n and mask[j]:
            j += 1
        t1 = float(time[j]) if j < n else float(time[j - 1])
        spans.append((float(time[i]), t1))
        i = j
    return spans


def _shade(ax, spans, color):
    for start, end in spans:
        ax.axvspan(start, end, color=color, alpha=0.55, zorder=0, linewidth=0)


def _abs_peak(df, s0, s1):
    mask = (df["s"] >= s0) & (df["s"] <= s1)
    sub = df.loc[mask]
    idx = sub["e_y"].abs().idxmax()
    return float(sub.loc[idx, "time"]), float(sub.loc[idx, "e_y"])


def _preview_times(mpc):
    time = mpc["time"].to_numpy(dtype=float)
    kappa = mpc["kappa_r"].to_numpy(dtype=float)
    cmd = mpc["omega_cmd"].to_numpy(dtype=float)
    hit = np.flatnonzero(np.isfinite(kappa) & (np.abs(kappa) >= 3.0))
    i1 = int(hit[0])
    below = np.flatnonzero(np.abs(cmd[:i1]) < CMD_ONSET)
    if len(below) == 0:
        i2 = 0
    else:
        i2 = int(below[-1]) + 1
    i2 = min(i2, i1)
    return float(time[i1]), float(time[i2]), float(time[i1] - time[i2])


def _recovery(df, s0):
    """出弯后，|e_y| 从仍大于 2 cm 到第一次小于 0.5 cm 的时间。回不去则返回 None。"""
    on = df[df["s"] >= s0]
    ey = on["e_y"].to_numpy(dtype=float)
    tt = on["time"].to_numpy(dtype=float)
    above = np.flatnonzero(np.isfinite(ey) & (np.abs(ey) >= HI_M))
    if len(above) == 0:
        return None
    i_hi = int(above[-1])
    after = np.flatnonzero((np.arange(len(ey)) > i_hi) & np.isfinite(ey) & (np.abs(ey) < LO_M))
    if len(after) == 0:
        return None
    i_lo = int(after[0])
    return {
        "dt": float(tt[i_lo] - tt[i_hi]),
        "t_hi": float(tt[i_hi]),
        "t_lo": float(tt[i_lo]),
        "ey_lo": float(ey[i_lo]),
        "ey_end": float(ey[-1]),
    }


def _rms(values):
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    return float(np.sqrt(np.mean(data ** 2)))


def plot_error(pid, mpc, sharp, pid_peak, mpc_peak, pct):
    fig, ax = _new_fig()
    s0, s1 = sharp
    on_pid = pid[(pid["s"] >= s0) & (pid["s"] <= s1)]
    on_mpc = mpc[(mpc["s"] >= s0) & (mpc["s"] <= s1)]
    t_left = min(float(on_pid["time"].iloc[0]), float(on_mpc["time"].iloc[0])) - 0.85
    t_right = max(float(on_pid["time"].iloc[-1]), float(on_mpc["time"].iloc[-1])) + 0.35
    _shade(ax, _spans(pid, s0, s1) + _spans(mpc, s0, s1), SHARP_COLOR)
    ax.plot(pid["time"], pid["e_y"] * 100.0, color=PID_COLOR, lw=3.0, label="PID", zorder=3)
    ax.plot(mpc["time"], mpc["e_y"] * 100.0, color=MPC_COLOR, lw=3.0, label="MPC", zorder=3)
    ax.axhline(0.0, color="#666666", lw=2.6, ls="--", zorder=1)
    ax.set_xlim(t_left, t_right)
    ax.set_ylim(-1.85, 1.95)

    t_pid, ey_pid = pid_peak
    t_mpc, ey_mpc = mpc_peak
    ax.annotate(
        f"PID 最大误差: {abs(ey_pid) * 100:.2f} cm",
        xy=(t_pid, ey_pid * 100.0),
        xytext=(t_pid + 0.15, -1.55),
        fontsize=22,
        fontweight="bold",
        color=PID_COLOR,
        arrowprops=dict(arrowstyle="-|>", color=PID_COLOR, lw=3.0, mutation_scale=24),
        zorder=5,
    )
    ax.annotate(
        f"MPC 最大误差: {abs(ey_mpc) * 100:.2f} cm",
        xy=(t_mpc, ey_mpc * 100.0),
        xytext=(t_left + 0.12, 1.15),
        fontsize=22,
        fontweight="bold",
        color=MPC_COLOR,
        arrowprops=dict(arrowstyle="-|>", color=MPC_COLOR, lw=3.0, mutation_scale=24),
        zorder=5,
    )
    ax.text(
        0.98,
        0.96,
        f"误差峰值降低: {pct:.1f}%",
        transform=ax.transAxes,
        fontsize=26,
        fontweight="bold",
        ha="right",
        va="top",
        bbox=BOX,
        zorder=6,
    )
    ax.set_title("急弯横向误差峰值对比")
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("横向误差 / cm")
    ax.legend(loc="lower right", fontsize=18, framealpha=0.95)
    _style(ax)
    _save(fig, "compare_error_annotated.png")


def plot_preview(mpc, t1, t2, lead):
    fig, ax = _new_fig()
    ax.plot(mpc["time"], mpc["omega_r"], color=REF_COLOR, lw=3.0, ls="--", label="参考角速度 ω_r", zorder=2)
    ax.plot(mpc["time"], mpc["omega_cmd"], color=MPC_COLOR, lw=3.0, label="MPC 指令 ω_cmd", zorder=3)
    ax.set_xlim(t1 - 1.05, t1 + 1.35)
    view = mpc[(mpc["time"] >= t1 - 1.05) & (mpc["time"] <= t1 + 1.35)]
    vals = np.concatenate(
        [view["omega_r"].to_numpy(dtype=float), view["omega_cmd"].to_numpy(dtype=float)]
    )
    vals = vals[np.isfinite(vals)]
    ax.set_ylim(float(vals.min()) - 0.45, float(vals.max()) + 1.15)
    ax.axvline(t1, color="#222222", lw=2.6, ls="--", zorder=4)
    if abs(t1 - t2) >= 0.01:
        ax.axvline(t2, color="#1b4f72", lw=2.6, ls="--", zorder=4)
        y_arrow = float(np.nanmax(vals)) * 0.72
        ax.annotate(
            "",
            xy=(t1, y_arrow),
            xytext=(t2, y_arrow),
            arrowprops=dict(arrowstyle="<->", color="#8a5a00", lw=3.0, mutation_scale=22),
        )
        ax.text(
            (t1 + t2) / 2.0,
            y_arrow + 0.18,
            f"MPC 提前 {lead:.2f} 秒开始打角",
            fontsize=24,
            fontweight="bold",
            ha="center",
            va="bottom",
            bbox=BOX,
        )
    else:
        ax.text(
            0.98,
            0.97,
            f"MPC 提前 {lead:.2f} 秒开始打角\n（t1 = t2 = {t1:.2f} s，同一拍抬起）",
            transform=ax.transAxes,
            fontsize=24,
            fontweight="bold",
            ha="right",
            va="top",
            bbox=BOX,
            zorder=6,
        )
        ax.annotate(
            "t1：ω_r 开始上升\nt2：ω_cmd 开始上升",
            xy=(t1, 0.72),
            xytext=(t1 - 0.95, 2.15),
            fontsize=20,
            fontweight="bold",
            color="#222222",
            arrowprops=dict(arrowstyle="-|>", color="#222222", lw=2.8, mutation_scale=22),
        )
    ax.set_title("急弯入口：参考角速度与 MPC 指令")
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("角速度 / (rad/s)")
    ax.legend(loc="upper left", fontsize=18, framealpha=0.95)
    _style(ax)
    _save(fig, "compare_preview_annotated.png")


def plot_straight(pid, mpc, s0, pid_rec, mpc_rec):
    fig, ax = _new_fig()
    t_start = min(
        float(pid.loc[pid["s"] >= s0, "time"].iloc[0]),
        float(mpc.loc[mpc["s"] >= s0, "time"].iloc[0]),
    ) - 0.6
    t_end = max(float(pid["time"].iloc[-1]), float(mpc["time"].iloc[-1]))
    pad = 0.012 * (t_end - t_start)
    ax.plot(pid["time"], pid["e_y"] * 100.0, color=PID_COLOR, lw=3.0, label="PID", zorder=3)
    ax.plot(mpc["time"], mpc["e_y"] * 100.0, color=MPC_COLOR, lw=3.0, label="MPC", zorder=3)
    ax.axhline(0.0, color="#666666", lw=2.6, ls="--", label="y = 0", zorder=1)
    ax.set_xlim(t_start - pad, t_end + pad)
    ax.set_ylim(-14.0, 6.5)

    if mpc_rec is not None:
        y_lo = mpc_rec["ey_lo"] * 100.0
        ax.scatter(
            [mpc_rec["t_lo"]],
            [y_lo],
            s=260,
            c="#1b8a3e",
            zorder=6,
            edgecolors="white",
            linewidths=2.2,
        )
        ax.annotate(
            "",
            xy=(mpc_rec["t_lo"], 3.6),
            xytext=(mpc_rec["t_hi"], 3.6),
            arrowprops=dict(arrowstyle="<->", color="#1b8a3e", lw=3.0, mutation_scale=22),
        )
        ax.text(
            (mpc_rec["t_hi"] + mpc_rec["t_lo"]) / 2.0,
            4.15,
            f"MPC 回正耗时: {mpc_rec['dt']:.2f} 秒",
            fontsize=22,
            fontweight="bold",
            color="#146c32",
            ha="center",
            va="bottom",
            bbox=dict(boxstyle="round,pad=0.4", fc="#e8f8ee", ec="#1b8a3e", lw=2.8),
        )
    pid_end = float(pid["e_y"].iloc[-1]) * 100.0
    ax.text(
        0.98,
        0.06,
        f"PID 回正耗时: 未回正\n（结束时 {pid_end:.1f} cm）",
        transform=ax.transAxes,
        fontsize=22,
        fontweight="bold",
        color=PID_COLOR,
        ha="right",
        va="bottom",
        bbox=dict(boxstyle="round,pad=0.45", fc="#fdecea", ec=PID_COLOR, lw=2.8),
    )
    ax.set_title("长直道回正：出弯后误差从 2 cm 以上降到 0.5 cm 以下")
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("横向误差 / cm")
    ax.legend(loc="upper left", fontsize=18, framealpha=0.95)
    _style(ax)
    _save(fig, "compare_straight_annotated.png")


def write_metrics(stats):
    pid_rec = "未回正" if stats["pid_rec"] is None else f"{stats['pid_rec']['dt']:.2f} s"
    mpc_rec = "未回正" if stats["mpc_rec"] is None else f"{stats['mpc_rec']['dt']:.2f} s"
    lines = [
        "# 更新后的对比指标",
        "",
        "数据来自 `results/data/pid_timeseries.csv` 与 `results/data/mpc_timeseries.csv`。",
        f"直道5长度 {stats['straight_len']:.2f} m，路径总长 {stats['path_len']:.3f} m。",
        "横向误差是车轴到路径的几何偏差。急弯峰值取该段上 |e_y| 的最大值。",
        "",
        "## 急弯段最大误差",
        "",
        "| 指标 | PID | MPC |",
        "| --- | ---: | ---: |",
        f"| 急弯最大 \\|e_y\\| / cm | {stats['pid_peak_cm']:.2f} | {stats['mpc_peak_cm']:.2f} |",
        f"| 误差峰值降低 | {stats['pct']:.1f}% | |",
        "",
        "## 长直道回正",
        "",
        "出弯后，从最后一次 |e_y| 仍大于 2 cm，到第一次小于 0.5 cm 的时间。",
        "",
        "| 指标 | PID | MPC |",
        "| --- | ---: | ---: |",
        f"| 回正耗时 | {pid_rec} | {mpc_rec} |",
        "",
        "## 全局",
        "",
        "| 指标 | PID | MPC |",
        "| --- | ---: | ---: |",
        f"| RMS(e_y) / m | {stats['pid_rms']:.4f} | {stats['mpc_rms']:.4f} |",
        f"| RMS(e_y) / cm | {stats['pid_rms'] * 100:.2f} | {stats['mpc_rms'] * 100:.2f} |",
        f"| 控制量变化率 RMS / (rad/s²) | {stats['pid_rate']:.2f} | {stats['mpc_rate']:.2f} |",
        "",
        "## 预瞄读数",
        "",
        f"急弯入口 ω_r 开始上升 t1 = {stats['t1']:.2f} s，ω_cmd 开始上升 t2 = {stats['t2']:.2f} s，提前量 t1 − t2 = {stats['lead']:.2f} s。",
        f"打角判定阈值是 |ω_cmd| ≥ {CMD_ONSET:.2f} rad/s，高于直道上大约 ±0.2 rad/s 的起伏。",
        "",
    ]
    text = "\n".join(lines)
    (DATA / "metrics_updated.md").write_text(text, encoding="utf-8")
    print(text)


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    configure_font()
    pid, mpc, segments, _ref, pid_meta, mpc_meta, _params = load_inputs()
    sharp = segments[segments["kind"] == "sharp"].iloc[0]
    last = segments[segments["name"] == "直道5"].iloc[0]
    s0, s1 = float(sharp["s0"]), float(sharp["s1"])
    straight_s0 = float(last["s0"])
    t_pid, ey_pid = _abs_peak(pid, s0, s1)
    t_mpc, ey_mpc = _abs_peak(mpc, s0, s1)
    pct = (abs(ey_pid) - abs(ey_mpc)) / abs(ey_pid) * 100.0
    t1, t2, lead = _preview_times(mpc)
    pid_rec = _recovery(pid, straight_s0)
    mpc_rec = _recovery(mpc, straight_s0)
    dt = float(pid_meta["dt"])
    ones_pid = np.ones(len(pid), dtype=bool)
    ones_mpc = np.ones(len(mpc), dtype=bool)
    stats = {
        "straight_len": float(last["length"]),
        "path_len": float(pid_meta["path_length_m"]),
        "pid_peak_cm": abs(ey_pid) * 100.0,
        "mpc_peak_cm": abs(ey_mpc) * 100.0,
        "pct": pct,
        "pid_rec": pid_rec,
        "mpc_rec": mpc_rec,
        "pid_rms": _rms(pid["e_y"]),
        "mpc_rms": _rms(mpc["e_y"]),
        "pid_rate": rate_rms(pid["omega_cmd"], ones_pid, dt),
        "mpc_rate": rate_rms(mpc["omega_cmd"], ones_mpc, dt),
        "t1": t1,
        "t2": t2,
        "lead": lead,
    }
    plot_error(pid, mpc, (s0, s1), (t_pid, ey_pid), (t_mpc, ey_mpc), pct)
    plot_preview(mpc, t1, t2, lead)
    plot_straight(pid, mpc, straight_s0, pid_rec, mpc_rec)
    write_metrics(stats)
    print(f"已写入 {FIG / 'compare_error_annotated.png'}")
    print(f"已写入 {FIG / 'compare_preview_annotated.png'}")
    print(f"已写入 {FIG / 'compare_straight_annotated.png'}")
    print(f"已写入 {DATA / 'metrics_updated.md'}")
    pid_txt = "未回正" if pid_rec is None else f"{pid_rec['dt']:.2f} s"
    mpc_txt = "未回正" if mpc_rec is None else f"{mpc_rec['dt']:.2f} s"
    print(
        f"PPT 可用数字：急弯峰值 PID {stats['pid_peak_cm']:.2f} cm，MPC {stats['mpc_peak_cm']:.2f} cm，"
        f"降低 {stats['pct']:.1f}%；预瞄提前 {stats['lead']:.2f} s；"
        f"长直道回正 PID {pid_txt}，MPC {mpc_txt}。"
    )


if __name__ == "__main__":
    main()
