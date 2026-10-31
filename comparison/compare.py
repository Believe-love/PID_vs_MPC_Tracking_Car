# -*- coding: utf-8 -*-
"""读取 PID / MPC 时序 CSV，生成轨迹、误差、指令、预瞄四张图和指标表。

指标公式与原 comparison_report/generate_report.py 相同。
原始时序来自 results/data，图写到 results/figures。
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

DATA = ROOT / "results" / "data"
FIG = ROOT / "results" / "figures"
PID_CSV = DATA / "pid_timeseries.csv"
MPC_CSV = DATA / "mpc_timeseries.csv"

PID_COLOR = "#d62728"
MPC_COLOR = "#1f77b4"
REF_COLOR = "#6e6e6e"
SHARP_COLOR = "#ffd7a8"
CORNER_COLOR = "#f6b3b3"

# 与仿真配置一致：传感器杆在轴心前方 7.5 cm，阵列总宽 12 cm。
LOOKAHEAD = 0.075
SENSOR_HALF = 0.06
LOSS_STEPS = 3
EY_ZOOM = 0.12
# 直道上指令起伏大约 ±0.2 rad/s。超过 0.5 rad/s 才算一次明确的提前打角。
RISE_LEVEL = 0.50
RISE_LOOKBACK = 0.55


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
            return name
        except (OSError, RuntimeError, ValueError):
            continue
    matplotlib.rcParams["axes.unicode_minus"] = False
    return None


def apply_style():
    matplotlib.rcParams["axes.grid"] = True
    matplotlib.rcParams["grid.alpha"] = 0.35
    matplotlib.rcParams["grid.linestyle"] = "--"
    matplotlib.rcParams["figure.facecolor"] = "white"
    matplotlib.rcParams["axes.facecolor"] = "white"
    matplotlib.rcParams["legend.framealpha"] = 0.92
    matplotlib.rcParams["axes.axisbelow"] = True


def _wrap(angle):
    return (np.asarray(angle, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


def _pwm_to_wheel_speed(pwm, cfg):
    if not np.isfinite(pwm):
        return np.nan
    if float(pwm) < float(cfg["pwm_deadzone"]):
        return 0.0
    return float(cfg["speed_at_pwm_100"]) / 100.0 * float(pwm)


def _body_omega(v_left, v_right, cfg):
    return (float(v_right) - float(v_left)) / float(cfg["wheel_base"])


def _sensor_scale(cfg):
    sensor_x = cfg["sensor_x"]
    return float(cfg["sensor_span"]) / float(sensor_x[-1] - sensor_x[0])


def _commanded_omega(left_pwm, right_pwm, cfg):
    out = np.empty(len(left_pwm), dtype=float)
    for i, (left, right) in enumerate(zip(left_pwm, right_pwm)):
        if not np.isfinite(left) or not np.isfinite(right):
            out[i] = np.nan
            continue
        v_left = _pwm_to_wheel_speed(float(left), cfg)
        v_right = _pwm_to_wheel_speed(float(right), cfg)
        out[i] = _body_omega(v_left, v_right, cfg)
    return out


def _geometry(ref, arc):
    heading = np.empty(len(arc), dtype=float)
    kappa = np.empty(len(arc), dtype=float)
    kinds = []
    for i, s in enumerate(np.asarray(arc, dtype=float)):
        _, _, heading[i], kappa[i] = ref.pose(float(s))
        kinds.append(ref.segments[ref.segment_index_at(float(s))].kind)
    return heading, kappa, kinds


def _reference_omega(kappa, kinds, cfg):
    v_high = float(cfg["speed_at_pwm_100"])
    v_low = v_high / 100.0 * float(cfg["base_low_pwm"])
    v_r = np.where(np.isin(np.asarray(kinds), ["sharp", "corner"]), v_low, v_high).astype(float)
    return np.asarray(kappa, dtype=float) * v_r


def _to_report_frame(raw, ref, cfg, omega_cmd=None, kappa=None, omega_r=None):
    heading, kappa_pose, kinds = _geometry(ref, raw["s"])
    if kappa is None:
        kappa = kappa_pose
    if omega_r is None:
        omega_r = _reference_omega(kappa_pose, kinds, cfg)
    if omega_cmd is None:
        omega_cmd = _commanded_omega(raw["left_pwm"], raw["right_pwm"], cfg)
    scale = _sensor_scale(cfg)
    error = pd.to_numeric(raw["error"], errors="coerce").to_numpy(dtype=float)
    return pd.DataFrame(
        {
            "time": pd.to_numeric(raw["t"], errors="coerce").to_numpy(dtype=float),
            "x": pd.to_numeric(raw["x"], errors="coerce").to_numpy(dtype=float),
            "y": pd.to_numeric(raw["y"], errors="coerce").to_numpy(dtype=float),
            "theta": pd.to_numeric(raw["theta"], errors="coerce").to_numpy(dtype=float),
            "e_y": pd.to_numeric(raw["lat"], errors="coerce").to_numpy(dtype=float),
            "e_theta": _wrap(pd.to_numeric(raw["theta"], errors="coerce").to_numpy(dtype=float) - heading),
            "v": pd.to_numeric(raw["v"], errors="coerce").to_numpy(dtype=float),
            "omega_cmd": np.asarray(omega_cmd, dtype=float),
            "kappa_r": np.asarray(kappa, dtype=float),
            "s": pd.to_numeric(raw["s"], errors="coerce").to_numpy(dtype=float),
            "segment": kinds,
            "dist": pd.to_numeric(raw["dist"], errors="coerce").to_numpy(dtype=float),
            "sum_weight": pd.to_numeric(raw["sum_weight"], errors="coerce").to_numpy(dtype=float),
            "omega": pd.to_numeric(raw["omega"], errors="coerce").to_numpy(dtype=float),
            "omega_r": np.asarray(omega_r, dtype=float),
            "e_y_sensor": error * scale,
        }
    )


def _meta(name, df, ref, cfg):
    end_margin = float(cfg["sensor_lookahead"]) + 0.08
    reached = float(df["s"].iloc[-1]) >= float(ref.total_length) - end_margin and float(df["dist"].iloc[-1]) < 0.08
    return {
        "controller": name,
        "seed": int(cfg["seed"]),
        "dt": float(cfg["dt"]),
        "steps": int(len(df)),
        "duration_s": float(df["time"].iloc[-1]),
        "path_length_m": float(ref.total_length),
        "stopped_off_line": int(not reached),
        "initial_lateral_offset": float(cfg["initial_lateral_offset"]),
        "initial_heading_error": float(cfg["initial_heading_error"]),
        "sensor_lookahead": float(cfg["sensor_lookahead"]),
        "sensor_span": float(cfg["sensor_span"]),
    }


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (np.floating, float)):
        return float(value)
    if isinstance(value, (np.integer, int)) and not isinstance(value, bool):
        return int(value)
    if isinstance(value, (np.bool_, bool)) or value is None:
        return bool(value) if value is not None else None
    return value


def load_inputs():
    """读取仿真导出的原始时序，并在 results/data 写下派生表。

    原始 pid_timeseries.csv / mpc_timeseries.csv 保留给重复作图，派生表另存。
    """
    from config.params import CONFIG, MPC_CONFIG
    from core.path import build_demo_path

    if not PID_CSV.is_file() or not MPC_CSV.is_file():
        raise SystemExit(f"缺少最新时序：{PID_CSV} 或 {MPC_CSV}")
    pid_ref = build_demo_path()
    mpc_ref = build_demo_path()
    pid_seg = pd.DataFrame(pid_ref.describe())
    mpc_seg = pd.DataFrame(mpc_ref.describe())
    if not pid_seg.equals(mpc_seg):
        raise SystemExit("两边的参考路径分段不一致，停止作图。")
    pid_raw = pd.read_csv(PID_CSV)
    mpc_raw = pd.read_csv(MPC_CSV)
    pid = _to_report_frame(pid_raw, pid_ref, CONFIG)
    mpc = _to_report_frame(
        mpc_raw,
        mpc_ref,
        CONFIG,
        omega_cmd=pd.to_numeric(mpc_raw["omega_cmd"], errors="coerce").to_numpy(dtype=float),
        kappa=pd.to_numeric(mpc_raw["kappa_r"], errors="coerce").to_numpy(dtype=float),
        omega_r=pd.to_numeric(mpc_raw["omega_r"], errors="coerce").to_numpy(dtype=float),
    )
    ref = pd.DataFrame(
        {
            "s": pid_ref.s,
            "x": pid_ref.x,
            "y": pid_ref.y,
            "theta": pid_ref.heading,
            "kappa": pid_ref.curvature,
        }
    )
    pid_meta = _meta("pid", pid, pid_ref, CONFIG)
    mpc_meta = _meta("mpc", mpc, mpc_ref, CONFIG)
    for key in ("seed", "dt", "path_length_m", "initial_lateral_offset"):
        if pid_meta[key] != mpc_meta[key]:
            raise SystemExit(f"仿真条件不一致：{key}")
    for col in ("time", "x", "y", "theta"):
        if abs(float(pid.iloc[0][col]) - float(mpc.iloc[0][col])) > 1e-6:
            raise SystemExit(f"初始 {col} 不一致")
    params = {
        "N": MPC_CONFIG["N"],
        "Q": list(MPC_CONFIG["Q"]),
        "R": MPC_CONFIG["R"],
        "S": MPC_CONFIG["S"],
        "P": list(MPC_CONFIG["P"]),
        "v_high": MPC_CONFIG["v_high"],
        "v_low": MPC_CONFIG["v_low"],
        "omega_min": MPC_CONFIG["omega_min"],
        "omega_max": MPC_CONFIG["omega_max"],
        "freeze_linearization": MPC_CONFIG["freeze_linearization"],
        "e_theta_noise_std": MPC_CONFIG["e_theta_noise_std"],
    }
    DATA.mkdir(parents=True, exist_ok=True)
    pid.to_csv(DATA / "pid_frame.csv", index=False, encoding="utf-8")
    mpc.to_csv(DATA / "mpc_frame.csv", index=False, encoding="utf-8")
    pid_seg.to_csv(DATA / "pid_path_segments.csv", index=False, encoding="utf-8")
    mpc_seg.to_csv(DATA / "mpc_path_segments.csv", index=False, encoding="utf-8")
    ref.to_csv(DATA / "reference_path.csv", index=False, encoding="utf-8")
    (DATA / "pid_meta.json").write_text(json.dumps(pid_meta, ensure_ascii=False, indent=2), encoding="utf-8")
    (DATA / "mpc_meta.json").write_text(json.dumps(mpc_meta, ensure_ascii=False, indent=2), encoding="utf-8")
    (DATA / "mpc_controller_params.json").write_text(
        json.dumps(_jsonable(params), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(
        f"已读取原始时序：PID {pid_meta['steps']} 步 / {pid_meta['duration_s']:.2f} s，"
        f"MPC {mpc_meta['steps']} 步 / {mpc_meta['duration_s']:.2f} s，"
        f"路径 {pid_meta['path_length_m']:.3f} m"
    )
    return pid, mpc, pid_seg, ref, pid_meta, mpc_meta, _jsonable(params)


def rms(values):
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    if len(data) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(data ** 2)))


def max_abs(values):
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    if len(data) == 0:
        return float("nan")
    return float(np.max(np.abs(data)))


def mask_to_spans(time, mask):
    time = np.asarray(time, dtype=float)
    mask = np.asarray(mask, dtype=bool)
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
        if t1 <= float(time[i]):
            t1 = float(time[i]) + 0.02
        spans.append((float(time[i]), t1))
        i = j
    return spans


def union_spans(groups):
    spans = [span for group in groups for span in group]
    if not spans:
        return []
    spans = sorted(spans)
    merged = [list(spans[0])]
    for start, end in spans[1:]:
        if start <= merged[-1][1] + 1e-9:
            merged[-1][1] = max(merged[-1][1], end)
        else:
            merged.append([start, end])
    return [(a, b) for a, b in merged]


def shade(ax, spans, color):
    for start, end in spans:
        ax.axvspan(start, end, color=color, alpha=0.55, zorder=0, linewidth=0)


def style_time_axis(ax, xlabel=True):
    ax.set_xlabel("时间 / s" if xlabel else "")
    ax.margins(x=0.01)


def cover_time(ax, frames):
    """按本次数据的起止时间撑开横轴，避免加长直道被截在图外。"""
    t0 = min(float(df["time"].iloc[0]) for df in frames)
    t1 = max(float(df["time"].iloc[-1]) for df in frames)
    pad = 0.012 * max(t1 - t0, 1e-6)
    ax.set_xlim(t0 - pad, t1 + pad)


def cover_xy(ax, frames_xy):
    xs = np.concatenate([np.asarray(x, dtype=float) for x, _ in frames_xy])
    ys = np.concatenate([np.asarray(y, dtype=float) for _, y in frames_xy])
    finite = np.isfinite(xs) & np.isfinite(ys)
    xs = xs[finite]
    ys = ys[finite]
    pad = 0.03 * max(float(xs.max() - xs.min()), float(ys.max() - ys.min()), 0.1)
    ax.set_xlim(float(xs.min()) - pad, float(xs.max()) + pad)
    ax.set_ylim(float(ys.min()) - pad, float(ys.max()) + pad)


def loss_events(time, active, dt):
    time = np.asarray(time, dtype=float)
    active = np.asarray(active, dtype=bool)
    events = []
    i = 0
    n = len(active)
    while i < n:
        if not active[i]:
            i += 1
            continue
        j = i + 1
        while j < n and active[j]:
            j += 1
        steps = j - i
        if steps >= LOSS_STEPS:
            events.append(
                {
                    "t_start": float(time[i]),
                    "t_end": float(time[j - 1] + dt),
                    "duration": steps * dt,
                    "steps": int(steps),
                }
            )
        i = j
    return events


def corner_bounds(segments):
    corners = segments[segments["kind"] == "corner"]
    sharps = segments[segments["kind"] == "sharp"]
    if corners.empty or sharps.empty:
        raise SystemExit("路径里没有急弯或直角弯，无法按路径定义分段。")
    return {
        "corner_s0": float(corners["s0"].iloc[0]),
        "corner_s1": float(corners["s1"].iloc[-1]),
        "sharp_s0": float(sharps["s0"].iloc[0]),
        "sharp_s1": float(sharps["s1"].iloc[-1]),
    }


def shade_masks(df, bounds):
    s = df["s"].to_numpy(dtype=float)
    dist = df["dist"].to_numpy(dtype=float)
    sharp = (s >= bounds["sharp_s0"]) & (s <= bounds["sharp_s1"])
    corner = (
        (s >= bounds["corner_s0"] - LOOKAHEAD)
        & (s <= bounds["corner_s1"] + 0.12)
        & (dist <= 0.18)
    )
    return sharp, corner


def metric_masks(df, bounds):
    s = df["s"].to_numpy(dtype=float)
    dist = df["dist"].to_numpy(dtype=float)
    seg = df["segment"].astype(str).to_numpy()
    e_y = df["e_y"].to_numpy(dtype=float)
    pre_corner = s < (bounds["corner_s0"] - LOOKAHEAD)
    straight_gentle = np.isin(seg, ["straight", "gentle"]) & pre_corner
    s_curve = seg == "s"
    sharp = seg == "sharp"
    corner_near = (
        (s >= bounds["corner_s0"] - LOOKAHEAD)
        & (s <= bounds["corner_s1"] + 0.12)
        & (dist <= 0.18)
    )
    entered = np.flatnonzero(s >= bounds["corner_s0"] - LOOKAHEAD)
    corner = np.zeros(len(df), dtype=bool)
    if len(entered):
        i0 = int(entered[0])
        i1 = len(df)
        run = 0
        for i in range(i0, len(df)):
            recovered = (
                s[i] > bounds["corner_s1"]
                and seg[i] == "straight"
                and np.isfinite(dist[i])
                and dist[i] < 0.05
                and np.isfinite(e_y[i])
                and abs(e_y[i]) < 0.03
            )
            run = run + 1 if recovered else 0
            if run >= 10:
                i1 = i - 9
                break
        corner[i0:i1] = True
    bend = sharp | corner
    return {
        "straight_gentle": straight_gentle,
        "s_curve": s_curve,
        "sharp": sharp,
        "corner": corner,
        "corner_near": corner_near,
        "bend": bend,
    }


def rate_rms(omega, mask, dt):
    omega = np.asarray(omega, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    pair = mask[:-1] & mask[1:] & np.isfinite(omega[:-1]) & np.isfinite(omega[1:])
    if not np.any(pair):
        return float("nan")
    rate = np.diff(omega)[pair] / dt
    return float(np.sqrt(np.mean(rate ** 2)))


def command_lead(df, kappa_thr):
    """阶跃之前，同方向的 ω_cmd 有没有提前超过 0.5 rad/s，以及能不能保持到阶跃前一拍。

    只在 t_ref 之前的窗口里找。和阶跃同一拍才抬起来的，不算提前。
    """
    time = df["time"].to_numpy(dtype=float)
    kappa = df["kappa_r"].to_numpy(dtype=float)
    cmd = df["omega_cmd"].to_numpy(dtype=float)
    speed = df["v"].to_numpy(dtype=float)
    hit = np.flatnonzero(np.isfinite(kappa) & (np.abs(kappa) >= kappa_thr))
    if len(hit) == 0:
        return None
    i_ref = int(hit[0])
    t_ref = float(time[i_ref])
    sign = float(np.sign(kappa[i_ref])) or 1.0
    info = {
        "t_ref": t_ref,
        "t_rise": None,
        "lead_s": None,
        "held": False,
        "peak_before": float("nan"),
        "cmd_before": float("nan"),
        "sensor_lead_s": float("nan"),
        "sign": sign,
    }
    speed_mask = (time >= t_ref - 0.50) & (time < t_ref) & np.isfinite(speed) & (speed > 0.05)
    if np.any(speed_mask):
        info["sensor_lead_s"] = LOOKAHEAD / float(np.mean(speed[speed_mask]))
    if i_ref == 0:
        return info
    info["cmd_before"] = float(cmd[i_ref - 1]) if np.isfinite(cmd[i_ref - 1]) else float("nan")
    window = (time >= t_ref - RISE_LOOKBACK) & (time < t_ref) & np.isfinite(cmd)
    signed = cmd * sign
    if np.any(window):
        info["peak_before"] = float(np.max(signed[window]))
    early = np.flatnonzero(window & (signed >= RISE_LEVEL))
    if len(early) == 0:
        return info
    i_rise = int(early[0])
    info["t_rise"] = float(time[i_rise])
    info["lead_s"] = float(t_ref - time[i_rise])
    info["held"] = bool(np.isfinite(signed[i_ref - 1]) and signed[i_ref - 1] >= RISE_LEVEL)
    return info


def collect(df, meta, masks, dt):
    e_y = df["e_y"].to_numpy(dtype=float)
    time = df["time"].to_numpy(dtype=float)
    geo = np.isfinite(e_y) & (np.abs(e_y) > SENSOR_HALF)
    sensor = np.isfinite(df["sum_weight"].to_numpy(dtype=float)) & (df["sum_weight"].to_numpy(dtype=float) <= 0.0)
    all_mask = np.ones(len(df), dtype=bool)
    return {
        "sg_rms": rms(e_y[masks["straight_gentle"]]),
        "s_rms": rms(e_y[masks["s_curve"]]),
        "sharp_max": max_abs(e_y[masks["sharp"]]),
        "sharp_rms": rms(e_y[masks["sharp"]]),
        "corner_max": max_abs(e_y[masks["corner"]]),
        "corner_rms": rms(e_y[masks["corner"]]),
        "bend_max": max_abs(e_y[masks["bend"]]),
        "bend_rms": rms(e_y[masks["bend"]]),
        "rate_all": rate_rms(df["omega_cmd"], all_mask, dt),
        "rate_bend": rate_rms(df["omega_cmd"], masks["bend"], dt),
        "rate_sg": rate_rms(df["omega_cmd"], masks["straight_gentle"], dt),
        "rate_sharp": rate_rms(df["omega_cmd"], masks["sharp"], dt),
        "rate_corner_near": rate_rms(df["omega_cmd"], masks["corner_near"], dt),
        "sharp_v": float(np.nanmean(df["v"].to_numpy(dtype=float)[masks["sharp"]]))
        if np.any(masks["sharp"])
        else float("nan"),
        "geo_loss": loss_events(time, geo, dt),
        "sensor_loss": loss_events(time, sensor, dt),
        "n": {key: int(np.sum(value)) for key, value in masks.items()},
        "duration": float(meta["duration_s"]),
        "steps": int(meta["steps"]),
        "stopped": bool(meta["stopped_off_line"]),
        "mean_v": float(np.nanmean(df["v"].to_numpy(dtype=float))),
        "max_ey": max_abs(e_y),
        "lead_sharp": command_lead(df, 3.0),
        "lead_corner": command_lead(df, 10.0),
    }


def fmt(value, digits=4):
    if value is None or not np.isfinite(value):
        return "—"
    return f"{value:.{digits}f}"


def loss_cell(events):
    if not events:
        return "否"
    duration = sum(item["duration"] for item in events)
    return f"是（{len(events)} 次，{duration:.2f} s）"


def lead_cell(info):
    if info is None:
        return "未能判定"
    if info.get("lead_s") is None:
        return "没有提前抬起"
    milliseconds = info["lead_s"] * 1000.0
    if info.get("held"):
        return f"提前 {milliseconds:.0f} ms，并保持到阶跃前"
    return f"提前 {milliseconds:.0f} ms 抬起，阶跃前已回落"


def event_table(events):
    if not events:
        return "无。\n"
    lines = [
        "| 开始 / s | 结束 / s | 持续 / s | 步数 |",
        "| ---: | ---: | ---: | ---: |",
    ]
    for item in events:
        lines.append(
            f"| {item['t_start']:.2f} | {item['t_end']:.2f} | {item['duration']:.2f} | {item['steps']} |"
        )
    return "\n".join(lines) + "\n"


def compare_line(label, pid, mpc, unit, digits=4):
    if not (np.isfinite(pid) and np.isfinite(mpc)):
        return f"- {label}：数据不足。"
    gap = pid - mpc
    if abs(pid) < 1e-12:
        rel = float("nan")
    else:
        rel = gap / abs(pid) * 100.0
    if unit == "m" and abs(gap) < 0.05:
        shown = f"绝对差 {abs(gap) * 1000:.1f} mm"
    else:
        shown = f"绝对差 {abs(gap):.{digits}f} {unit}"
    if not np.isfinite(rel) or abs(rel) < 5.0:
        tail = f"{shown}，相对差不到 5%。"
    elif gap > 0:
        tail = f"MPC 更低，{shown}，相对 PID 减小 {rel:.1f}%。"
    else:
        tail = f"PID 更低，{shown}，MPC 比 PID 高 {-rel:.1f}%。"
    return f"- {label}：PID {fmt(pid, digits)} {unit}，MPC {fmt(mpc, digits)} {unit}。{tail}"


def preview_line(name, info):
    if info is None:
        return f"- {name}：没有找到曲率阶跃。"
    sensor_ms = info["sensor_lead_s"] * 1000.0
    sensor_txt = (
        f"进入前车速对应的传感器前视时间约 {sensor_ms:.0f} ms。"
        if np.isfinite(info["sensor_lead_s"])
        else ""
    )
    if info.get("lead_s") is None:
        return (
            f"- {name}：阶跃前 {RISE_LOOKBACK:.2f} s 内，同方向指令没有超过 {RISE_LEVEL:.1f} rad/s。"
            f"指令是和 ω_r 一起抬起来的。{sensor_txt}"
        )
    sentence = f"- {name}：{lead_cell(info)}。阶跃前同方向峰值 {info['peak_before']:.2f} rad/s。{sensor_txt}"
    if info.get("held") and np.isfinite(info["sensor_lead_s"]):
        if abs(info["lead_s"] - info["sensor_lead_s"]) <= 0.08:
            sentence += "提前量和车头传感器看到弯道的时间同量级。"
        elif info["lead_s"] > info["sensor_lead_s"] + 0.08:
            sentence += "提前量大于传感器前视时间。"
    elif not info.get("held"):
        sentence += "抬起没有保持到阶跃前一拍，不能算完成的预打角。"
    return sentence


def build_report(pid_stat, mpc_stat, pid_meta, mpc_meta, params, segments):
    rows = [
        ("直道/缓弯 RMS(e_y) / m", "sg_rms", 4),
        ("S 弯 RMS(e_y) / m", "s_rms", 4),
        ("急弯最大 |e_y| / m", "sharp_max", 4),
        ("急弯 RMS(e_y) / m", "sharp_rms", 4),
        ("急弯平均速度 / (m/s)", "sharp_v", 3),
        ("直角弯最大 |e_y| / m", "corner_max", 4),
        ("直角弯 RMS(e_y) / m", "corner_rms", 4),
        ("急弯+直角弯最大 |e_y| / m", "bend_max", 4),
        ("急弯+直角弯 RMS(e_y) / m", "bend_rms", 4),
        ("全程最大 |e_y| / m", "max_ey", 4),
        ("全程控制量变化率 RMS / (rad/s²)", "rate_all", 2),
        ("直道/缓弯控制量变化率 RMS / (rad/s²)", "rate_sg", 2),
        ("急弯控制量变化率 RMS / (rad/s²)", "rate_sharp", 2),
        ("直角弯路径窗口控制量变化率 RMS / (rad/s²)", "rate_corner_near", 2),
        ("急弯+直角弯控制量变化率 RMS / (rad/s²)", "rate_bend", 2),
        ("平均速度 / (m/s)", "mean_v", 3),
    ]
    table = [
        "| 指标 | PID | MPC |",
        "| --- | ---: | ---: |",
    ]
    for label, key, digits in rows:
        table.append(f"| {label} | {fmt(pid_stat[key], digits)} | {fmt(mpc_stat[key], digits)} |")
    table.append(f"| 几何丢线（\\|e_y\\| > {SENSOR_HALF:.2f} m 连续 ≥ {LOSS_STEPS} 步） | {loss_cell(pid_stat['geo_loss'])} | {loss_cell(mpc_stat['geo_loss'])} |")
    table.append(f"| 传感器丢线（Σweight = 0 连续 ≥ {LOSS_STEPS} 步） | {loss_cell(pid_stat['sensor_loss'])} | {loss_cell(mpc_stat['sensor_loss'])} |")
    table.append(f"| 急弯入口 ω_cmd 相对 ω_r | {lead_cell(pid_stat['lead_sharp'])} | {lead_cell(mpc_stat['lead_sharp'])} |")
    table.append(f"| 直角弯入口 ω_cmd 相对 ω_r | {lead_cell(pid_stat['lead_corner'])} | {lead_cell(mpc_stat['lead_corner'])} |")
    table.append(f"| 仿真时长 / s | {pid_stat['duration']:.2f} | {mpc_stat['duration']:.2f} |")
    table.append(f"| 因离线过远结束 | {'是' if pid_stat['stopped'] else '否'} | {'是' if mpc_stat['stopped'] else '否'} |")

    counts = [
        "| 区段 | PID 样本 | MPC 样本 |",
        "| --- | ---: | ---: |",
    ]
    count_name = {
        "straight_gentle": "直道/缓弯",
        "s_curve": "S 弯",
        "sharp": "急弯",
        "corner_near": "直角弯路径窗口",
        "corner": "直角弯指标窗口",
        "bend": "急弯+直角弯",
    }
    for key, label in count_name.items():
        counts.append(f"| {label} | {pid_stat['n'][key]} | {mpc_stat['n'][key]} |")

    path_lines = [
        "| 名称 | 类型 | 起点 / m | 终点 / m | 曲率 / m⁻¹ |",
        "| --- | --- | ---: | ---: | ---: |",
    ]
    for _, seg in segments.iterrows():
        path_lines.append(
            f"| {seg['name']} | {seg['kind']} | {float(seg['s0']):.3f} | {float(seg['s1']):.3f} | {float(seg['kappa']):.2f} |"
        )

    freeze = params.get("freeze_linearization")
    lines = [
        "# PID 与 MPC 仿真对比",
        "",
        "数据来自 `results/data/pid_timeseries.csv` 和 `results/data/mpc_timeseries.csv`。直道5已加长到 2.90 m。没有改控制律。",
        "",
        "## 仿真条件",
        "",
        f"- 随机种子：{pid_meta['seed']}",
        f"- 控制周期：{pid_meta['dt']:.3f} s",
        f"- 参考路径长度：{pid_meta['path_length_m']:.3f} m",
        f"- 初始横向偏差：{pid_meta['initial_lateral_offset']} m，初始航向偏差：{pid_meta['initial_heading_error']} rad",
        f"- 两边 t = 0 的位姿相同。PID 跑了 {pid_meta['steps']} 步，MPC 跑了 {mpc_meta['steps']} 步。",
        f"- MPC 参数保持项目原值：N = {params['N']}，Q = {params['Q']}，R = {params['R']}，S = {params['S']}，P = {params['P']}。",
        f"- `freeze_linearization = {freeze}`：预测时域内参考角速度保持当前拍的 ω_r，不读取前方路段曲率。",
        "",
        "## 量定义",
        "",
        "- `e_y` 是车轴中心到路径的几何横向误差，路径左侧为正，单位 m。传感器加权误差在八路都看不到线时会被置 0，并且在约 ±0.06 m 饱和，所以不用它做偏离对比。",
        "- `omega_cmd` 是轮速滞后之前的角速度指令。MPC 用控制器输出；PID 由左右轮 PWM 按同一差速模型反算。",
        "- `omega_r = kappa_r * v_r`。急弯和直角弯 `v_r = 0.18 m/s`，其余 `0.30 m/s`。",
        "- 直道/缓弯只统计直角弯前视窗之前、段类型为 straight 或 gentle 的样本，避免丢线后投影落到最后一段直道上。",
        "- 急弯按段类型 sharp。直角弯指标从弧长进入 `s0 - 0.075 m` 起，到连续 0.2 s 回到下一段直道（到线距离 < 0.05 m 且 |e_y| < 0.03 m）为止；回不去就计到仿真结束。",
        "- 图上的色带比指标窗口短：急弯是投影落在 sharp 段；直角弯是弧长落在 `[s0 - 0.075, s1 + 0.12] m` 且到线距离 ≤ 0.18 m。色带取两车时间段的并集。",
        f"- 几何丢线：|e_y| > {SENSOR_HALF:.2f} m（传感器阵列半宽）连续不少于 {LOSS_STEPS} 步。传感器丢线沿用原仿真：Σweight = 0 连续不少于 {LOSS_STEPS} 步。",
        "- 控制量变化率是相邻拍 `omega_cmd` 之差除以 dt 后的 RMS。直角弯的曲率阶跃会抬高弯道内的这个数；直道/缓弯上的变化率用来看持续抖动。",
        f"- 入口提前量：在 ω_r 阶跃之前 {RISE_LOOKBACK:.2f} s 内，同方向 ω_cmd 第一次超过 {RISE_LEVEL:.1f} rad/s 的时刻。直道起伏大约 ±0.2 rad/s，0.5 rad/s 用来避开这些起伏。若阶跃前一拍仍不低于这个幅度，记为“保持”；否则记为“已回落”。和阶跃同一拍才抬起来的，不算提前。",
        "",
        "## 指标",
        "",
        *table,
        "",
        "### 样本数",
        "",
        *counts,
        "",
        "### 几何丢线区间",
        "",
        "PID：",
        "",
        event_table(pid_stat["geo_loss"]).rstrip(),
        "",
        "MPC：",
        "",
        event_table(mpc_stat["geo_loss"]).rstrip(),
        "",
        "## 数值对照",
        "",
        compare_line("直道/缓弯 RMS 横向误差", pid_stat["sg_rms"], mpc_stat["sg_rms"], "m"),
        compare_line("S 弯 RMS 横向误差", pid_stat["s_rms"], mpc_stat["s_rms"], "m"),
        compare_line("急弯最大横向误差", pid_stat["sharp_max"], mpc_stat["sharp_max"], "m"),
        compare_line("急弯 RMS 横向误差", pid_stat["sharp_rms"], mpc_stat["sharp_rms"], "m"),
        compare_line("急弯平均速度", pid_stat["sharp_v"], mpc_stat["sharp_v"], "m/s", 3),
        compare_line("直角弯最大横向误差", pid_stat["corner_max"], mpc_stat["corner_max"], "m"),
        compare_line("直角弯 RMS 横向误差", pid_stat["corner_rms"], mpc_stat["corner_rms"], "m"),
        compare_line("全程控制量变化率 RMS", pid_stat["rate_all"], mpc_stat["rate_all"], "rad/s²", 2),
        compare_line("直道/缓弯控制量变化率 RMS", pid_stat["rate_sg"], mpc_stat["rate_sg"], "rad/s²", 2),
        compare_line("急弯控制量变化率 RMS", pid_stat["rate_sharp"], mpc_stat["rate_sharp"], "rad/s²", 2),
        compare_line("直角弯路径窗口控制量变化率 RMS", pid_stat["rate_corner_near"], mpc_stat["rate_corner_near"], "rad/s²", 2),
        compare_line("急弯+直角弯控制量变化率 RMS", pid_stat["rate_bend"], mpc_stat["rate_bend"], "rad/s²", 2),
        (
            f"- 几何丢线：PID {loss_cell(pid_stat['geo_loss'])}；"
            f"MPC {loss_cell(mpc_stat['geo_loss'])}。"
        ),
        (
            f"- 传感器丢线：PID {loss_cell(pid_stat['sensor_loss'])}；"
            f"MPC {loss_cell(mpc_stat['sensor_loss'])}。"
        ),
        (
            "- 路径完成："
            + (
                "PID 因离线过远提前结束，MPC 跑完设定路径。"
                if pid_stat["stopped"] and not mpc_stat["stopped"]
                else (
                    "两边都因离线过远提前结束。"
                    if pid_stat["stopped"] and mpc_stat["stopped"]
                    else "两边都没有因离线过远提前结束。"
                )
            )
        ),
        "",
        "## 预瞄",
        "",
        "图 `compare_preview.png` 画的是 MPC 的 ω_cmd 和当前投影点的 ω_r。点线是轴心进入弯道、ω_r 发生阶跃的时刻。",
        "这版 MPC 的 `freeze_linearization` 为 true，预测时域里的参考角速度保持当前值，不读取前方路段的曲率。弯道内指令高于 ω_r，是在修已经出现的误差，不是提前打角。",
        "",
        preview_line("MPC 急弯入口", mpc_stat["lead_sharp"]),
        preview_line("MPC 直角弯入口", mpc_stat["lead_corner"]),
        preview_line("PID 急弯入口（同一套阈值，供对照）", pid_stat["lead_sharp"]),
        preview_line("PID 直角弯入口（同一套阈值，供对照）", pid_stat["lead_corner"]),
        "",
        "## 图上能核对的结论",
        "",
        *figure_notes(pid_stat, mpc_stat, params, segments),
        "",
        "## 参考路径",
        "",
        *path_lines,
        "",
    ]
    return "\n".join(lines) + "\n"


def legend_with_spans(ax, sharp_spans, corner_spans, loc="best"):
    handles, labels = ax.get_legend_handles_labels()
    if sharp_spans:
        handles.append(Patch(facecolor=SHARP_COLOR, edgecolor="none", alpha=0.9, label="急弯"))
        labels.append("急弯")
    if corner_spans:
        handles.append(Patch(facecolor=CORNER_COLOR, edgecolor="none", alpha=0.9, label="直角弯"))
        labels.append("直角弯")
    ax.legend(handles, labels, loc=loc)


def figure_notes(pid_stat, mpc_stat, params, segments):
    corner = segments[segments["kind"] == "corner"].iloc[0]
    omega_need = abs(float(corner["kappa"])) * float(params["v_low"])
    omega_max = abs(float(params["omega_max"]))
    return [
        "轨迹图在急弯之前，红线和蓝线都贴着灰色参考线。直角弯之后红线离开参考线并且没有回来，蓝线向外鼓出一截后又回到线上。右图把这段放大，离开路径的是 PID。",
        (
            f"误差图在橙色急弯里两条线都还在 ±1 cm 以内。MPC 急弯最大横向误差 {mpc_stat['sharp_max']*1000:.1f} mm，"
            f"PID 为 {pid_stat['sharp_max']*1000:.1f} mm。粉色直角弯之后，PID 一直降到 {pid_stat['corner_max']*100:.1f} cm 并且仿真结束；"
            f"MPC 降到 {mpc_stat['corner_max']*100:.1f} cm 后拉回来。两条线都会穿过 ±6 cm 的丢线线，差别是 PID 回不来。"
        ),
        (
            f"指令图在直道、缓弯和急弯上，红线和蓝线的起伏幅度接近：直道/缓弯变化率都是约 {pid_stat['rate_sg']:.2f} rad/s²，"
            f"急弯段 PID {pid_stat['rate_sharp']:.2f}、MPC {mpc_stat['rate_sharp']:.2f} rad/s²。"
            "剧烈来回打方向盘出现在直角弯之后的红线上；蓝线是打到上限、停住，再收回。"
        ),
        (
            f"预瞄图的急弯入口，蓝线是和灰色虚线一起抬起来的，阶跃前没有超过 {RISE_LEVEL:.1f} rad/s。"
            "直角弯入口在点线之前有一个大约 1 rad/s 的鼓包，但点线之前已经回到负值；随后和 ω_r 一起打到约 "
            f"{omega_max:.2f} rad/s 的上限。ω_r 本身阶跃到 {omega_need:.1f} rad/s，超出纵轴，也超出这个上限。"
        ),
        (
            f"直角弯半径 0.025 m、低速参考 {float(params['v_low']):.2f} m/s 时，几何上需要角速度 {omega_need:.2f} rad/s，"
            f"而左右轮 PWM 上限对应的角速度只有 {omega_max:.2f} rad/s。这条直角弯比当前车速允许的转弯更急，所以两边都会在这里偏离；"
            "MPC 偏离后能拉回，PID 不能。"
        ),
        (
            f"急弯上 MPC 平均速度 {mpc_stat['sharp_v']:.2f} m/s，PID 为 {pid_stat['sharp_v']:.2f} m/s。"
            "急弯误差更小，有一部分来自 MPC 按路段把参考速度降到 0.18 m/s，不全是反馈律的差别。"
        ),
    ]


def mark_lead(ax, info, df):
    if info is None:
        return
    t_ref = info["t_ref"]
    ax.axvline(t_ref, color="#444444", linestyle=":", linewidth=1.0, zorder=2)
    if info.get("t_rise") is None:
        ax.annotate(
            "与 ω_r 同时抬起",
            xy=(t_ref, 0.0),
            xytext=(6, 16),
            textcoords="offset points",
            fontsize=9,
            color="#333333",
        )
        return
    time = df["time"].to_numpy(dtype=float)
    cmd = df["omega_cmd"].to_numpy(dtype=float)
    k = int(np.argmin(np.abs(time - info["t_rise"])))
    y = float(cmd[k]) if np.isfinite(cmd[k]) else 0.0
    ax.plot([time[k]], [y], "o", color=MPC_COLOR, markersize=5, zorder=4)
    text = "提前抬起并保持" if info.get("held") else "曾提前抬起，阶跃前已回落"
    ax.annotate(text, xy=(time[k], y), xytext=(6, 12), textcoords="offset points", fontsize=9, color="#333333")


def save_trajectory(pid, mpc, ref, segments, bounds):
    fig, axes = plt.subplots(1, 2, figsize=(12.2, 6.2))
    panels = (
        (axes[0], "全程轨迹", None),
        (axes[1], "急弯与直角弯附近", bounds),
    )
    for ax, title, limit in panels:
        ax.plot(ref["x"], ref["y"], color=REF_COLOR, linestyle="--", linewidth=1.6, label="参考路径", zorder=1)
        ax.plot(pid["x"], pid["y"], color=PID_COLOR, linewidth=1.6, label="PID", zorder=2)
        ax.plot(mpc["x"], mpc["y"], color=MPC_COLOR, linewidth=1.5, label="MPC", zorder=3)
        ax.scatter([ref["x"].iloc[0]], [ref["y"].iloc[0]], s=22, c="black", zorder=4, label="起点")
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(title)
        ax.set_xlabel("x / m")
        ax.set_ylabel("y / m")
        if limit is None:
            cover_xy(
                ax,
                [
                    (ref["x"], ref["y"]),
                    (pid["x"], pid["y"]),
                    (mpc["x"], mpc["y"]),
                ],
            )
        else:
            bend = ref[(ref["s"] >= limit["sharp_s0"] - 0.05) & (ref["s"] <= limit["corner_s1"] + 0.20)]
            pad = 0.28
            ax.set_xlim(float(bend["x"].min()) - pad, float(bend["x"].max()) + pad)
            ax.set_ylim(float(bend["y"].min()) - pad, float(bend["y"].max()) + pad)
    for _, seg in segments.iterrows():
        if seg["kind"] not in ("sharp", "corner"):
            continue
        mid = 0.5 * (float(seg["s0"]) + float(seg["s1"]))
        i = int(np.argmin(np.abs(ref["s"].to_numpy(dtype=float) - mid)))
        name = "急弯" if seg["kind"] == "sharp" else "直角弯"
        for ax in axes:
            ax.annotate(
                name,
                (float(ref["x"].iloc[i]), float(ref["y"].iloc[i])),
                textcoords="offset points",
                xytext=(6, 8),
                fontsize=9,
                color="#444444",
            )
    axes[0].legend(loc="lower left")
    fig.suptitle("轨迹对比", fontsize=14)
    fig.tight_layout()
    fig.savefig(FIG / "compare_trajectory.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def save_error(pid, mpc, sharp_spans, corner_spans):
    fig, axes = plt.subplots(2, 1, figsize=(10.8, 7.2), sharex=True)
    titles = (
        "横向误差对比（全程）",
        f"横向误差对比（纵轴放大到 ±{EY_ZOOM * 100:.0f} cm）",
    )
    for ax, title in zip(axes, titles):
        shade(ax, sharp_spans, SHARP_COLOR)
        shade(ax, corner_spans, CORNER_COLOR)
        ax.plot(pid["time"], pid["e_y"], color=PID_COLOR, linewidth=1.15, label="PID")
        ax.plot(mpc["time"], mpc["e_y"], color=MPC_COLOR, linewidth=1.15, label="MPC")
        ax.axhline(0.0, color="#999999", linewidth=0.8)
        ax.set_title(title)
        ax.set_ylabel("横向误差 e_y / m（左为正）")
        legend_with_spans(ax, sharp_spans, corner_spans, loc="upper left")
    axes[1].set_ylim(-EY_ZOOM, EY_ZOOM)
    axes[1].axhline(SENSOR_HALF, color="#666666", linestyle="--", linewidth=0.8)
    axes[1].axhline(-SENSOR_HALF, color="#666666", linestyle="--", linewidth=0.8)
    style_time_axis(axes[0], xlabel=False)
    style_time_axis(axes[1], xlabel=True)
    cover_time(axes[1], (pid, mpc))
    fig.suptitle("色带为急弯（橙）和直角弯（红）。灰色虚线是 ±0.06 m 几何丢线阈值。", fontsize=11)
    fig.tight_layout()
    fig.savefig(FIG / "compare_error.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def save_omega(pid, mpc, sharp_spans, corner_spans):
    fig, axes = plt.subplots(2, 1, figsize=(10.8, 7.2), sharex=True)
    for ax in axes:
        shade(ax, sharp_spans, SHARP_COLOR)
        shade(ax, corner_spans, CORNER_COLOR)
        ax.plot(pid["time"], pid["omega_cmd"], color=PID_COLOR, linewidth=0.9, label="PID")
        ax.plot(mpc["time"], mpc["omega_cmd"], color=MPC_COLOR, linewidth=1.05, label="MPC")
        ax.set_ylabel("角速度指令 ω_cmd / (rad/s)")
        legend_with_spans(ax, sharp_spans, corner_spans, loc="upper left")
    axes[0].set_title("角速度指令对比（全程）")
    axes[1].set_title("角速度指令对比（纵轴放大到 ±2 rad/s）")
    axes[1].set_ylim(-2.0, 2.0)
    style_time_axis(axes[0], xlabel=False)
    style_time_axis(axes[1], xlabel=True)
    cover_time(axes[1], (pid, mpc))
    fig.suptitle("色带为急弯（橙）和直角弯（红）。下图用来看直道和缓弯上的抖动。", fontsize=11)
    fig.tight_layout()
    fig.savefig(FIG / "compare_omega.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def save_preview(mpc, sharp_spans, corner_spans, lead_sharp, lead_corner):
    fig, axes = plt.subplots(3, 1, figsize=(10.8, 9.2))
    axes[0].set_title("MPC 参考角速度与指令（全程）")
    shade(axes[0], sharp_spans, SHARP_COLOR)
    shade(axes[0], corner_spans, CORNER_COLOR)
    axes[0].plot(mpc["time"], mpc["omega_r"], color=REF_COLOR, linestyle="--", linewidth=1.4, label="参考角速度 ω_r")
    axes[0].plot(mpc["time"], mpc["omega_cmd"], color=MPC_COLOR, linewidth=1.15, label="MPC 指令 ω_cmd")
    legend_with_spans(axes[0], sharp_spans, corner_spans, loc="upper left")
    axes[0].set_ylabel("角速度 / (rad/s)")
    style_time_axis(axes[0], xlabel=False)
    cover_time(axes[0], (mpc,))

    windows = (
        (axes[1], lead_sharp, "急弯入口"),
        (axes[2], lead_corner, "直角弯入口"),
    )
    time = mpc["time"].to_numpy(dtype=float)
    for ax, info, title in windows:
        if info is None:
            ax.set_title(f"{title}（没有找到曲率阶跃）")
            continue
        t_ref = info["t_ref"]
        window = (time >= t_ref - 0.80) & (time <= t_ref + 0.70)
        ax.plot(mpc["time"], mpc["omega_r"], color=REF_COLOR, linestyle="--", linewidth=1.5, label="参考角速度 ω_r")
        ax.plot(mpc["time"], mpc["omega_cmd"], color=MPC_COLOR, linewidth=1.4, label="MPC 指令 ω_cmd")
        mark_lead(ax, info, mpc)
        ax.set_xlim(t_ref - 0.80, t_ref + 0.70)
        local_r = mpc.loc[window, "omega_r"].to_numpy(dtype=float)
        if np.nanmax(np.abs(local_r)) > 5.5:
            ax.set_ylim(-1.5, 5.5)
            ax.text(
                0.01,
                0.97,
                "ω_r 阶跃超过纵轴（直角弯约 7.2 rad/s），点线是阶跃时刻",
                transform=ax.transAxes,
                va="top",
                fontsize=8,
                color="#333333",
            )
        ax.set_title(f"{title}：{lead_cell(info)}")
        ax.set_ylabel("角速度 / (rad/s)")
        ax.legend(loc="best")
    style_time_axis(axes[1], xlabel=False)
    style_time_axis(axes[2], xlabel=True)
    fig.suptitle("预瞄核对：ω_cmd 是否早于 ω_r 的弯道阶跃", fontsize=14)
    fig.tight_layout()
    fig.savefig(FIG / "compare_preview.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def summarize_final_straight(pid, mpc, segments):
    """末段直道是否回正。只打印，不改指标表的算法。"""
    last = segments[segments["name"] == "直道5"]
    if last.empty:
        print("路径里没有直道5，跳过末段回正说明。")
        return
    s0 = float(last["s0"].iloc[0])
    length = float(last["length"].iloc[0])

    def profile(df, label):
        on = df[df["s"] >= s0]
        if on.empty:
            print(f"- {label} 没有投影到直道5。")
            return
        t_enter = float(on["time"].iloc[0])
        peak_i = on["e_y"].abs().idxmax()
        t_peak = float(on.loc[peak_i, "time"])
        peak = float(on.loc[peak_i, "e_y"])
        after = on[on["time"] >= t_peak]
        ey = after["e_y"].to_numpy(dtype=float)
        tt = after["time"].to_numpy(dtype=float)
        hit = np.flatnonzero(np.isfinite(ey) & (np.abs(ey) < 0.01))
        end = float(on["e_y"].iloc[-1])
        if len(hit) == 0:
            print(
                f"- {label} 从 t={t_enter:.2f} s 起投影落在直道5，"
                f"横向误差峰值 {peak * 100:.1f} cm（t={t_peak:.2f} s），"
                f"结束时仍为 {end * 100:.1f} cm，没有回到 1 cm 以内。"
            )
            return
        t_ok = float(tt[int(hit[0])])
        print(
            f"- {label} 在 t={t_enter:.2f} s 进入直道5，"
            f"横向误差峰值 {peak * 100:.1f} cm（t={t_peak:.2f} s），"
            f"之后在 t={t_ok:.2f} s 回到 1 cm 以内"
            f"（进入后 {t_ok - t_enter:.2f} s，峰值后 {t_ok - t_peak:.2f} s），"
            f"结束时 {end * 1000:.2f} mm。"
        )

    def settled_after_peak(df):
        on = df[df["s"] >= s0]
        if on.empty:
            return False
        t_peak = float(on.loc[on["e_y"].abs().idxmax(), "time"])
        after = on[on["time"] > t_peak]
        if after.empty:
            return False
        return bool(np.any(np.abs(after["e_y"].to_numpy(dtype=float)) < 0.01))

    print(f"末段直道5长度 {length:.2f} m，起点弧长 {s0:.3f} m。")
    profile(pid, "PID")
    profile(mpc, "MPC")
    pid_back = settled_after_peak(pid)
    mpc_back = settled_after_peak(mpc)
    if mpc_back and not pid_back:
        print(
            "末段直道上，MPC 能把横向误差收回并收到接近 0；"
            "PID 的误差一直留在分米量级，没有回正。"
            "这里的差别是 MPC 回得来，不是两边都收敛、只是 MPC 更快。"
        )
    elif mpc_back and pid_back:
        print("末段直道上 PID 和 MPC 都回到了 1 cm 以内，见上面的时刻。")
    else:
        print("末段直道上两边都没有回到 1 cm 以内。")


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    font = configure_font()
    apply_style()
    DATA.mkdir(parents=True, exist_ok=True)
    FIG.mkdir(parents=True, exist_ok=True)
    print(f"字体：{font or '未找到中文字体'}")
    pid, mpc, segments, ref, pid_meta, mpc_meta, params = load_inputs()
    bounds = corner_bounds(segments)
    dt = float(pid_meta["dt"])
    pid_masks = metric_masks(pid, bounds)
    mpc_masks = metric_masks(mpc, bounds)
    pid_stat = collect(pid, pid_meta, pid_masks, dt)
    mpc_stat = collect(mpc, mpc_meta, mpc_masks, dt)

    pid_sharp, pid_corner = shade_masks(pid, bounds)
    mpc_sharp, mpc_corner = shade_masks(mpc, bounds)
    sharp_spans = union_spans(
        [mask_to_spans(pid["time"], pid_sharp), mask_to_spans(mpc["time"], mpc_sharp)]
    )
    corner_spans = union_spans(
        [mask_to_spans(pid["time"], pid_corner), mask_to_spans(mpc["time"], mpc_corner)]
    )

    save_trajectory(pid, mpc, ref, segments, bounds)
    save_error(pid, mpc, sharp_spans, corner_spans)
    save_omega(pid, mpc, sharp_spans, corner_spans)
    save_preview(mpc, sharp_spans, corner_spans, mpc_stat["lead_sharp"], mpc_stat["lead_corner"])
    report = build_report(pid_stat, mpc_stat, pid_meta, mpc_meta, params, segments)
    (DATA / "comparison_metrics.md").write_text(report, encoding="utf-8")
    print(report)
    summarize_final_straight(pid, mpc, segments)
    print(f"已写入 {FIG / 'compare_trajectory.png'}")
    print(f"已写入 {FIG / 'compare_error.png'}")
    print(f"已写入 {FIG / 'compare_omega.png'}")
    print(f"已写入 {FIG / 'compare_preview.png'}")
    print(f"已写入 {DATA / 'comparison_metrics.md'}")


if __name__ == "__main__":
    main()
