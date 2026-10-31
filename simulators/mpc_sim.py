# -*- coding: utf-8 -*-
"""MPC 闭环仿真入口。

预测模型、QP 和 PWM 映射与原 mpc_sim 相同。
这里只改了模块导入，以及把 CSV / 指标写到 results/data、把图和 GIF 写到 results/figures。
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import csv
import math
import os

import numpy as np

from core.car_model import (
    body_velocity,
    lag_wheel_speed,
    position_error,
    pwm_to_wheel_speed,
    rk4_unicycle,
    self_check_position_error,
    sensor_scale,
    sensor_world_positions,
    simulate_analogs,
)
from config.params import CONFIG, MPC_CONFIG
from controllers.mpc import (
    LinearMPC,
    reference_speed_for_segment,
    self_check_mpc_direction,
    wrap_angle,
)
from core.path import build_demo_path
from simulators.visualize_mpc import configure_font, render_all


def initial_pose(path, cfg):
    x0, y0, h0, _ = path.pose(0.0)
    lateral = float(cfg["initial_lateral_offset"])
    x = x0 + lateral * (-np.sin(h0))
    y = y0 + lateral * np.cos(h0)
    theta = h0 + float(cfg["initial_heading_error"])
    return float(x), float(y), float(theta)


def simulate(path=None, cfg=None, mpc_cfg=None):
    cfg = CONFIG if cfg is None else cfg
    mpc_cfg = MPC_CONFIG if mpc_cfg is None else mpc_cfg
    path = build_demo_path() if path is None else path
    rng = np.random.default_rng(int(cfg["seed"]))
    controller = LinearMPC(cfg, mpc_cfg)
    dt = float(cfg["dt"])
    scale = sensor_scale(cfg)
    e_theta_std = float(mpc_cfg["e_theta_noise_std"])
    # 预热 QP（摊销 cvxpy 编译），不计入后续统计
    controller.solve(0.0, 0.0, float(mpc_cfg["v_high"]), 0.0)
    controller.reset()

    if cfg["sim_time"] is None:
        t_max = path.total_length / float(cfg["speed_for_time_estimate"]) + float(
            cfg["sim_time_margin"]
        )
    else:
        t_max = float(cfg["sim_time"])

    x, y, theta = initial_pose(path, cfg)
    t = 0.0
    hint_s = 0.0
    off_time = 0.0
    rows = []
    cruise = pwm_to_wheel_speed(cfg["base_pwm"], cfg)
    v_left_state = cruise
    v_right_state = cruise
    tau = float(cfg["wheel_speed_tau"])
    end_margin = float(cfg["sensor_lookahead"]) + 0.08
    solve_times = []

    while True:
        proj = path.project(x, y, hint_s=hint_s)
        hint_s = proj["s"]
        sensor_xy = sensor_world_positions(x, y, theta, cfg)
        distances = np.empty(8, dtype=float)
        for i in range(8):
            sp = path.project(sensor_xy[i, 0], sensor_xy[i, 1], hint_s=hint_s)
            distances[i] = sp["euclid"]
        # 与 PID 相同：每步先抽 8 路模拟量噪声，保证种子一致时噪声序列对齐
        analogs = simulate_analogs(distances, rng, cfg)
        error, sum_w = position_error(analogs, cfg)

        # e_y：传感器加权误差换算到米，符号与 PID error 一致
        e_y = float(error) * scale
        theta_r = float(proj["heading"])
        kappa_r = float(proj["curvature"])
        seg = path.segments[int(proj["segment_index"])]
        v_r = reference_speed_for_segment(seg.kind, mpc_cfg)
        omega_r = kappa_r * v_r
        # e_theta：真值 + 与传感器噪声同量级的高斯噪声（理想化假设）
        e_theta_true = wrap_angle(theta - theta_r)
        e_theta = e_theta_true + float(rng.normal(0.0, e_theta_std))

        cmd = controller.step(e_y, e_theta, v_r, omega_r)
        solve_times.append(cmd["solve_time_s"])

        # 兼容 PID metrics 字段名：sharp 表示是否处于低速参考段
        sharp = seg.kind in ("sharp", "corner")
        work_n = int(sum(a > cfg["work_sensor_analog_min"] for a in analogs))

        v_left_cmd = pwm_to_wheel_speed(cmd["left_pwm"], cfg)
        v_right_cmd = pwm_to_wheel_speed(cmd["right_pwm"], cfg)
        v_left_state = lag_wheel_speed(v_left_state, v_left_cmd, dt, tau)
        v_right_state = lag_wheel_speed(v_right_state, v_right_cmd, dt, tau)
        v_left, v_right = v_left_state, v_right_state
        v, omega = body_velocity(v_left, v_right, cfg)

        rows.append(
            {
                "t": t,
                "x": x,
                "y": y,
                "theta": theta,
                "s": proj["s"],
                "lat": proj["d_left"],
                "dist": proj["euclid"],
                "error": error,
                "e_y": e_y,
                "e_theta": e_theta,
                "e_theta_true": e_theta_true,
                "sum_weight": sum_w,
                "analogs": analogs.copy(),
                "distances": distances.copy(),
                "v": v,
                "omega": omega,
                "v_left": v_left,
                "v_right": v_right,
                "v_r": v_r,
                "omega_r": omega_r,
                "kappa_r": kappa_r,
                "delta_u": cmd["delta_u"],
                "omega_cmd": cmd["omega_cmd"],
                "left_pwm": cmd["left_pwm"],
                "right_pwm": cmd["right_pwm"],
                "base_speed": cmd["base_speed"],
                "solve_time_s": cmd["solve_time_s"],
                "work_n": work_n,
                "sharp": sharp,
                # PID 兼容占位，便于 metrics 复用
                "u_pid": cmd["delta_u"],
                "u_cmd": cmd["omega_cmd"],
            }
        )

        reached_end = (
            proj["s"] >= path.total_length - end_margin
            and proj["euclid"] < 0.08
            and t > 0.5
        )
        if reached_end or t + dt > t_max + 1e-12:
            break

        x, y, theta = rk4_unicycle(x, y, theta, v, omega, dt)
        t += dt
        dist_next = path.project(x, y, hint_s=hint_s)["euclid"]
        if dist_next > float(cfg["depart_distance"]):
            off_time += dt
        else:
            off_time = 0.0
        if off_time >= float(cfg["depart_hold"]) and t > 2.0:
            proj_off = path.project(x, y, hint_s=hint_s)
            rows.append(
                {
                    "t": t,
                    "x": x,
                    "y": y,
                    "theta": theta,
                    "s": proj_off["s"],
                    "lat": proj_off["d_left"],
                    "dist": proj_off["euclid"],
                    "error": np.nan,
                    "e_y": np.nan,
                    "e_theta": np.nan,
                    "e_theta_true": np.nan,
                    "sum_weight": np.nan,
                    "analogs": np.full(8, np.nan),
                    "distances": np.full(8, np.nan),
                    "v": v,
                    "omega": omega,
                    "v_left": v_left,
                    "v_right": v_right,
                    "v_r": np.nan,
                    "omega_r": np.nan,
                    "kappa_r": np.nan,
                    "delta_u": np.nan,
                    "omega_cmd": np.nan,
                    "left_pwm": np.nan,
                    "right_pwm": np.nan,
                    "base_speed": np.nan,
                    "solve_time_s": np.nan,
                    "work_n": np.nan,
                    "sharp": False,
                    "u_pid": np.nan,
                    "u_cmd": np.nan,
                }
            )
            break

    result = _rows_to_arrays(rows)
    result["path"] = path
    result["stopped_off_line"] = bool(off_time >= float(cfg["depart_hold"]))
    result["t_max"] = t_max
    result["qp_solve_times_s"] = np.asarray(solve_times, dtype=float)
    result["mpc_backend"] = controller.backend
    return result


def _rows_to_arrays(rows):
    keys = [
        "t",
        "x",
        "y",
        "theta",
        "s",
        "lat",
        "dist",
        "error",
        "e_y",
        "e_theta",
        "e_theta_true",
        "sum_weight",
        "v",
        "omega",
        "v_left",
        "v_right",
        "v_r",
        "omega_r",
        "kappa_r",
        "delta_u",
        "omega_cmd",
        "left_pwm",
        "right_pwm",
        "base_speed",
        "solve_time_s",
        "work_n",
        "u_pid",
        "u_cmd",
    ]
    out = {k: np.asarray([r[k] for r in rows], dtype=float) for k in keys}
    out["sharp"] = np.asarray([bool(r["sharp"]) for r in rows], dtype=bool)
    out["analogs"] = np.vstack([np.asarray(r["analogs"], dtype=float) for r in rows])
    out["distances"] = np.vstack([np.asarray(r["distances"], dtype=float) for r in rows])
    return out


def _segment_window(seg, cfg):
    if seg.kind == "corner":
        look = float(cfg["sensor_lookahead"])
        return seg.s0 - look, seg.s1 + 0.12, 0.18
    if seg.kind == "sharp":
        return seg.s0, seg.s1, None
    return seg.s0, seg.s1, None


def _mask_between(result, s0, s1, dist_max=None):
    mask = (result["s"] >= s0) & (result["s"] <= s1) & np.isfinite(result["error"])
    if dist_max is not None:
        mask = mask & (result["dist"] <= dist_max)
    return mask


def _mask_for_kind(result, path, kind, cfg):
    mask = np.zeros(len(result["s"]), dtype=bool)
    for seg in path.segments:
        if seg.kind != kind:
            continue
        s0, s1, dist_max = _segment_window(seg, cfg)
        if kind == "straight":
            mask |= (result["s"] >= seg.s0) & (result["s"] <= seg.s1)
        else:
            mask |= _mask_between(result, s0, s1, dist_max)
    if kind == "straight":
        for seg in path.segments:
            if seg.kind != "corner":
                continue
            s0, s1, _ = _segment_window(seg, cfg)
            mask &= ~((result["s"] >= s0) & (result["s"] <= s1))
        mask &= result["dist"] <= 0.05
        mask &= np.isfinite(result["error"])
    return mask


def time_spans(result, kind, cfg):
    path = result["path"]
    t = result["t"]
    if len(t) == 0:
        return []
    dt = float(cfg["dt"])
    mask = _mask_for_kind(result, path, kind, cfg)
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
        t1 = float(t[j]) if j < n else float(t[j - 1] + dt)
        spans.append((float(t[i]), t1))
        i = j
    return spans


def curvature_rise_time(result, path, kind="sharp", kappa_thr=None):
    """参考曲率开始明显增大的时刻（用于控制图标注预打角）。"""
    if kappa_thr is None:
        kappa_thr = 0.5
    for seg in path.segments:
        if seg.kind != kind:
            continue
        # 段起点即曲率跳变
        s0 = seg.s0
        # 找 s 首次进入该段的时间
        mask = result["s"] >= s0
        if np.any(mask):
            return float(result["t"][int(np.argmax(mask))])
    # 备选：kappa_r 首次超过阈值
    kr = result.get("kappa_r")
    if kr is not None and np.any(np.isfinite(kr) & (np.abs(kr) >= kappa_thr)):
        return float(result["t"][int(np.argmax(np.isfinite(kr) & (np.abs(kr) >= kappa_thr)))])
    return None


def _line_loss_events(result, cfg):
    sum_w = result["sum_weight"]
    t = result["t"]
    dt = float(cfg["dt"])
    lost = np.isfinite(sum_w) & (sum_w <= 0.0)
    min_steps = int(cfg["line_loss_min_steps"])
    events = []
    i = 0
    n = len(lost)
    while i < n:
        if not lost[i]:
            i += 1
            continue
        j = i + 1
        while j < n and lost[j]:
            j += 1
        steps = j - i
        if steps >= min_steps:
            events.append(
                {
                    "t_start": float(t[i]),
                    "t_end": float(t[j - 1] + dt),
                    "duration": steps * dt,
                    "steps": steps,
                }
            )
        i = j
    return lost, events


def compute_metrics(result, cfg=None, mpc_cfg=None):
    cfg = CONFIG if cfg is None else cfg
    mpc_cfg = MPC_CONFIG if mpc_cfg is None else mpc_cfg
    path = result["path"]
    err = result["error"]
    e_y = result["e_y"]
    valid = np.isfinite(err)
    err_v = err[valid]
    abs_e = np.abs(err_v)
    dt = float(cfg["dt"])
    lost, events = _line_loss_events(result, cfg)
    lost_steps = int(np.sum(lost))

    def region_stats(kind):
        mask = _mask_for_kind(result, path, kind, cfg)
        if not np.any(mask):
            return {
                "samples": 0,
                "max_abs_error": None,
                "rms_error": None,
                "max_abs_lat_m": None,
                "max_dist_m": None,
                "line_loss_steps": 0,
            }
        e = result["error"][mask]
        return {
            "samples": int(np.sum(mask)),
            "max_abs_error": float(np.max(np.abs(e))),
            "rms_error": float(np.sqrt(np.mean(e ** 2))),
            "max_abs_lat_m": float(np.max(np.abs(result["lat"][mask]))),
            "max_dist_m": float(np.max(result["dist"][mask])),
            "line_loss_steps": int(np.sum(lost[mask])),
        }

    sharp_flag = result["sharp"] & valid
    rising = np.zeros(len(sharp_flag), dtype=bool)
    rising[0] = bool(sharp_flag[0])
    rising[1:] = sharp_flag[1:] & ~sharp_flag[:-1]

    solve = result["qp_solve_times_s"]
    solve = solve[np.isfinite(solve)]
    omega_cmd = result["omega_cmd"]
    omega_valid = np.isfinite(omega_cmd)
    if np.sum(omega_valid) >= 2:
        domega = np.diff(omega_cmd[omega_valid]) / dt
        ctrl_rate_rms = float(np.sqrt(np.mean(domega ** 2)))
    else:
        ctrl_rate_rms = None
    mean_speed = float(np.nanmean(result["v"])) if len(result["v"]) else None

    metrics = {
        "dt": dt,
        "seed": int(cfg["seed"]),
        "duration_s": float(result["t"][-1]),
        "steps": int(len(result["t"])),
        "max_abs_error": float(np.max(abs_e)) if len(abs_e) else None,
        "rms_error": float(np.sqrt(np.mean(err_v ** 2))) if len(err_v) else None,
        "max_abs_e_y": float(np.nanmax(np.abs(e_y))) if np.any(np.isfinite(e_y)) else None,
        "rms_e_y": float(np.sqrt(np.nanmean(e_y[np.isfinite(e_y)] ** 2))) if np.any(np.isfinite(e_y)) else None,
        "max_abs_lat_m": float(np.max(np.abs(result["lat"][valid]))) if np.any(valid) else None,
        "max_dist_m": float(np.max(result["dist"])) if len(result["dist"]) else None,
        "line_loss_steps": lost_steps,
        "line_loss_time_s": lost_steps * dt,
        "line_loss_events": events,
        "line_loss_event_count": len(events),
        "line_loss_longest_s": max((e["duration"] for e in events), default=0.0),
        "sharp_mode_steps": int(np.sum(sharp_flag)),
        "sharp_mode_time_s": float(np.sum(sharp_flag) * dt),
        "sharp_mode_entries": int(np.sum(rising)),
        "stopped_off_line": bool(result["stopped_off_line"]),
        "mean_speed_mps": mean_speed,
        "control_rate_rms": ctrl_rate_rms,
        "qp_solve_time_ms_max": float(np.max(solve) * 1000.0) if len(solve) else None,
        "qp_solve_time_ms_mean": float(np.mean(solve) * 1000.0) if len(solve) else None,
        "mpc_backend": result.get("mpc_backend"),
        "e_y_max": float(mpc_cfg["e_y_max"]),
        "spans": {
            "sharp": time_spans(result, "sharp", cfg),
            "corner": time_spans(result, "corner", cfg),
        },
        "curvature_rise_t_sharp": curvature_rise_time(result, path, "sharp"),
        "curvature_rise_t_corner": curvature_rise_time(result, path, "corner"),
        "regions": {
            "straight": region_stats("straight"),
            "gentle": region_stats("gentle"),
            "s": region_stats("s"),
            "sharp": region_stats("sharp"),
            "corner": region_stats("corner"),
        },
    }
    corner_segs = [seg for seg in path.segments if seg.kind == "corner"]
    if corner_segs and np.any(valid):
        s_ref = corner_segs[0].s0 - float(cfg["sensor_lookahead"])
        after = result["s"] >= s_ref
        metrics["corner_max_dist_m"] = float(np.max(result["dist"][after])) if np.any(after) else None
    else:
        metrics["corner_max_dist_m"] = None
    sharp_mask = _mask_for_kind(result, path, "sharp", cfg)
    if np.any(sharp_mask):
        metrics["sharp_max_dist_m"] = float(np.max(result["dist"][sharp_mask]))
        metrics["sharp_max_abs_error"] = float(np.max(np.abs(result["error"][sharp_mask])))
    else:
        metrics["sharp_max_dist_m"] = None
        metrics["sharp_max_abs_error"] = None
    return metrics


def format_preview(result, n=20):
    lines = [
        f"{'k':>4} {'t':>6} {'e_y_cm':>8} {'e_th':>8} {'om_cmd':>8} "
        f"{'om_r':>8} {'L':>5} {'R':>5} {'lat_cm':>8} {'seg':>6}"
    ]
    count = min(n, len(result["t"]))
    for k in range(count):
        if not np.isfinite(result["error"][k]):
            continue
        mode = "sharp" if result["sharp"][k] else "norm"
        lines.append(
            f"{k:4d} {result['t'][k]:6.2f} {result['e_y'][k]*100:8.3f} "
            f"{result['e_theta'][k]:8.3f} {result['omega_cmd'][k]:8.3f} "
            f"{result['omega_r'][k]:8.3f} {result['left_pwm'][k]:5.0f} "
            f"{result['right_pwm'][k]:5.0f} {result['lat'][k]*100:8.2f} {mode:>6}"
        )
    return "\n".join(lines)


def run_self_checks(cfg=None):
    cfg = CONFIG if cfg is None else cfg
    self_check_position_error(cfg)
    self_check_mpc_direction()


def _cell(value, fmt):
    if value is None:
        return ""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(v):
        return ""
    return format(v, fmt)


def write_timeseries(result, out_dir):
    """导出完整时序与轨迹，供后续与 PID 对比。"""
    csv_path = os.path.join(out_dir, "mpc_timeseries.csv")
    fields = [
        "t",
        "x",
        "y",
        "theta",
        "s",
        "lat",
        "dist",
        "error",
        "e_y",
        "e_theta",
        "sum_weight",
        "delta_u",
        "omega_cmd",
        "omega_r",
        "kappa_r",
        "v_r",
        "base_speed",
        "left_pwm",
        "right_pwm",
        "work_n",
        "sharp",
        "v",
        "omega",
        "v_left",
        "v_right",
        "solve_time_s",
    ]
    n = len(result["t"])
    with open(csv_path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for i in range(n):
            writer.writerow(
                [
                    _cell(result["t"][i], ".6f"),
                    _cell(result["x"][i], ".8f"),
                    _cell(result["y"][i], ".8f"),
                    _cell(result["theta"][i], ".8f"),
                    _cell(result["s"][i], ".8f"),
                    _cell(result["lat"][i], ".8f"),
                    _cell(result["dist"][i], ".8f"),
                    _cell(result["error"][i], ".8f"),
                    _cell(result["e_y"][i], ".8f"),
                    _cell(result["e_theta"][i], ".8f"),
                    _cell(result["sum_weight"][i], ".6f"),
                    _cell(result["delta_u"][i], ".6f"),
                    _cell(result["omega_cmd"][i], ".8f"),
                    _cell(result["omega_r"][i], ".8f"),
                    _cell(result["kappa_r"][i], ".8f"),
                    _cell(result["v_r"][i], ".8f"),
                    _cell(result["base_speed"][i], ".3f"),
                    _cell(result["left_pwm"][i], ".0f"),
                    _cell(result["right_pwm"][i], ".0f"),
                    _cell(result["work_n"][i], ".0f"),
                    int(bool(result["sharp"][i])),
                    _cell(result["v"][i], ".8f"),
                    _cell(result["omega"][i], ".8f"),
                    _cell(result["v_left"][i], ".8f"),
                    _cell(result["v_right"][i], ".8f"),
                    _cell(result["solve_time_s"][i], ".8f"),
                ]
            )
    return csv_path


def _fmt(value, digits=3):
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_metrics(metrics, path, out_dir, cfg):
    regions = metrics["regions"]
    lines = [
        "# MPC 仿真指标",
        "",
        "非控制器参数与 `tracking_car_sim` 完全一致；控制器为线性 MPC。",
        "",
        "## 总览",
        "",
        f"- 随机种子：{metrics['seed']}",
        f"- 控制周期：{metrics['dt']:.3f} s",
        f"- 仿真时长：{metrics['duration_s']:.2f} s，共 {metrics['steps']} 步",
        f"- QP 后端：{metrics.get('mpc_backend')}",
        f"- QP 求解时间 max/mean：{_fmt(metrics['qp_solve_time_ms_max'])} / "
        f"{_fmt(metrics['qp_solve_time_ms_mean'])} ms",
        f"- 最大 |error|：{_fmt(metrics['max_abs_error'])}",
        f"- RMS(error)：{_fmt(metrics['rms_error'])}",
        f"- 最大 |e_y|：{_fmt(metrics['max_abs_e_y'])} m",
        f"- RMS(e_y)：{_fmt(metrics['rms_e_y'])} m",
        f"- 全程最大横向偏离 |lat|：{_fmt(metrics['max_abs_lat_m'])} m",
        f"- 全程最大到线距离：{_fmt(metrics['max_dist_m'])} m",
        f"- 平均速度：{_fmt(metrics['mean_speed_mps'])} m/s",
        f"- 控制量变化率 RMS：{_fmt(metrics['control_rate_rms'])} rad/s²",
        f"- 丢线步数（Σweight = 0）：{metrics['line_loss_steps']}",
        f"- 丢线时长：{metrics['line_loss_time_s']:.2f} s",
        f"- 丢线次数：{metrics['line_loss_event_count']}",
        f"- 最长一次丢线：{metrics['line_loss_longest_s']:.2f} s",
        f"- 急弯/直角参考段累计时间：{metrics['sharp_mode_time_s']:.2f} s",
        f"- 是否因离线过远而结束：{metrics['stopped_off_line']}",
        f"- 急弯段最大 |error|：{_fmt(metrics['sharp_max_abs_error'])}",
        f"- 急弯段最大到线距离：{_fmt(metrics['sharp_max_dist_m'])} m",
        f"- 直角弯开始后的最大到线距离：{_fmt(metrics['corner_max_dist_m'])} m",
        "",
        "## 分段",
        "",
        "| 路段 | 样本数 | 最大 \\|error\\| | RMS(error) | 最大 \\|横向\\| / m | 最大到线距离 / m | 丢线步数 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    name = {
        "straight": "直道",
        "gentle": "缓弯 R=0.8 m",
        "s": "S 弯 R=0.6 m",
        "sharp": "急弯 R=0.25 m",
        "corner": "直角弯 R=0.025 m",
    }
    for key in ("straight", "gentle", "s", "sharp", "corner"):
        row = regions[key]
        lines.append(
            "| {name} | {n} | {e} | {rms} | {lat} | {dist} | {loss} |".format(
                name=name[key],
                n=row["samples"],
                e=_fmt(row["max_abs_error"]),
                rms=_fmt(row["rms_error"]),
                lat=_fmt(row["max_abs_lat_m"]),
                dist=_fmt(row["max_dist_m"]),
                loss=row["line_loss_steps"],
            )
        )
    lines.extend(["", "## 丢线事件", ""])
    if metrics["line_loss_events"]:
        lines.append("| 开始 / s | 结束 / s | 持续 / s | 步数 |")
        lines.append("| ---: | ---: | ---: | ---: |")
        for event in metrics["line_loss_events"]:
            lines.append(
                f"| {event['t_start']:.2f} | {event['t_end']:.2f} | "
                f"{event['duration']:.2f} | {event['steps']} |"
            )
    else:
        lines.append("本次没有连续丢线。")
    lines.extend(["", "## 参考路径", ""])
    lines.append("| 名称 | 类型 | 起点弧长 / m | 终点弧长 / m | 曲率 / m⁻¹ | 半径 / m |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: |")
    for seg in path.describe():
        radius = "inf" if seg["radius"] == float("inf") else f"{seg['radius']:.3f}"
        lines.append(
            f"| {seg['name']} | {seg['kind']} | {seg['s0']:.3f} | {seg['s1']:.3f} | "
            f"{seg['kappa']:.2f} | {radius} |"
        )
    lines.append("")
    md_path = os.path.join(out_dir, "mpc_metrics.md")
    with open(md_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))

    csv_path = os.path.join(out_dir, "mpc_metrics.csv")
    rows = [
        ("seed", metrics["seed"], "", "随机种子"),
        ("dt_s", metrics["dt"], "s", "控制周期"),
        ("duration_s", round(metrics["duration_s"], 4), "s", "仿真时长"),
        ("steps", metrics["steps"], "", "步数"),
        ("max_abs_error", metrics["max_abs_error"], "sensor_unit", "最大 |error|"),
        ("rms_error", metrics["rms_error"], "sensor_unit", "RMS(error)"),
        ("max_abs_lat_m", metrics["max_abs_lat_m"], "m", "全程最大 |横向误差|"),
        ("max_dist_m", metrics["max_dist_m"], "m", "全程最大到线距离"),
        ("line_loss_steps", metrics["line_loss_steps"], "", "Σweight=0 的步数"),
        ("line_loss_time_s", round(metrics["line_loss_time_s"], 4), "s", "丢线时长"),
        ("line_loss_event_count", metrics["line_loss_event_count"], "", "连续丢线次数"),
        ("line_loss_longest_s", round(metrics["line_loss_longest_s"], 4), "s", "最长丢线"),
        ("sharp_mode_time_s", round(metrics["sharp_mode_time_s"], 4), "s", "急弯模式时长"),
        ("sharp_mode_entries", metrics["sharp_mode_entries"], "", "进入急弯模式的次数"),
        ("sharp_max_abs_error", metrics["sharp_max_abs_error"], "sensor_unit", "急弯段最大 |error|"),
        ("sharp_max_dist_m", metrics["sharp_max_dist_m"], "m", "急弯段最大到线距离"),
        ("corner_max_dist_m", metrics["corner_max_dist_m"], "m", "直角弯开始后的最大到线距离"),
        ("stopped_off_line", int(metrics["stopped_off_line"]), "", "因离线结束"),
        ("mean_speed_mps", metrics["mean_speed_mps"], "m/s", "平均速度"),
        ("control_rate_rms", metrics["control_rate_rms"], "rad/s^2", "控制量变化率 RMS"),
        ("qp_solve_time_ms_max", metrics["qp_solve_time_ms_max"], "ms", "QP 单步最大求解时间"),
        ("qp_solve_time_ms_mean", metrics["qp_solve_time_ms_mean"], "ms", "QP 单步平均求解时间"),
    ]
    for key, row in regions.items():
        rows.append((f"{key}_max_abs_error", row["max_abs_error"], "sensor_unit", key))
        rows.append((f"{key}_rms_error", row["rms_error"], "sensor_unit", key))
        rows.append((f"{key}_max_dist_m", row["max_dist_m"], "m", key))
        rows.append((f"{key}_line_loss_steps", row["line_loss_steps"], "", key))
    with open(csv_path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value", "unit", "note"])
        writer.writerows(rows)


def _print_summary(metrics):
    print(
        f"时长 {metrics['duration_s']:.2f}s  "
        f"max|error|={metrics['max_abs_error']:.3f}  "
        f"RMS={metrics['rms_error']:.3f}"
    )
    print(
        f"丢线 {metrics['line_loss_event_count']} 次 / {metrics['line_loss_time_s']:.2f}s  "
        f"离线结束={metrics['stopped_off_line']}"
    )
    print(
        f"QP {metrics.get('mpc_backend')}  "
        f"max={metrics['qp_solve_time_ms_max']:.2f} ms  "
        f"mean={metrics['qp_solve_time_ms_mean']:.2f} ms"
    )
    for key, row in metrics["regions"].items():
        print(
            f"  {key:8} max|e|={row['max_abs_error']}  "
            f"rms={row['rms_error']}  "
            f"maxDist={row['max_dist_m']}  lossSteps={row['line_loss_steps']}"
        )


def main():
    print("=== MPC 自检 ===")
    run_self_checks(CONFIG)
    path = build_demo_path()
    print(f"路径长度 {path.total_length:.3f} m")
    for seg in path.describe():
        radius = "inf" if seg["radius"] == float("inf") else f"{seg['radius']:.3f}"
        print(f"  {seg['name']:6} {seg['kind']:8} R={radius:>6} s=[{seg['s0']:.2f},{seg['s1']:.2f}]")

    print("=== 闭环仿真 ===")
    print(
        f"N={MPC_CONFIG['N']}  Q={MPC_CONFIG['Q']}  R={MPC_CONFIG['R']}  "
        f"S={MPC_CONFIG['S']}  P={MPC_CONFIG['P']}"
    )
    result = simulate(path, CONFIG, MPC_CONFIG)
    print(format_preview(result, 20))
    metrics = compute_metrics(result, CONFIG, MPC_CONFIG)
    _print_summary(metrics)

    budget_ms = float(CONFIG["dt"]) * 1000.0 / 5.0
    dt_ms = float(CONFIG["dt"]) * 1000.0
    qmax = metrics["qp_solve_time_ms_max"]
    qmean = metrics["qp_solve_time_ms_mean"]
    if qmax is not None:
        print(f"QP 统计: max={qmax:.2f} ms  mean={qmean:.2f} ms  (dt={dt_ms:.0f} ms, dt/5={budget_ms:.0f} ms)")
        if qmax > dt_ms:
            print(f"[警告] QP 单步最大 {qmax:.2f} ms 超过控制周期 {dt_ms:.0f} ms")
        elif qmean > budget_ms:
            print(
                f"[说明] 平均求解 {qmean:.2f} ms > dt/5；已关闭软约束并预热 QP，"
                f"仍保留 N={MPC_CONFIG['N']}（降到 5 耗时几乎不变，预瞄更差）"
            )

    data_dir = str(ROOT / "results" / "data")
    fig_dir = str(ROOT / "results" / "figures")
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(fig_dir, exist_ok=True)
    write_metrics(metrics, path, data_dir, CONFIG)
    ts_path = write_timeseries(result, data_dir)
    use_zh = configure_font()
    print("中文字体" if use_zh else "字体回退为英文标签")
    print("正在写曲线和 GIF ...")
    render_all(result, metrics, fig_dir, CONFIG, use_zh=use_zh)
    print(f"数据目录 {data_dir}")
    print(f"图片目录 {fig_dir}")
    print(f"时序 CSV: {ts_path}")
    print("文件: mpc_simulation.gif, mpc_error_vs_time.png, mpc_trajectory.png,")
    print("      mpc_control_signals.png, mpc_metrics.csv, mpc_metrics.md, mpc_timeseries.csv")


if __name__ == "__main__":
    main()
