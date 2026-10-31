# -*- coding: utf-8 -*-
"""PID 闭环仿真入口。

闭环顺序、增量式 PID、急弯分支和运动学积分与原 tracking_car_sim 相同。
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
    sensor_world_positions,
    simulate_analogs,
)
from config.params import CONFIG
from controllers.pid import TrackingController, self_check_controller
from core.path import build_demo_path
from simulators.visualize_pid import configure_font, render_all


def initial_pose(path, cfg):
    """路径起点沿左法向偏移，heading 可再加一个小偏差。"""
    x0, y0, h0, _ = path.pose(0.0)
    lateral = float(cfg["initial_lateral_offset"])
    x = x0 + lateral * (-np.sin(h0))
    y = y0 + lateral * np.cos(h0)
    theta = h0 + float(cfg["initial_heading_error"])
    return float(x), float(y), float(theta)


def simulate(path=None, cfg=None):
    """跑完整条路径，返回时序数组。"""
    cfg = CONFIG if cfg is None else cfg
    path = build_demo_path() if path is None else path
    rng = np.random.default_rng(int(cfg["seed"]))
    controller = TrackingController(cfg)
    dt = float(cfg["dt"])

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
    # 进入循迹时小车已经在以基础 PWM 巡航，滞后只作用在随后的差速修正上。
    cruise = pwm_to_wheel_speed(cfg["base_pwm"], cfg)
    v_left_state = cruise
    v_right_state = cruise
    tau = float(cfg["wheel_speed_tau"])
    end_margin = float(cfg["sensor_lookahead"]) + 0.08

    while True:
        proj = path.project(x, y, hint_s=hint_s)
        hint_s = proj["s"]
        sensor_xy = sensor_world_positions(x, y, theta, cfg)
        distances = np.empty(8, dtype=float)
        for i in range(8):
            sp = path.project(sensor_xy[i, 0], sensor_xy[i, 1], hint_s=hint_s)
            distances[i] = sp["euclid"]
        analogs = simulate_analogs(distances, rng, cfg)
        error, sum_w = position_error(analogs, cfg)
        cmd = controller.step(analogs, error)

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
                "sum_weight": sum_w,
                "analogs": analogs.copy(),
                "distances": distances.copy(),
                "v": v,
                "omega": omega,
                "v_left": v_left,
                "v_right": v_right,
                **cmd,
            }
        )

        # 传感器在车头前方。轴心走到终点前就停，避免传感器冲出黑线末端被记成丢线。
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
            # 再记一步离线后的位姿，让轨迹能画出“已经离开”而不是停在判据刚触发的点
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
                    "sum_weight": np.nan,
                    "analogs": np.full(8, np.nan),
                    "distances": np.full(8, np.nan),
                    "v": v,
                    "omega": omega,
                    "v_left": v_left,
                    "v_right": v_right,
                    "u_pid": np.nan,
                    "delta_u": np.nan,
                    "u_cmd": np.nan,
                    "base_speed": np.nan,
                    "left_pwm": np.nan,
                    "right_pwm": np.nan,
                    "work_n": np.nan,
                    "sharp": False,
                }
            )
            break

    result = _rows_to_arrays(rows)
    result["path"] = path
    result["stopped_off_line"] = bool(off_time >= float(cfg["depart_hold"]))
    result["t_max"] = t_max
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
        "sum_weight",
        "v",
        "omega",
        "v_left",
        "v_right",
        "u_pid",
        "delta_u",
        "u_cmd",
        "base_speed",
        "left_pwm",
        "right_pwm",
        "work_n",
    ]
    out = {k: np.asarray([r[k] for r in rows], dtype=float) for k in keys}
    out["sharp"] = np.asarray([bool(r["sharp"]) for r in rows], dtype=bool)
    out["analogs"] = np.vstack([np.asarray(r["analogs"], dtype=float) for r in rows])
    out["distances"] = np.vstack([np.asarray(r["distances"], dtype=float) for r in rows])
    return out


def _segment_window(seg, cfg):
    """统计和涂色用的弧长窗口。

    直角弯的传感器在车头，轴心还没进弯道时误差已经起来，所以窗口向前扩一个前视距离。
    涂色时再要求车还没跑远，避免丢线后投影点粘在弯道上、把后面整段都涂成直角弯。
    """
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
        # 直道不按距离截断，但要避开直角弯前视窗口，否则进弯峰值会被算进直道。
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
        # 丢线后投影会落到下一段直道上，那些点不能算进“直道贴线”的误差。
        mask &= result["dist"] <= 0.05
        mask &= np.isfinite(result["error"])
    return mask


def time_spans(result, kind, cfg):
    """某种路段在时间轴上的连续区间，供误差图和控制图涂色。"""
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


def _line_loss_events(result, cfg):
    """连续至少 N 步 Σweight == 0 记为一次丢线。"""
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


def compute_metrics(result, cfg=None):
    """汇总验收里要求的误差、丢线和急弯偏差。"""
    cfg = CONFIG if cfg is None else cfg
    path = result["path"]
    err = result["error"]
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

    metrics = {
        "dt": dt,
        "seed": int(cfg["seed"]),
        "duration_s": float(result["t"][-1]),
        "steps": int(len(result["t"])),
        "max_abs_error": float(np.max(abs_e)) if len(abs_e) else None,
        "rms_error": float(np.sqrt(np.mean(err_v ** 2))) if len(err_v) else None,
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
        "spans": {
            "sharp": time_spans(result, "sharp", cfg),
            "corner": time_spans(result, "corner", cfg),
        },
        "regions": {
            "straight": region_stats("straight"),
            "gentle": region_stats("gentle"),
            "s": region_stats("s"),
            "sharp": region_stats("sharp"),
            "corner": region_stats("corner"),
        },
    }
    # 直角弯窗口为了涂色截断了距离，这里另记丢线之后车离开中心线的最大距离。
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
    """打印开头若干拍的 error / u / base，用来确认直道在收敛。"""
    lines = [
        f"{'k':>4} {'t':>6} {'error':>8} {'u_pid':>8} {'u_cmd':>8} "
        f"{'base':>6} {'L':>5} {'R':>5} {'lat_cm':>8} {'mode':>6}"
    ]
    count = min(n, len(result["t"]))
    for k in range(count):
        if not np.isfinite(result["error"][k]):
            continue
        mode = "sharp" if result["sharp"][k] else "norm"
        lines.append(
            f"{k:4d} {result['t'][k]:6.2f} {result['error'][k]:8.3f} "
            f"{result['u_pid'][k]:8.2f} {result['u_cmd'][k]:8.2f} "
            f"{result['base_speed'][k]:6.0f} {result['left_pwm'][k]:5.0f} "
            f"{result['right_pwm'][k]:5.0f} {result['lat'][k] * 100:8.2f} {mode:>6}"
        )
    return "\n".join(lines)


def self_check_sensor_sign(cfg=None):
    """车在直道左侧时，无噪声误差必须为正。这是源码约定的符号。"""
    from core.path import ReferencePath, Segment

    cfg = CONFIG if cfg is None else cfg
    path = ReferencePath([Segment("straight", "直道", 2.0, 0.0)])
    # 固定“无噪声”读数：把噪声标准差临时视为 0，用独立 rng 也不会加噪声
    rng = np.random.default_rng(0)
    local_cfg = dict(cfg)
    local_cfg["analog_noise_std"] = 0.0
    x, y, theta = 0.40, 0.015, 0.0  # 路径在 y=0，车在左侧 1.5 cm
    hint = path.project(x, y)["s"]
    sensor_xy = sensor_world_positions(x, y, theta, local_cfg)
    dist = np.array(
        [
            path.project(sensor_xy[i, 0], sensor_xy[i, 1], hint_s=hint)["euclid"]
            for i in range(8)
        ]
    )
    analogs = simulate_analogs(dist, rng, local_cfg)
    err, _ = position_error(analogs, local_cfg)
    if err <= 0.0:
        raise AssertionError(f"车在路径左侧时误差应为正，实际 error={err}")


def run_self_checks(cfg=None):
    cfg = CONFIG if cfg is None else cfg
    self_check_position_error(cfg)
    self_check_controller(cfg)
    self_check_sensor_sign(cfg)


def _fmt(value, digits=3):
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_metrics(metrics, path, out_dir, cfg):
    """写 metrics.md 和 metrics.csv。"""
    regions = metrics["regions"]
    lines = [
        "# 仿真指标",
        "",
        "控制律与 `PIDtracking.ino` 一致，没有为了把图画好看而改 Kp / Ki / Kd。",
        "误差 `error` 是 8 路传感器加权结果，单位是源码里的 `SENSOR_X` 坐标，不是米。",
        "",
        "## 总览",
        "",
        f"- 随机种子：{metrics['seed']}",
        f"- 控制周期：{metrics['dt']:.3f} s",
        f"- 仿真时长：{metrics['duration_s']:.2f} s，共 {metrics['steps']} 步",
        f"- 最大 |error|：{_fmt(metrics['max_abs_error'])}",
        f"- RMS(error)：{_fmt(metrics['rms_error'])}",
        f"- 全程最大横向偏离 |lat|：{_fmt(metrics['max_abs_lat_m'])} m",
        f"- 全程最大到线距离：{_fmt(metrics['max_dist_m'])} m",
        f"- 丢线步数（Σweight = 0）：{metrics['line_loss_steps']}",
        f"- 丢线时长：{metrics['line_loss_time_s']:.2f} s",
        f"- 丢线次数（连续 ≥ {cfg['line_loss_min_steps']} 步）：{metrics['line_loss_event_count']}",
        f"- 最长一次丢线：{metrics['line_loss_longest_s']:.2f} s",
        f"- 急弯模式累计时间：{metrics['sharp_mode_time_s']:.2f} s，进入 {metrics['sharp_mode_entries']} 次",
        f"- 是否因离线过远而结束：{metrics['stopped_off_line']}",
        f"- 急弯段最大 |error|：{_fmt(metrics['sharp_max_abs_error'])}",
        f"- 急弯段最大到线距离：{_fmt(metrics['sharp_max_dist_m'])} m",
        f"- 直角弯开始后的最大到线距离：{_fmt(metrics['corner_max_dist_m'])} m",
        "",
        "丢线定义：某一拍 8 路模拟值都低于 250，从而 Σweight = 0，`calculatePositionError` 返回 0。",
        "连续达到 `line_loss_min_steps` 拍才计为一次事件，避免把单拍噪声算进去。",
        "",
        "## 分段",
        "",
        "直道统计去掉了直角弯前视窗口，也去掉了丢线后距离超过 5 cm 的点。",
        "否则车已经离开黑线、投影却落在下一段直道上，直道误差会被直角弯污染。",
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
    md_path = os.path.join(out_dir, "pid_metrics.md")
    with open(md_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))

    csv_path = os.path.join(out_dir, "pid_metrics.csv")
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


def write_timeseries(result, out_dir):
    """导出完整时序与轨迹，供后续方法对比使用。"""
    csv_path = os.path.join(out_dir, "pid_timeseries.csv")
    fields = [
        "t",
        "x",
        "y",
        "theta",
        "s",
        "lat",
        "dist",
        "error",
        "sum_weight",
        "u_pid",
        "u_cmd",
        "delta_u",
        "base_speed",
        "left_pwm",
        "right_pwm",
        "work_n",
        "sharp",
        "v",
        "omega",
        "v_left",
        "v_right",
    ]
    n = len(result["t"])
    with open(csv_path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for i in range(n):
            writer.writerow(
                [
                    f"{float(result['t'][i]):.6f}",
                    f"{float(result['x'][i]):.8f}",
                    f"{float(result['y'][i]):.8f}",
                    f"{float(result['theta'][i]):.8f}",
                    f"{float(result['s'][i]):.8f}",
                    f"{float(result['lat'][i]):.8f}",
                    f"{float(result['dist'][i]):.8f}",
                    "" if not math.isfinite(float(result["error"][i])) else f"{float(result['error'][i]):.8f}",
                    "" if not math.isfinite(float(result["sum_weight"][i])) else f"{float(result['sum_weight'][i]):.6f}",
                    "" if not math.isfinite(float(result["u_pid"][i])) else f"{float(result['u_pid'][i]):.6f}",
                    "" if not math.isfinite(float(result["u_cmd"][i])) else f"{float(result['u_cmd'][i]):.6f}",
                    "" if not math.isfinite(float(result["delta_u"][i])) else f"{float(result['delta_u'][i]):.6f}",
                    "" if not math.isfinite(float(result["base_speed"][i])) else f"{float(result['base_speed'][i]):.3f}",
                    "" if not math.isfinite(float(result["left_pwm"][i])) else f"{float(result['left_pwm'][i]):.0f}",
                    "" if not math.isfinite(float(result["right_pwm"][i])) else f"{float(result['right_pwm'][i]):.0f}",
                    "" if not math.isfinite(float(result["work_n"][i])) else f"{float(result['work_n'][i]):.0f}",
                    int(bool(result["sharp"][i])),
                    f"{float(result['v'][i]):.8f}",
                    f"{float(result['omega'][i]):.8f}",
                    f"{float(result['v_left'][i]):.8f}",
                    f"{float(result['v_right'][i]):.8f}",
                ]
            )
    return csv_path


def _print_summary(metrics):
    print(
        f"时长 {metrics['duration_s']:.2f}s  "
        f"max|error|={metrics['max_abs_error']:.3f}  "
        f"RMS={metrics['rms_error']:.3f}"
    )
    print(
        f"丢线 {metrics['line_loss_event_count']} 次 / {metrics['line_loss_time_s']:.2f}s  "
        f"急弯模式 {metrics['sharp_mode_time_s']:.2f}s  "
        f"离线结束={metrics['stopped_off_line']}"
    )
    for key, row in metrics["regions"].items():
        print(
            f"  {key:8} max|e|={row['max_abs_error']}  "
            f"rms={row['rms_error']}  "
            f"maxDist={row['max_dist_m']}  lossSteps={row['line_loss_steps']}"
        )
    print(f"直角弯后最大偏离 {metrics['corner_max_dist_m']} m")
    print(f"色带 急弯 {metrics['spans']['sharp']}")
    print(f"色带 直角弯 {metrics['spans']['corner']}")


def main():
    run_self_checks(CONFIG)
    path = build_demo_path()
    print(f"路径长度 {path.total_length:.3f} m")
    for seg in path.describe():
        radius = "inf" if seg["radius"] == float("inf") else f"{seg['radius']:.3f}"
        print(f"  {seg['name']:6} {seg['kind']:8} R={radius:>6} s=[{seg['s0']:.2f},{seg['s1']:.2f}]")

    result = simulate(path, CONFIG)
    print(format_preview(result, 20))
    metrics = compute_metrics(result, CONFIG)
    _print_summary(metrics)

    data_dir = str(ROOT / "results" / "data")
    fig_dir = str(ROOT / "results" / "figures")
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(fig_dir, exist_ok=True)
    write_metrics(metrics, path, data_dir, CONFIG)
    ts_path = write_timeseries(result, data_dir)
    print(f"时序 CSV {ts_path}")
    use_zh = configure_font()
    print("中文字体" if use_zh else "字体回退为英文标签")
    print("正在写曲线和 GIF ...")
    render_all(result, metrics, fig_dir, CONFIG, use_zh=use_zh)
    print(f"数据目录 {data_dir}")
    print(f"图片目录 {fig_dir}")
    print(f"路径总长 {path.total_length:.3f} m")


if __name__ == "__main__":
    main()
