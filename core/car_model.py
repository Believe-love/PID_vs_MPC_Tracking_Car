# -*- coding: utf-8 -*-
"""车辆运动学、PWM 映射、传感器模拟量和加权误差。

对应关系：

- ``position_error`` ← ``PIDtracking.ino`` 的 ``calculatePositionError()``
- 轮速、独轮车模型和模拟量发生器在实物源码里不存在，是仿真补上的被控对象

误差符号（源码注释与博客一致）：

- error > 0：车偏左，需要右转
- 因此 ``SENSOR_X`` 的正方向放在车体右侧
- 车体坐标仍用“前向 x、左侧 y 为正”，传感器横向坐标取 ``y = -scale * SENSOR_X``
"""

import numpy as np

from config.params import CONFIG, pwm_to_speed_scale


def position_error(analogs, cfg=None):
    """加权平均横向误差，逻辑与 ``calculatePositionError`` 相同。

    模拟值 < 250 时权重取 ``replace_weight``（源码里是 0，不是注释里的 10）。
    Σweight == 0 时返回 0。这就是丢线后误差被清零的行为。

    返回 ``(error, sum_weight)``。
    """
    cfg = CONFIG if cfg is None else cfg
    sensor_x = np.asarray(cfg["sensor_x"], dtype=float)
    values = np.asarray(analogs, dtype=float)
    if values.shape != sensor_x.shape:
        raise ValueError("模拟量路数必须与 SENSOR_X 一致")
    weight = np.where(values < cfg["sensor_threshold"], cfg["replace_weight"], values)
    sum_w = float(np.sum(weight))
    if sum_w == 0.0:
        return 0.0, 0.0
    error = float(np.sum(sensor_x * weight) / sum_w)
    return error, sum_w


def pwm_to_wheel_speed(pwm, cfg=None):
    """线性 PWM→轮速。PWM 低于死区时该轮速度为 0。

    高于死区后速度 = (0.30/100) * PWM，因此 PWM=100 时为 0.30 m/s。
    死区边界处速度从 0 跳到约 0.09 m/s，这是“过原点的直线 + 硬死区”的直接结果。
    """
    cfg = CONFIG if cfg is None else cfg
    if pwm < cfg["pwm_deadzone"]:
        return 0.0
    return pwm_to_speed_scale(cfg) * float(pwm)


def lag_wheel_speed(speed, speed_cmd, dt, tau):
    """轮速一阶滞后的精确离散：v ← v_cmd + (v - v_cmd) e^{-dt/τ}。

    τ<=0 时直接取目标轮速。稳态增益不变，仍然是 ``pwm_to_wheel_speed``。
    """
    if tau is None or tau <= 1e-6:
        return float(speed_cmd)
    alpha = float(np.exp(-dt / tau))
    return float(speed_cmd + (speed - speed_cmd) * alpha)


def body_velocity(v_left, v_right, cfg=None):
    """差速模型：v = (v_R + v_L) / 2，ω = (v_R - v_L) / L。

    ω > 0 为逆时针（左转）。u > 0 时左轮更快，ω < 0，车向右转，
    用来纠正 error > 0（车偏左）。
    """
    cfg = CONFIG if cfg is None else cfg
    v = 0.5 * (v_right + v_left)
    omega = (v_right - v_left) / float(cfg["wheel_base"])
    return v, omega


def rk4_unicycle(x, y, theta, v, omega, dt):
    """独轮车模型的 RK4 一步。本步内 v、ω 保持为刚算出的轮速，步长等于控制周期。"""

    def deriv(th):
        return (v * np.cos(th), v * np.sin(th), omega)

    k1 = deriv(theta)
    k2 = deriv(theta + 0.5 * dt * k1[2])
    k3 = deriv(theta + 0.5 * dt * k2[2])
    k4 = deriv(theta + dt * k3[2])
    x2 = x + (dt / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
    y2 = y + (dt / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    th2 = theta + (dt / 6.0) * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2])
    return float(x2), float(y2), float(th2)


def sensor_scale(cfg=None):
    """SENSOR_X 相对坐标到米的比例。跨度 7 对应 sensor_span。"""
    cfg = CONFIG if cfg is None else cfg
    sensor_x = np.asarray(cfg["sensor_x"], dtype=float)
    span_units = float(sensor_x[-1] - sensor_x[0])
    return float(cfg["sensor_span"]) / span_units


def sensor_world_positions(x, y, theta, cfg=None):
    """8 路传感器在世界坐标中的位置，shape (8, 2)。

    传感器杆在车轴前方 ``sensor_lookahead`` 处，沿车体横向等间距排列。
    """
    cfg = CONFIG if cfg is None else cfg
    sensor_x = np.asarray(cfg["sensor_x"], dtype=float)
    y_body = -sensor_scale(cfg) * sensor_x
    forward = np.array([np.cos(theta), np.sin(theta)])
    left = np.array([-np.sin(theta), np.cos(theta)])
    origin = np.array([x, y], dtype=float) + float(cfg["sensor_lookahead"]) * forward
    return origin + y_body[:, None] * left


def simulate_analogs(distances, rng, cfg=None):
    """由各传感器到黑线中心的距离生成模拟量。

    高斯型：中心约 ``analog_peak``，远离后迅速落到 0 附近，再叠加高斯噪声并截断。
    这不是传感器数据手册里的标定曲线，只为复现“黑大白小、0~8000”这一量级。
    """
    cfg = CONFIG if cfg is None else cfg
    dist = np.asarray(distances, dtype=float)
    sigma = float(cfg["analog_sigma"])
    raw = float(cfg["analog_peak"]) * np.exp(-0.5 * (dist / sigma) ** 2)
    noisy = raw + rng.normal(0.0, float(cfg["analog_noise_std"]), size=dist.shape)
    return np.clip(noisy, cfg["analog_clip_min"], cfg["analog_clip_max"])


def self_check_position_error(cfg=None):
    """用几个静态传感器向量核对误差公式。"""
    err, sum_w = position_error([0, 0, 0, 0, 0, 0, 0, 0], cfg)
    assert err == 0.0 and sum_w == 0.0

    err, _ = position_error([0, 0, 0, 0, 3000, 0, 0, 0], cfg)
    assert abs(err - 0.5) < 1e-12

    err, _ = position_error([0, 0, 0, 4000, 4000, 0, 0, 0], cfg)
    assert abs(err) < 1e-12

    # 100 < 250，权重为 0；只剩 SENSOR_X = 3.5 那一路
    err, _ = position_error([100, 0, 0, 0, 0, 0, 0, 5000], cfg)
    assert abs(err - 3.5) < 1e-12

    # 源码判断是“小于阈值”，等于 250 时仍使用原值
    err, sum_w = position_error([0, 0, 0, 0, 250, 0, 0, 0], cfg)
    assert abs(err - 0.5) < 1e-12 and abs(sum_w - 250.0) < 1e-12

    dead = pwm_to_wheel_speed(29, cfg)
    nominal = pwm_to_wheel_speed(100, cfg)
    assert dead == 0.0
    assert abs(nominal - 0.30) < 1e-12
