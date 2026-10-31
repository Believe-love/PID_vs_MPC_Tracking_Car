# -*- coding: utf-8 -*-
"""仿真参数集中配置。

PID 增益、阈值、限幅和急弯切换系数严格对应
``PIDtracking.ino`` 中的可执行代码，不在这里做“美化调参”。

运动学和传感器映射是实物源码里没有的部分，集中放在本文件，便于对照假设。
"""

CONFIG = {
    # ------------------------------------------------------------------
    # 控制周期
    # 实物 loop() 由串口数据包到达驱动，没有固定 delay。
    # PRINT_INTERVAL = 10 只控制打印，不控制采样。
    # 仿真默认 20 ms（50 Hz），与任务说明中的采样节奏一致。
    # ------------------------------------------------------------------
    "dt": 0.02,
    "seed": 42,

    # ------------------------------------------------------------------
    # 增量式 PID（PIDtracking.ino）
    # 注释写明 Ki、Kd 已融入采样周期 T，公式里不再乘/除 dt。
    # ------------------------------------------------------------------
    "Kp": 33.0,
    "Ki": 0.0,
    "Kd": 10.0,
    "u_min": -150.0,
    "u_max": 150.0,

    # 传感器加权误差（calculatePositionError）
    "sensor_x": (-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5),
    "sensor_threshold": 250.0,
    # 源码注释写“小于 250 则权重为 10”，但 replace_weight 实际为 0。
    # 以可执行代码为准。
    "replace_weight": 0.0,

    # 急弯判据与双参数行为（loop 内，不是第二组 Kp/Ki/Kd）
    "work_sensor_analog_min": 1000.0,
    "work_sensor_count_thr": 3,
    "error_sharp_thr": 2.0,
    "sharp_u_gain": 1.5,
    "base_pwm": 100.0,
    "base_low_pwm": 60.0,
    "pwm_min": 0.0,
    "pwm_max": 255.0,

    # ------------------------------------------------------------------
    # 车辆运动学（源码未给出，按任务说明取值）
    # ------------------------------------------------------------------
    "wheel_base": 0.16,            # m，左右轮距 L
    "speed_at_pwm_100": 0.30,      # m/s，PWM=100 且高于死区时的稳态线速度
    "pwm_deadzone": 30.0,          # PWM 低于此值时该轮目标速度为 0
    # 轮速一阶滞后。稳态仍是上面的线性映射。
    # τ=0 时一个控制周期内轮速就跳到目标值，运动学最小转弯半径约 3 cm，
    # 直角弯也会被跟住，和博客里“急弯仍会丢线”不符。τ 只描述电机/车身跟不上 PWM。
    "wheel_speed_tau": 0.18,       # s。直道仍能稳住；直角弯上来不及把航向转过 90°

    # ------------------------------------------------------------------
    # 传感器几何与模拟量模型（简化，不是厂家标定曲线）
    # SENSOR_X 从 -3.5 到 3.5，总跨度 7 个相对单位，对应 0.12 m。
    # 正的 SENSOR_X 在车体右侧：error>0 表示线在右侧、车偏左。
    # ------------------------------------------------------------------
    "sensor_span": 0.12,           # m，最外侧两路传感器中心间距
    "sensor_lookahead": 0.075,     # m，传感器杆相对驱动轴中心的前视距离
    "analog_peak": 8000.0,         # 压在黑线中心时的模拟量
    "analog_sigma": 0.0082,        # m，高斯响应标准差（线宽+光斑的等效宽度）
    "analog_noise_std": 120.0,     # 模拟量高斯噪声标准差
    "analog_clip_min": 0.0,
    "analog_clip_max": 8000.0,

    # 丢线事件：连续若干步 Σweight == 0
    "line_loss_min_steps": 3,

    # 初始位姿相对路径起点：横向为正表示车在路径左侧
    "initial_lateral_offset": 0.008,  # m
    "initial_heading_error": 0.0,     # rad

    # 仿真时长不足时按路径长度自动估计；车辆远离路径后提前结束
    "sim_time": None,
    "speed_for_time_estimate": 0.20,  # m/s，用来估计最长仿真时间
    "sim_time_margin": 2.5,           # s
    "depart_distance": 0.55,          # m，连续远离超过该距离则开始计离线
    "depart_hold": 1.6,               # s
}


def pwm_to_speed_scale(cfg=None):
    """PWM 到轮速的线性系数，单位 m/s 每 PWM。死区以下另处理。"""
    cfg = CONFIG if cfg is None else cfg
    return float(cfg["speed_at_pwm_100"]) / 100.0


def sensor_unit_to_meters(cfg=None):
    """SENSOR_X 一个单位对应的横向米数。跨度 7 对应 sensor_span。"""
    cfg = CONFIG if cfg is None else cfg
    sensor_x = cfg["sensor_x"]
    span_units = float(sensor_x[-1] - sensor_x[0])
    return float(cfg["sensor_span"]) / span_units


def derive_omega_limits(cfg=None):
    """由左右轮 PWM 上下限反推 |ω| 可行域。

    一侧轮速最大（PWM=255）、另一侧为 0 时 |ω| = v_max / L。
    """
    cfg = CONFIG if cfg is None else cfg
    scale = pwm_to_speed_scale(cfg)
    v_max = scale * float(cfg["pwm_max"])
    omega_max = v_max / float(cfg["wheel_base"])
    return -omega_max, omega_max


# ------------------------------------------------------------------
# MPC 专属参数（仿真环境参数在 CONFIG，未改动）
# ------------------------------------------------------------------
_OMEGA_MIN, _OMEGA_MAX = derive_omega_limits(CONFIG)
_E_Y_MAX = float(CONFIG["error_sharp_thr"]) * sensor_unit_to_meters(CONFIG)
# abs(error)>=2 触发急弯：2 * (0.12/7) ≈ 0.034286 m

MPC_CONFIG = {
    "N": 10,                       # 预测/控制时域；关闭软约束后单步约 2–4 ms，保留 N=10
    # 初值 Q=(10,1) 对米制 e_y 过弱（0.02^2*10≪R*du^2）；按米制重标定如下
    "Q": (5000.0, 80.0),           # diag(Q)，横向权重大于航向
    "R": 0.1,                      # delta_u 权重
    "S": 1.0,                      # 控制增量突变惩罚（急弯关键旋钮）
    "P": (10000.0, 200.0),         # 终端代价，经验值，未做严格稳定性证明
    "omega_min": _OMEGA_MIN,       # ≈ -4.781 rad/s
    "omega_max": _OMEGA_MAX,       # ≈  4.781 rad/s
    "delta_omega_max": 0.5 * _OMEGA_MAX,  # 角速度变化率约束初值
    "e_y_max": _E_Y_MAX,           # 软约束边界（图上标注；硬 QP 默认关闭软约束以加速）
    "soft_constraint_M": 1.0e4,    # 软约束松弛罚
    "use_soft_ey": False,          # True 时约 +3–5 ms/步；图中仍画 e_y_max
    # 参考速度：与 PID base_pwm=100/60 经同一 PWM 映射反推
    "v_high": float(CONFIG["speed_at_pwm_100"]),                          # 0.30
    "v_low": pwm_to_speed_scale(CONFIG) * float(CONFIG["base_low_pwm"]),  # 0.18
    # 航向偏差噪声（理想化：真值 + 噪声；实物 8 路红外测不到航向）
    # 量级取与模拟量噪声相当的小角扰动
    "e_theta_noise_std": 0.01,     # rad（过大时直道 PWM 易抖）
    # 第一版：预测时域内 A、B 固定为当前步线性化，不随预瞄点变化
    "freeze_linearization": True,
}
