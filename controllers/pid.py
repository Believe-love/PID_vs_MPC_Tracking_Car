# -*- coding: utf-8 -*-
"""增量式 PID 与急弯切换。

对应 ``PIDtracking.ino``：

- ``calculatePositionError()`` 的误差在 ``car_model.position_error``
- ``pid_calc()``：本文件 ``IncrementalPID.update``
- ``loop()`` 里 ``workSensor_num`` / ``u_now *= 1.5`` / ``base_speed`` 切换
- ``pid_tracking()``：左右 PWM 映射

急弯分支只放大“本周期送去电机的 u”，不写回 ``u_k``。
下一周期仍从未经 1.5 倍的 ``u_k`` 上累加增量。这与源码一致：
``u_now`` 是 ``pid_calc`` 的返回值副本，``u_k`` 只在 ``pid_calc`` 内部更新。
"""

from config.params import CONFIG


def constrain(value, low, high):
    """对应 Arduino ``constrain`` 宏。"""
    if value < low:
        return low
    if value > high:
        return high
    return value


class IncrementalPID:
    """增量式 PID，状态变量与源码全局量一一对应。

    ``e_k_1``、``e_k_2``、``u_k`` 初值均为 0。
    ``Ki`` 保持为 0，不额外把采样周期乘进积分项。
    """

    def __init__(self, cfg=None):
        cfg = CONFIG if cfg is None else cfg
        self.kp = float(cfg["Kp"])
        self.ki = float(cfg["Ki"])
        self.kd = float(cfg["Kd"])
        self.u_min = float(cfg["u_min"])
        self.u_max = float(cfg["u_max"])
        self.reset()

    def reset(self):
        self.e_k_1 = 0.0
        self.e_k_2 = 0.0
        self.u_k = 0.0
        self.delta_u = 0.0

    def update(self, e_k):
        """执行一次 ``pid_calc(e_k)``，返回限幅后的 ``u_k``。"""
        self.delta_u = (
            self.kp * (e_k - self.e_k_1)
            + self.ki * e_k
            + self.kd * (e_k - 2.0 * self.e_k_1 + self.e_k_2)
        )
        self.u_k = constrain(self.u_k + self.delta_u, self.u_min, self.u_max)
        # 先用旧的历史计算增量，再滚动。与源码语句顺序一致。
        self.e_k_2 = self.e_k_1
        self.e_k_1 = e_k
        return self.u_k


class TrackingController:
    """一次控制周期：误差 → PID → 急弯切换 → 左右 PWM。"""

    def __init__(self, cfg=None):
        self.cfg = CONFIG if cfg is None else cfg
        self.pid = IncrementalPID(self.cfg)

    def reset(self):
        self.pid.reset()

    def apply_sharp_policy(self, u_pid, error, work_n):
        """复现 loop() 中的急弯分支。不修改 ``pid.u_k``。"""
        cfg = self.cfg
        sharp = (work_n >= cfg["work_sensor_count_thr"]) or (
            abs(error) >= cfg["error_sharp_thr"]
        )
        if sharp:
            u_cmd = constrain(
                u_pid * cfg["sharp_u_gain"], cfg["u_min"], cfg["u_max"]
            )
            base = float(cfg["base_low_pwm"])
        else:
            u_cmd = u_pid
            base = float(cfg["base_pwm"])
        return u_cmd, base, sharp

    def mix_pwm(self, u_cmd, base_speed):
        """对应 ``pid_tracking``：左轮 base+u，右轮 base-u，再限幅到 0~255。

        源码左电机 PWM 写在后退引脚、右电机写在前进引脚，但两轮都朝前转。
        这里只保留差速数值，接线方向不进入仿真。
        ``analogWrite`` 接收 int，正数从 float 截断（向 0 取整）。
        """
        cfg = self.cfg
        left = constrain(base_speed + u_cmd, cfg["pwm_min"], cfg["pwm_max"])
        right = constrain(base_speed - u_cmd, cfg["pwm_min"], cfg["pwm_max"])
        return int(left), int(right)

    def step(self, analogs, error):
        """已知本周期模拟量和误差，算出电机指令。

        误差由调用方用 ``position_error`` 先算好，便于记录和单测。
        ``workSensor_num`` 统计的是原始模拟值 > 1000 的路数，不是权重。
        """
        cfg = self.cfg
        u_pid = self.pid.update(error)
        work_n = int(sum(a > cfg["work_sensor_analog_min"] for a in analogs))
        u_cmd, base, sharp = self.apply_sharp_policy(u_pid, error, work_n)
        left_pwm, right_pwm = self.mix_pwm(u_cmd, base)
        return {
            "error": float(error),
            "u_pid": float(u_pid),
            "delta_u": float(self.pid.delta_u),
            "u_cmd": float(u_cmd),
            "base_speed": float(base),
            "left_pwm": left_pwm,
            "right_pwm": right_pwm,
            "work_n": work_n,
            "sharp": bool(sharp),
        }


def self_check_controller(cfg=None):
    """用手算结果核对 PID 与急弯分支，失败则抛 AssertionError。"""
    cfg = CONFIG if cfg is None else cfg
    pid = IncrementalPID(cfg)
    # e 从 0 跳到 1 并保持：
    # 第 1 拍 delta = 33*(1-0) + 10*(1-0+0) = 43
    u1 = pid.update(1.0)
    assert abs(u1 - 43.0) < 1e-9, u1
    # 第 2 拍 delta = 10*(1 - 2*1 + 0) = -10，u = 33
    u2 = pid.update(1.0)
    assert abs(u2 - 33.0) < 1e-9, u2
    # 第 3 拍微分项为 0，稳态 u = Kp*e = 33（Ki = 0）
    u3 = pid.update(1.0)
    assert abs(u3 - 33.0) < 1e-9, u3

    # 限幅：大误差一步打满
    pid.reset()
    u_sat = pid.update(10.0)
    # delta = 33*10 + 10*10 = 430，限幅到 150
    assert abs(u_sat - 150.0) < 1e-9, u_sat

    ctl = TrackingController(cfg)
    u_cmd, base, sharp = ctl.apply_sharp_policy(40.0, 2.5, 1)
    assert sharp and abs(u_cmd - 60.0) < 1e-9 and base == 60.0
    u_cmd, base, sharp = ctl.apply_sharp_policy(40.0, 0.4, 3)
    assert sharp and abs(u_cmd - 60.0) < 1e-9 and base == 60.0
    u_cmd, base, sharp = ctl.apply_sharp_policy(40.0, 0.4, 2)
    assert (not sharp) and u_cmd == 40.0 and base == 100.0
    u_cmd, base, sharp = ctl.apply_sharp_policy(120.0, 3.0, 4)
    assert sharp and abs(u_cmd - 150.0) < 1e-9 and base == 60.0

    # 1.5 倍不能写回 u_k
    ctl.pid.u_k = 40.0
    cmd = ctl.step([0, 0, 0, 0, 3000, 3000, 3000, 0], error=0.2)
    assert cmd["sharp"] and abs(ctl.pid.u_k - cmd["u_pid"]) < 1e-9
    assert abs(cmd["u_cmd"] - constrain(cmd["u_pid"] * 1.5, -150, 150)) < 1e-9
    assert cmd["u_pid"] != cmd["u_cmd"] or abs(cmd["u_pid"]) * 1.5 > 150.0

    left, right = ctl.mix_pwm(30.2, 100.0)
    assert left == 130 and right == 69
    left, right = ctl.mix_pwm(150.0, 60.0)
    assert left == 210 and right == 0
