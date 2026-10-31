# -*- coding: utf-8 -*-
"""线性 MPC：误差状态 e=[e_y, e_theta]，单输入优化 delta_u = omega - omega_r。

第一版：
- 预测时域内 A、B 固定为当前步（v_r 当前值）
- 只优化 omega，线速度按路径段预调度 v_r
- 求解器优先原生 OSQP 稠密 QP；其次 cvxpy；再次 scipy SLSQP
"""

from __future__ import annotations

import time

import numpy as np

from config.params import CONFIG, MPC_CONFIG, pwm_to_speed_scale

_SOLVER_BACKEND = None


def _detect_solver():
    global _SOLVER_BACKEND
    if _SOLVER_BACKEND is not None:
        return _SOLVER_BACKEND
    try:
        import osqp  # noqa: F401
        from scipy import sparse  # noqa: F401

        _SOLVER_BACKEND = "osqp_native"
        return _SOLVER_BACKEND
    except ImportError:
        pass
    try:
        import cvxpy as cp  # noqa: F401

        try:
            import osqp  # noqa: F401

            _SOLVER_BACKEND = "cvxpy_osqp"
        except ImportError:
            _SOLVER_BACKEND = "cvxpy_default"
    except ImportError:
        _SOLVER_BACKEND = "scipy"
    return _SOLVER_BACKEND


def wrap_angle(angle):
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def constrain(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def reference_speed_for_segment(kind, mpc_cfg=None):
    """与 PID base_speed 切换一致的几何规则：急弯/直角弯用 v_low，其余 v_high。"""
    mpc_cfg = MPC_CONFIG if mpc_cfg is None else mpc_cfg
    if kind in ("sharp", "corner"):
        return float(mpc_cfg["v_low"])
    return float(mpc_cfg["v_high"])


def speed_omega_to_pwm(v, omega, cfg=None):
    """把 (v, ω) 映射为左右轮 PWM，施加死区与饱和。"""
    cfg = CONFIG if cfg is None else cfg
    L = float(cfg["wheel_base"])
    scale = pwm_to_speed_scale(cfg)
    dead = float(cfg["pwm_deadzone"])
    v_left = float(v) - 0.5 * float(omega) * L
    v_right = float(v) + 0.5 * float(omega) * L

    def one(speed):
        if speed <= 1e-9:
            return 0.0
        pwm = speed / scale
        if pwm < dead:
            pwm = dead
        return constrain(pwm, cfg["pwm_min"], cfg["pwm_max"])

    return int(one(v_left)), int(one(v_right))


class LinearMPC:
    """滚动时域线性 MPC。"""

    def __init__(self, cfg=None, mpc_cfg=None):
        self.cfg = CONFIG if cfg is None else cfg
        self.mpc_cfg = dict(MPC_CONFIG if mpc_cfg is None else mpc_cfg)
        self.N = int(self.mpc_cfg["N"])
        self.Q = np.diag(np.asarray(self.mpc_cfg["Q"], dtype=float))
        self.R = float(self.mpc_cfg["R"])
        self.S = float(self.mpc_cfg["S"])
        self.P = np.diag(np.asarray(self.mpc_cfg["P"], dtype=float))
        self.omega_min = float(self.mpc_cfg["omega_min"])
        self.omega_max = float(self.mpc_cfg["omega_max"])
        self.delta_omega_max = float(self.mpc_cfg["delta_omega_max"])
        self.e_y_max = float(self.mpc_cfg["e_y_max"])
        self.M = float(self.mpc_cfg["soft_constraint_M"])
        self.use_soft = bool(self.mpc_cfg.get("use_soft_ey", True))
        self.dt = float(self.cfg["dt"])
        self.backend = _detect_solver()
        self._cvx = None
        self._osqp = None
        self._cache_vr = None
        self.reset()
        if self.backend.startswith("cvxpy"):
            self._build_cvx_problem()

    def reset(self):
        self.omega_prev = 0.0
        self.delta_u_prev = 0.0
        self.last_solve_time_s = 0.0
        self.last_status = "reset"
        self.last_delta_u_seq = None
        self.last_e_pred = None

    def _ab(self, v_r):
        ts = self.dt
        a = np.array([[1.0, float(v_r) * ts], [0.0, 1.0]], dtype=float)
        b = np.array([[0.0], [ts]], dtype=float)
        return a, b

    def _prediction_maps(self, v_r):
        """e_{1:N} = Phi e0 + Gamma du。"""
        n = self.N
        a, b = self._ab(v_r)
        powers = [np.eye(2)]
        for _ in range(n):
            powers.append(powers[-1] @ a)
        phi = np.vstack([powers[k] for k in range(1, n + 1)])
        gamma = np.zeros((2 * n, n))
        for k in range(1, n + 1):
            for j in range(k):
                gamma[2 * (k - 1) : 2 * k, j] = (powers[k - 1 - j] @ b).ravel()
        qblk = np.zeros((2 * n, 2 * n))
        for k in range(n - 1):
            qblk[2 * k : 2 * k + 2, 2 * k : 2 * k + 2] = self.Q
        qblk[2 * (n - 1) : 2 * n, 2 * (n - 1) : 2 * n] = self.P
        return a, b, phi, gamma, qblk

    def _hu(self):
        n = self.N
        hu = self.R * np.eye(n) + self.S * np.eye(n)
        for k in range(1, n):
            hu[k, k] += self.S
            hu[k - 1, k - 1] += self.S
            hu[k, k - 1] -= self.S
            hu[k - 1, k] -= self.S
        return hu

    def _solve_osqp_native(self, e0, v_r, omega_r):
        import osqp
        from scipy import sparse

        n = self.N
        a, b, phi, gamma, qblk = self._prediction_maps(v_r)
        hu = self._hu()
        # 代价 du'(Gamma'Q Gamma + Hu) du + 2 (Phi e0)'Q Gamma du - 2 S du_prev du_0
        # OSQP: 0.5 x'P x + q'x  ⇒  P=2H, q=线性项
        h = gamma.T @ qblk @ gamma + hu
        h = 0.5 * (h + h.T)
        p = 2.0 * h
        q = (2.0 * (gamma.T @ qblk @ (phi @ e0))).ravel()
        q[0] -= 2.0 * self.S * self.delta_u_prev

        # 约束: omega bounds, rate on du0, rate on consecutive du
        # A_c du <= u_c  and  A_c du >= l_c
        # 1) omega_min - omega_r <= du_k <= omega_max - omega_r
        # 2) -dmax <= du_0 - (omega_prev - omega_r) <= dmax
        # 3) -dmax <= du_k - du_{k-1} <= dmax
        rows = []
        lo = []
        hi = []
        for k in range(n):
            row = np.zeros(n)
            row[k] = 1.0
            rows.append(row)
            lo.append(self.omega_min - omega_r)
            hi.append(self.omega_max - omega_r)
        row = np.zeros(n)
        row[0] = 1.0
        rows.append(row)
        # du_0 + omega_r - omega_prev ∈ [-dmax, dmax]
        lo.append(-self.delta_omega_max - omega_r + self.omega_prev)
        hi.append(self.delta_omega_max - omega_r + self.omega_prev)
        for k in range(1, n):
            row = np.zeros(n)
            row[k] = 1.0
            row[k - 1] = -1.0
            rows.append(row)
            lo.append(-self.delta_omega_max)
            hi.append(self.delta_omega_max)

        a_csc = sparse.csc_matrix(np.vstack(rows))
        p_csc = sparse.csc_matrix(p)
        l_arr = np.asarray(lo, dtype=float)
        u_arr = np.asarray(hi, dtype=float)

        t0 = time.perf_counter()
        if (
            self._osqp is None
            or self._cache_vr is None
            or abs(self._cache_vr - float(v_r)) > 1e-9
        ):
            prob = osqp.OSQP()
            # 新版 OSQP Python 绑定用 caps 参数名
            try:
                prob.setup(
                    P=p_csc,
                    q=q,
                    A=a_csc,
                    l=l_arr,
                    u=u_arr,
                    warm_starting=True,
                    verbose=False,
                    eps_abs=1e-4,
                    eps_rel=1e-4,
                    max_iter=4000,
                )
            except TypeError:
                prob.setup(
                    P=p_csc,
                    q=q,
                    A=a_csc,
                    l=l_arr,
                    u=u_arr,
                    warm_start=True,
                    verbose=False,
                    eps_abs=1e-4,
                    eps_rel=1e-4,
                    max_iter=4000,
                )
            self._osqp = prob
            self._cache_vr = float(v_r)
            self._osqp_a = a
            self._osqp_b = b
        else:
            self._osqp.update(q=q, l=l_arr, u=u_arr)
            # P 随 v_r 变；同 v_r 时 P 不变
        res = self._osqp.solve()
        self.last_solve_time_s = time.perf_counter() - t0
        if res.x is None or res.info.status_val not in (1, 2):
            self.last_status = str(getattr(res.info, "status", "osqp_fail"))
            return None, None
        du_seq = np.asarray(res.x, dtype=float).ravel()
        e = e0.copy()
        traj = [e.copy()]
        bvec = b.ravel()
        for k in range(n):
            e = a @ e + bvec * du_seq[k]
            traj.append(e.copy())
        self.last_status = str(res.info.status)
        return du_seq, np.vstack(traj)

    def _build_cvx_problem(self):
        import cvxpy as cp

        n = self.N
        e0 = cp.Parameter(2, name="e0")
        v_r = cp.Parameter(nonneg=True, name="v_r")
        omega_r = cp.Parameter(name="omega_r")
        omega_prev = cp.Parameter(name="omega_prev")
        du_prev = cp.Parameter(name="du_prev")
        du = cp.Variable(n, name="du")
        e_vars = [cp.Variable(2) for _ in range(n + 1)]
        constraints = [e_vars[0] == e0]
        ts = self.dt
        cost = cp.quad_form(e_vars[0], self.Q)
        for k in range(n):
            constraints.append(
                e_vars[k + 1][0] == e_vars[k][0] + v_r * ts * e_vars[k][1]
            )
            constraints.append(e_vars[k + 1][1] == e_vars[k][1] + ts * du[k])
            omega_k = omega_r + du[k]
            constraints += [omega_k >= self.omega_min, omega_k <= self.omega_max]
            omega_im1 = omega_prev if k == 0 else (omega_r + du[k - 1])
            constraints.append(cp.abs(omega_k - omega_im1) <= self.delta_omega_max)
            du_im1 = du_prev if k == 0 else du[k - 1]
            cost += self.R * cp.square(du[k]) + self.S * cp.square(du[k] - du_im1)
            if k < n - 1:
                cost += cp.quad_form(e_vars[k + 1], self.Q)
            if self.use_soft:
                eps = cp.Variable(nonneg=True)
                constraints += [
                    e_vars[k + 1][0] <= self.e_y_max + eps,
                    -e_vars[k + 1][0] <= self.e_y_max + eps,
                ]
                cost += self.M * eps
        cost += cp.quad_form(e_vars[n], self.P)
        self._cvx = {
            "cp": cp,
            "prob": cp.Problem(cp.Minimize(cost), constraints),
            "du": du,
            "e_vars": e_vars,
            "e0": e0,
            "v_r": v_r,
            "omega_r": omega_r,
            "omega_prev": omega_prev,
            "du_prev": du_prev,
        }

    def _solve_cvxpy(self, e0, v_r, omega_r):
        cvx = self._cvx
        cp = cvx["cp"]
        cvx["e0"].value = np.asarray(e0, dtype=float)
        cvx["v_r"].value = float(max(v_r, 1e-6))
        cvx["omega_r"].value = float(omega_r)
        cvx["omega_prev"].value = float(self.omega_prev)
        cvx["du_prev"].value = float(self.delta_u_prev)
        t0 = time.perf_counter()
        solved = False
        status = None
        if self.backend == "cvxpy_osqp":
            try:
                cvx["prob"].solve(
                    solver=cp.OSQP,
                    warm_start=True,
                    eps_abs=1e-4,
                    eps_rel=1e-4,
                    verbose=False,
                )
                solved = cvx["prob"].status in ("optimal", "optimal_inaccurate")
                status = cvx["prob"].status
            except Exception as exc:  # noqa: BLE001
                status = f"osqp_fail:{exc}"
        if not solved:
            try:
                cvx["prob"].solve(verbose=False)
                solved = cvx["prob"].status in ("optimal", "optimal_inaccurate")
                status = cvx["prob"].status
            except Exception as exc:  # noqa: BLE001
                status = f"cvxpy_fail:{exc}"
        self.last_solve_time_s = time.perf_counter() - t0
        if not solved or cvx["du"].value is None:
            self.last_status = status or "fail"
            return None, None
        du_seq = np.asarray(cvx["du"].value, dtype=float).ravel()
        e_pred = np.vstack(
            [np.asarray(ev.value, dtype=float).ravel() for ev in cvx["e_vars"]]
        )
        self.last_status = status
        return du_seq, e_pred

    def _solve_scipy(self, e0, v_r, omega_r):
        from scipy.optimize import minimize

        n = self.N
        a, b = self._ab(v_r)
        bvec = b.ravel()

        def rollout(du):
            e = e0.copy()
            traj = [e.copy()]
            for k in range(n):
                e = a @ e + bvec * du[k]
                traj.append(e.copy())
            return traj

        def objective(du):
            du = np.asarray(du, dtype=float)
            traj = rollout(du)
            cost = float(traj[0].T @ self.Q @ traj[0])
            prev = self.delta_u_prev
            for k in range(n):
                cost += self.R * du[k] ** 2 + self.S * (du[k] - prev) ** 2
                prev = du[k]
                if k < n - 1:
                    cost += float(traj[k + 1].T @ self.Q @ traj[k + 1])
            cost += float(traj[n].T @ self.P @ traj[n])
            return cost

        cons = []
        for k in range(n):
            cons.append({"type": "ineq", "fun": (lambda du, k=k: omega_r + du[k] - self.omega_min)})
            cons.append({"type": "ineq", "fun": (lambda du, k=k: self.omega_max - (omega_r + du[k]))})
        cons.append(
            {
                "type": "ineq",
                "fun": lambda du: self.delta_omega_max - abs(omega_r + du[0] - self.omega_prev),
            }
        )
        for k in range(1, n):
            cons.append(
                {
                    "type": "ineq",
                    "fun": (lambda du, k=k: self.delta_omega_max - abs(du[k] - du[k - 1])),
                }
            )
        t0 = time.perf_counter()
        res = minimize(
            objective,
            np.zeros(n),
            method="SLSQP",
            constraints=cons,
            options={"maxiter": 200, "ftol": 1e-6, "disp": False},
        )
        self.last_solve_time_s = time.perf_counter() - t0
        self.last_status = "scipy_ok" if res.success else f"scipy:{res.message}"
        du_seq = np.asarray(res.x, dtype=float).ravel()
        return du_seq, np.vstack(rollout(du_seq))

    def solve(self, e_y, e_theta, v_r, omega_r):
        e0 = np.array([float(e_y), float(e_theta)], dtype=float)
        if self.backend == "osqp_native":
            du_seq, e_pred = self._solve_osqp_native(e0, float(v_r), float(omega_r))
        elif self.backend.startswith("cvxpy"):
            du_seq, e_pred = self._solve_cvxpy(e0, float(v_r), float(omega_r))
        else:
            du_seq, e_pred = self._solve_scipy(e0, float(v_r), float(omega_r))

        if du_seq is None:
            du0 = float(
                np.clip(-2.0 * e_y / max(v_r, 1e-3), -self.delta_omega_max, self.delta_omega_max)
            )
            omega_cmd = float(np.clip(omega_r + du0, self.omega_min, self.omega_max))
            du0 = omega_cmd - omega_r
            self.last_delta_u_seq = np.full(self.N, du0)
            self.last_e_pred = None
        else:
            du0 = float(du_seq[0])
            omega_cmd = float(np.clip(omega_r + du0, self.omega_min, self.omega_max))
            du0 = omega_cmd - float(omega_r)
            self.last_delta_u_seq = du_seq
            self.last_e_pred = e_pred

        self.delta_u_prev = du0
        self.omega_prev = omega_cmd
        return {
            "delta_u": du0,
            "omega_cmd": float(self.omega_prev),
            "omega_r": float(omega_r),
            "v_r": float(v_r),
            "e_y": float(e_y),
            "e_theta": float(e_theta),
            "solve_time_s": float(self.last_solve_time_s),
            "status": self.last_status,
            "backend": self.backend,
        }

    def step(self, e_y, e_theta, v_r, omega_r):
        out = self.solve(e_y, e_theta, v_r, omega_r)
        left_pwm, right_pwm = speed_omega_to_pwm(out["v_r"], out["omega_cmd"], self.cfg)
        out["left_pwm"] = left_pwm
        out["right_pwm"] = right_pwm
        out["base_speed"] = out["v_r"] / pwm_to_speed_scale(self.cfg)
        return out


def self_check_mpc_direction():
    mpc = LinearMPC()
    mpc.reset()
    _ = mpc.solve(e_y=0.02, e_theta=0.0, v_r=0.30, omega_r=0.0)
    mpc.reset()
    out = mpc.solve(e_y=0.02, e_theta=0.0, v_r=0.30, omega_r=0.0)
    assert out["delta_u"] < 0.0, f"期望右转 delta_u<0，得到 {out['delta_u']}"
    assert mpc.last_e_pred is not None
    print(
        f"[MPC 单测] backend={out['backend']}  delta_u0={out['delta_u']:.4f}  "
        f"omega_cmd={out['omega_cmd']:.4f}  solve={out['solve_time_s']*1000:.2f} ms"
    )
    print("  预测 e_y:", np.array2string(mpc.last_e_pred[:, 0], precision=4))
    print("  预测 e_th:", np.array2string(mpc.last_e_pred[:, 1], precision=4))
    return out
