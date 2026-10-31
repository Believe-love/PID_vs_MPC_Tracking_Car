# PID 与 MPC 小车轨迹跟踪对比

同一辆差速循迹小车、同一条黑线、同一套传感器噪声，分别用增量式 PID 和线性模型预测控制（MPC）闭环。PID 复现实物固件 `PIDtracking.ino` 的可执行控制律；MPC 在同一被控对象上只替换控制器。本文件把两条控制律用到的每一个公式从定义推到代码里的那一行。

随机种子固定为 42。在项目根目录执行 `py -3 run_all.py` 会依次完成 PID 仿真、MPC 仿真、对比图和带数据标注的图。

## 目录

```text
PID_vs_MPC_Tracking_Car/
├── README.md
├── LICENSE
├── requirements.txt
├── .gitignore
├── run_all.py                      # PID → MPC → 对比图 → 标注图
├── config/
│   └── params.py                   # 车辆、传感器、路径仿真参数，以及 MPC_CONFIG
├── core/
│   ├── car_model.py                # 运动学、轮速滞后、传感器、加权误差
│   └── path.py                     # 直线/圆弧参考路径与投影
├── controllers/
│   ├── pid.py                      # 增量式 PID 与急弯分支
│   └── mpc.py                      # 线性 MPC（OSQP / cvxpy / SLSQP）
├── simulators/
│   ├── pid_sim.py                  # PID 闭环入口
│   ├── mpc_sim.py                  # MPC 闭环入口
│   ├── visualize_pid.py            # PID 曲线、轨迹、GIF
│   └── visualize_mpc.py            # MPC 曲线、轨迹、GIF
├── comparison/
│   ├── compare.py                  # 四张对比图和指标
│   └── generate_plots.py           # 三张带大字标注的图
└── results/
    ├── data/
    └── figures/
```

`visualize_pid.py` 与 `visualize_mpc.py` 只负责出图，不参与控制律。

## 安装与运行

需要 Python 3.9 及以上。

```text
cd PID_vs_MPC_Tracking_Car
py -3 -m pip install -r requirements.txt
py -3 run_all.py
```

`python` 可用时把 `py -3` 换成 `python`。依赖是 `numpy`、`matplotlib`、`Pillow`、`pandas`、`cvxpy`、`osqp`、`scipy`。

也可以分步运行：

```text
py -3 simulators/pid_sim.py
py -3 simulators/mpc_sim.py
py -3 comparison/compare.py
py -3 comparison/generate_plots.py
```

后两步读取 `results/data/pid_timeseries.csv` 和 `results/data/mpc_timeseries.csv`。对比脚本把派生表写成 `pid_frame.csv`、`mpc_frame.csv`，不覆盖这两份原始时序，因此可以单独重画对比图。

图题优先使用微软雅黑。系统没有中文字体时自动改成英文标签。

### 输出

| 路径 | 内容 |
| --- | --- |
| `results/data/pid_timeseries.csv` | PID 原始时序 |
| `results/data/pid_metrics.md`、`pid_metrics.csv` | PID 分段误差、丢线 |
| `results/figures/error_vs_time.png` | PID 传感器误差与到线距离 |
| `results/figures/trajectory_compare.png` | PID 轨迹 |
| `results/figures/control_signals.png` | PID 的 PWM、`u_k`、`base_speed` |
| `results/figures/simulation.gif` | PID 动画 |
| `results/data/mpc_timeseries.csv` | MPC 原始时序 |
| `results/data/mpc_metrics.md`、`mpc_metrics.csv` | MPC 指标与 QP 耗时 |
| `results/figures/mpc_error_vs_time.png` | MPC 的 \(e_y\) |
| `results/figures/mpc_trajectory.png` | MPC 轨迹 |
| `results/figures/mpc_control_signals.png` | \(\omega_{\mathrm{cmd}}\)、\(\omega_r\)、\(\Delta u\)、曲率 |
| `results/figures/mpc_simulation.gif` | MPC 动画 |
| `results/figures/compare_trajectory.png` | 两车轨迹，含弯道放大 |
| `results/figures/compare_error.png` | 几何横向误差 |
| `results/figures/compare_omega.png` | 角速度指令 |
| `results/figures/compare_preview.png` | MPC 指令相对 \(\omega_r\) 阶跃 |
| `results/data/comparison_metrics.md` | 对比指标与文字结论 |
| `results/figures/compare_error_annotated.png` | 急弯峰值标注 |
| `results/figures/compare_preview_annotated.png` | 急弯入口打角时刻 |
| `results/figures/compare_straight_annotated.png` | 末段直道回正 |
| `results/data/metrics_updated.md` | 标注图用到的三个数字 |

## 公平对比约定

两边共用 `config/params.py` 里的 `CONFIG`：

| 量 | 值 |
| --- | --- |
| 控制周期 \(T_s\) | 0.02 s（50 Hz） |
| 随机种子 | 42 |
| 轮距 \(L\) | 0.16 m |
| PWM = 100 的稳态轮速 | 0.30 m/s，系数 0.003 m/s 每 PWM |
| PWM 死区 / 限幅 | 30 / \([0, 255]\) |
| 轮速滞后 \(\tau\) | 0.18 s |
| 传感器坐标 `SENSOR_X` | \(-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5\) |
| 传感器跨度 / 前视 | 0.12 m / 0.075 m |
| 模拟量 | 峰值 8000，\(\sigma=0.0082\,\mathrm{m}\)，噪声标准差 120，截断到 \([0,8000]\) |
| 初始横向偏移 | 0.008 m（车在路径左侧），初始航向误差 0 |

每个控制周期的顺序相同：用当前位姿生成 8 路模拟量，计算加权误差，调用控制器，把 PWM 经死区和一阶滞后变成轮速，再用 RK4 推进独轮车一个周期。随机数只进入模拟量；MPC 在 8 路模拟量抽完之后，再额外抽一次航向噪声。因此种子相同时，两车的模拟量噪声序列对齐。

PID 的增益没有为了仿真改过：\(K_p=33\)，\(K_i=0\)，\(K_d=10\)，控制量限幅 \([-150,150]\)。

## 符号

世界坐标是俯视图。\(x\) 向右，\(y\) 向上，航向 \(\theta\) 从 \(x\) 轴逆时针量到车头。车体坐标以前向为 \(x_b\)、左侧为 \(y_b\)。

| 符号 | 含义 | 代码 |
| --- | --- | --- |
| \(q=(x,y,\theta)\) | 驱动轴中心位姿 | `x, y, theta` |
| \(s\) | 路径弧长 | `s` |
| \(\kappa\) | 曲率，左转为正 | `kappa` / `kappa_r` |
| \(v_L, v_R\) | 左、右轮前进速度 | `v_left`, `v_right` |
| \(v,\omega\) | 轴心线速度、角速度（左转为正） | `v`, `omega` |
| \(e_{\mathrm{sensor}}\) | 8 路加权误差，传感器坐标 | PID 的 `error` |
| \(e_y^{\mathrm{sensor}}\) | 上式乘以米制比例 | MPC 优化用的 `e_y` |
| \(e_y\) | 轴心相对路径的几何横向误差，左侧为正 | 时序里的 `lat`；对比图的 `e_y` |
| \(e_\theta\) | 航向误差 \(\theta-\theta_r\)，再包到 \((-\pi,\pi]\) | MPC 的 `e_theta` |
| \(u_k\) | 增量式 PID 的累加输出 | `u_pid` |
| \(\Delta u\) | MPC 决策变量 \(\omega-\omega_r\) | `delta_u` |

对比图里的 \(e_y\) 是几何量 `lat`。MPC 优化器里的 \(e_y\) 是传感器加权误差换成米。丢线时后者被置 0，前者仍是轴心到中心线的真实距离。比“谁偏离黑线”必须用几何量。

## 1. 参考路径

实物固件没有地图，只消费 8 路模拟量。仿真构造一条开放中心线，供传感器测距，也供 MPC 读取曲率和参考航向。曲率 \(\kappa>0\) 表示向左（逆时针）转弯。

路径由直线段和圆弧段串接。第 \(i\) 段在自身起点 \((x_0,y_0,h_0)\) 处的局部弧长记为 \(s_{\mathrm{loc}}\)。

### 1.1 直线

曲率为 0 时，位姿沿切线平移：

\[
\begin{aligned}
x &= x_0 + s_{\mathrm{loc}}\cos h_0,\\
y &= y_0 + s_{\mathrm{loc}}\sin h_0,\\
h &= h_0.
\end{aligned}
\]

### 1.2 圆弧

常曲率 \(\kappa\neq 0\) 的曲线是圆，半径 \(R=1/|\kappa|\)。圆心在起点的左法向一侧，距离为 \(1/\kappa\)（\(\kappa<0\) 时这个“距离”为负，圆心落到右侧）。左法向是 \((-\sin h_0,\cos h_0)\)，所以

\[
\begin{aligned}
c_x &= x_0 - \frac{\sin h_0}{\kappa},\\
c_y &= y_0 + \frac{\cos h_0}{\kappa}.
\end{aligned}
\]

把从圆心指向起点的向量 \(\mathbf{v}_0=(x_0-c_x,\, y_0-c_y)\) 逆时针旋转 \(\phi=\kappa s_{\mathrm{loc}}\)：

\[
\begin{bmatrix} x-c_x \\ y-c_y \end{bmatrix}
=
\begin{bmatrix} \cos\phi & -\sin\phi \\ \sin\phi & \cos\phi \end{bmatrix}
\mathbf{v}_0,
\qquad
h = h_0+\phi.
\]

\(\kappa>0\) 时 \(\phi\) 随弧长增加，航向增加，车向左转。弧长走完 \(R\cdot\alpha\) 时，转过的角度就是 \(\alpha\)。这两段解析式就是 `pose_on_segment`。

### 1.3 本仓库的赛道

| 名称 | 类型 | 半径 | 转角 | 弧长 | 曲率 |
| --- | --- | ---: | ---: | ---: | ---: |
| 直道1 | straight | \(\infty\) | 0 | 1.20 m | 0 |
| 缓弯 | gentle | 0.80 m | 左转 \(90^\circ\) | \(0.8\cdot\pi/2\) | \(1.25\,\mathrm{m}^{-1}\) |
| 直道2 | straight | \(\infty\) | 0 | 0.70 m | 0 |
| S弯1 | s | 0.60 m | 右转 \(45^\circ\) | \(0.6\cdot\pi/4\) | \(-1.667\,\mathrm{m}^{-1}\) |
| S弯2 | s | 0.60 m | 左转 \(90^\circ\) | \(0.6\cdot\pi/2\) | \(+1.667\,\mathrm{m}^{-1}\) |
| S弯3 | s | 0.60 m | 右转 \(45^\circ\) | \(0.6\cdot\pi/4\) | \(-1.667\,\mathrm{m}^{-1}\) |
| 直道3 | straight | \(\infty\) | 0 | 0.55 m | 0 |
| 急弯 | sharp | 0.25 m | 左转 \(90^\circ\) | \(0.25\cdot\pi/2\) | \(4\,\mathrm{m}^{-1}\) |
| 直道4 | straight | \(\infty\) | 0 | 0.50 m | 0 |
| 直角弯 | corner | 0.025 m | 左转 \(90^\circ\) | \(0.025\cdot\pi/2\) | \(40\,\mathrm{m}^{-1}\) |
| 直道5 | straight | \(\infty\) | 0 | 2.90 m | 0 |

总长

\[
\begin{aligned}
&1.20+0.70+0.55+0.50+2.90\\
&+0.80\cdot\frac{\pi}{2}
+0.60\pi
+0.25\cdot\frac{\pi}{2}
+0.025\cdot\frac{\pi}{2}
= 9.424\,\mathrm{m}.
\end{aligned}
\]

S 弯三段的转角 \(-45^\circ+90^\circ-45^\circ=0\)，进出 S 弯的航向相同。全程转角净增 \(270^\circ\)。

`path.py` 文件头有一句旧注释写着直角弯半径 0.05 m、曲率 \(20\,\mathrm{m}^{-1}\)。可执行的段表是上表：半径 0.025 m，曲率 \(40\,\mathrm{m}^{-1}\)。仿真以段表为准。

直角弯放在急弯之后，中间有 0.50 m 直道，这样两处偏差不会叠在同一段统计里。直道5 长 2.90 m，用来看丢线之后还有没有机会回到线上。

### 1.4 把点投影到中心线

路径先按 \(\Delta s=2\,\mathrm{mm}\) 采样成折线。给定点 \((x,y)\) 和上一步弧长提示 \(s_{\mathrm{hint}}\)，只在 \([s_{\mathrm{hint}}-0.30,\, s_{\mathrm{hint}}+0.80]\,\mathrm{m}\) 里找最近采样点，再向相邻的两条线段做垂足。窗口是为了避免 S 弯折返后吸附到另一支。

线段 \(A\to B\) 上的垂足参数

\[
t=\mathrm{clip}\!\left(
\frac{(x-A_x)(B_x-A_x)+(y-A_y)(B_y-A_y)}{\|B-A\|^2},\,
0,\,1
\right),
\]

垂足 \(P=A+t(B-A)\)，弧长与航向按 \(t\) 线性插值。点相对垂足的位移 \((d_x,d_y)=(x-P_x,\, y-P_y)\) 在左法向上的投影是几何横向误差：

\[
e_y = d_{\mathrm{left}} = -\sin h\, d_x + \cos h\, d_y.
\]

\(h=0\) 时 \(e_y=d_y\)：点在路径上方（左侧）则为正。欧氏距离 \(\sqrt{d_x^2+d_y^2}\) 只给传感器模型用，不进入 PID。

## 2. 车辆运动学

### 2.1 差速到独轮车

左右轮没有侧滑，轮距为 \(L\)。轴心在两轮中点。一个短时间 \(\mathrm{d}t\) 内左轮走过 \(v_L\mathrm{d}t\)，右轮走过 \(v_R\mathrm{d}t\)。轴心位移是二者平均，左右位移差造成转角：

\[
v=\frac{v_R+v_L}{2},
\qquad
\omega=\frac{v_R-v_L}{L}.
\]

\(\omega>0\) 为逆时针，即左转。若 \(v_L>v_R\)，则 \(\omega<0\)，车向右转。

瞬时转弯半径 \(R_{\mathrm{turn}}=v/\omega\)（\(\omega\neq 0\)）。代入上式：

\[
R_{\mathrm{turn}}
=\frac{L}{2}\cdot\frac{v_R+v_L}{v_R-v_L}.
\]

本车 PWM 只有 \([0,255]\)，轮速不能为负。在 \(v_L\ge 0\)、\(v_R\ge 0\) 里，

\[
\left|\frac{v_R+v_L}{v_R-v_L}\right|\ge 1,
\]

等号在一只轮子速度为 0、另一只轮子速度为正时取到。因此

\[
\boxed{|R_{\mathrm{turn}}|\ge \frac{L}{2}=0.08\,\mathrm{m}}.
\]

半径 0.025 m 的直角弯小于 8 cm。只要两轮都不能倒转，任何前进速度都跟不住这条圆弧的曲率。这是机构约束，不是 PID 或 MPC 调参能去掉的。

### 2.2 平面运动的微分方程

轴心速度沿车头方向，角速度直接改变航向：

\[
\dot x = v\cos\theta,
\qquad
\dot y = v\sin\theta,
\qquad
\dot\theta = \omega.
\]

一个控制周期内把刚算出的 \(v,\omega\) 当成常数。\(\omega\neq 0\) 时这段 ODE 有解析解

\[
\begin{aligned}
\theta(T_s)&=\theta_0+\omega T_s,\\
x(T_s)&=x_0+\frac{v}{\omega}\big(\sin(\theta_0+\omega T_s)-\sin\theta_0\big),\\
y(T_s)&=y_0-\frac{v}{\omega}\big(\cos(\theta_0+\omega T_s)-\cos\theta_0\big).
\end{aligned}
\]

代码没有调用这个解析式，而是对同一 ODE 做一步 RK4，步长等于控制周期。记 \(f(\theta)=(v\cos\theta,\, v\sin\theta,\, \omega)\)，

\[
\begin{aligned}
k_1&=f(\theta),\\
k_2&=f(\theta+\tfrac12 T_s k_{1,\theta}),\\
k_3&=f(\theta+\tfrac12 T_s k_{2,\theta}),\\
k_4&=f(\theta+T_s k_{3,\theta}),\\
q^+&=q+\frac{T_s}{6}(k_1+2k_2+2k_3+k_4).
\end{aligned}
\]

\(v,\omega\) 在步内为常数时，\(k\) 的第三分量始终是 \(\omega\)，航向更新就是 \(\theta+T_s\omega\)，与解析解一致。\(x,y\) 的 RK4 是对 \(\cos\theta(t)\)、\(\sin\theta(t)\) 的四阶近似。\(T_s=0.02\,\mathrm{s}\) 时，即便 \(\omega\) 到约 \(5\,\mathrm{rad/s}\)，一步转角也只有 \(0.1\,\mathrm{rad}\)，局部截断误差远小于传感器噪声。

模型里没有侧滑，也没有绕竖直轴的转动惯量。车身跟不上 PWM 的效应全部放进下一节的轮速滞后。

### 2.3 PWM、死区与一阶滞后

稳态映射是一条过原点的直线，再加硬死区。记 \(k_v=0.30/100=0.003\,\mathrm{m/s}\) 每 PWM，

\[
v_{\mathrm{wheel}}(\mathrm{PWM})=
\begin{cases}
0, & \mathrm{PWM}<30,\\
k_v\cdot\mathrm{PWM}, & \mathrm{PWM}\ge 30.
\end{cases}
\]

PWM = 100 时轮速 0.30 m/s。死区边界上速度从 0 跳到 \(0.003\times 30=0.09\,\mathrm{m/s}\)。PID 送给电机的 PWM 先被截成整数（向 0 取整），再进入这个映射。

电机和车身对 PWM 的跟踪用一阶滞后描述。连续时间

\[
\tau \dot v_w = v_{\mathrm{cmd}}-v_w,
\qquad \tau=0.18\,\mathrm{s}.
\]

分离变量。\(v_{\mathrm{cmd}}\) 在一个控制周期内为常数时，

\[
\int_{v_w(0)}^{v_w(T_s)}\frac{\mathrm{d}v}{v_{\mathrm{cmd}}-v}
=\int_0^{T_s}\frac{\mathrm{d}t}{\tau},
\]

\[
-\ln\frac{v_{\mathrm{cmd}}-v_w(T_s)}{v_{\mathrm{cmd}}-v_w(0)}=\frac{T_s}{\tau},
\]

\[
\boxed{v_w(T_s)=v_{\mathrm{cmd}}+\big(v_w(0)-v_{\mathrm{cmd}}\big)e^{-T_s/\tau}}.
\]

这就是 `lag_wheel_speed`。\(\tau\le 0\) 时直接取 \(v_{\mathrm{cmd}}\)。稳态增益不变，仍然是上面的 PWM 直线。\(T_s/\tau=0.02/0.18\approx 0.111\)，每步只消除约 \(10.5\%\) 的轮速误差，所以直角弯上来不及在几厘米的弧长里把航向转过 \(90^\circ\)。

仿真开始时两轮已经处在基础 PWM = 100 的稳态巡航速度上，滞后只作用在随后的差速修正上。

## 3. 传感器

### 3.1 几何

8 路传感器装在轴心前方 \(\ell=0.075\,\mathrm{m}\) 的横杆上。`SENSOR_X` 从 \(-3.5\) 到 \(3.5\)，跨度 7 个相对单位，对应物理跨度 \(0.12\,\mathrm{m}\)。比例

\[
\sigma_x=\frac{0.12}{7}\approx 0.017143\,\mathrm{m/单位}.
\]

正的 `SENSOR_X` 放在车体右侧，而车体 \(y_b\) 向左为正，所以第 \(i\) 路的车体横向坐标是

\[
y_{b,i}=-\sigma_x\, \mathrm{SENSOR\_X}_i.
\]

世界坐标：

\[
\mathbf{p}_i
=
\begin{bmatrix}x\\y\end{bmatrix}
+\ell
\begin{bmatrix}\cos\theta\\ \sin\theta\end{bmatrix}
+y_{b,i}
\begin{bmatrix}-\sin\theta\\ \cos\theta\end{bmatrix}.
\]

每一路到黑线中心的距离，是 \(\mathbf{p}_i\) 投影到路径后的欧氏距离。

### 3.2 模拟量

黑线在传感器下是一个窄的空间脉冲。这里用高斯近似线宽和光斑，不是厂家标定曲线：

\[
a_i^{\mathrm{raw}}
=8000\exp\!\left(-\frac{d_i^2}{2\sigma_a^2}\right),
\qquad
\sigma_a=0.0082\,\mathrm{m}.
\]

再加上 \(N(0,120^2)\) 的高斯噪声，截断到 \([0,8000]\)。\(d_i=0\) 时期望值约 8000；\(d_i=\sigma_a\) 时降到 \(8000e^{-1/2}\approx 4852\)；\(d_i=2\sigma_a\approx 1.6\,\mathrm{cm}\) 时降到约 1083，刚超过急弯判据用的 1000；再远就掉进阈值 250 以下。直道上通常只有中间两路超过 1000，普通路段不会一直满足“3 路大于 1000”。

### 3.3 加权横向误差

这是固件 `calculatePositionError` 的公式。第 \(i\) 路权重

\[
w_i=
\begin{cases}
0, & a_i<250,\\
a_i, & a_i\ge 250.
\end{cases}
\]

源码注释写过“小于 250 则权重为 10”，可执行代码里的 `replace_weight` 是 0。仿真跟可执行代码。

\[
e_{\mathrm{sensor}}
=
\begin{cases}
0, & \sum_i w_i=0,\\[4pt]
\dfrac{\sum_i \mathrm{SENSOR\_X}_i\, w_i}{\sum_i w_i}, & \text{否则}.
\end{cases}
\]

这是传感器坐标的一阶矩，也就是响应质量的重心。只有第 5 路（`SENSOR_X` \(=0.5\)）为 3000、其余为 0 时，\(e_{\mathrm{sensor}}=0.5\)。左右对称且权重相等时，重心在 0。全部低于 250 时分子分母都没有信息，固件返回 0。丢线之后控制器看到的误差是 0，不是“线在很远的地方”。

\(e_{\mathrm{sensor}}>0\) 表示重心在车体右侧，也就是线在右侧、车偏左，需要右转。MPC 把它换成米：

\[
e_y^{\mathrm{sensor}}=\sigma_x\, e_{\mathrm{sensor}}.
\]

传感器阵列半宽是 \(0.06\,\mathrm{m}\)。线离开横杆之后，加权误差饱和在最外侧那一路 \(\pm 3.5\)，约 \(\pm 0.060\,\mathrm{m}\)，再远就变成 0。所以它不能当作“偏离了多少厘米”的测量。

### 3.4 前视

传感器在轴心前方 \(\ell=0.075\,\mathrm{m}\)。车速 \(v\) 时，轴心还要过

\[
t_{\mathrm{look}}\approx\frac{\ell}{v}
\]

才到达传感器现在看到的弧长。\(v=0.30\,\mathrm{m/s}\) 时约 0.25 s，也就是 12 到 13 个控制周期。这个前视对 PID 和 MPC 是同一套几何，不是 MPC 单独加的预瞄模块。

## 4. 增量式 PID

### 4.1 从位置式到增量式

位置式 PID 写成

\[
u_k^{\mathrm{pos}}
=K_p e_k
+K_i\sum_{j=0}^{k} e_j
+K_d(e_k-e_{k-1}).
\]

上一拍

\[
u_{k-1}^{\mathrm{pos}}
=K_p e_{k-1}
+K_i\sum_{j=0}^{k-1} e_j
+K_d(e_{k-1}-e_{k-2}).
\]

两式相减，和式只剩下最新一项 \(e_k\)，微分项变成二阶差分：

\[
\boxed{
\Delta u_k
=K_p(e_k-e_{k-1})
+K_i e_k
+K_d(e_k-2e_{k-1}+e_{k-2})
}.
\]

\[
u_k=\mathrm{clip}(u_{k-1}+\Delta u_k,\,-150,\,150).
\]

固件注释写明 \(K_i\)、\(K_d\) 已经含采样周期，公式里不再乘或除 \(T_s\)。本仓库保持 \(K_i=0\)，没有另加积分。先用旧的 \(e_{k-1},e_{k-2}\) 计算增量，再把历史滚成 \(e_{k-2}\leftarrow e_{k-1}\)、\(e_{k-1}\leftarrow e_k\)。初值全是 0。

饱和发生在累加之后。因为 \(K_i=0\)，没有积分器可以风起；但 \(u_k\) 被削顶以后，下一拍是从削过的值上继续加增量。这和“先算不饱和的位置式、再整体限幅”不是同一个动态。

### 4.2 三个节拍的手算

\(K_p=33\)，\(K_i=0\)，\(K_d=10\)。误差从 0 跳到 1 并保持。

第 1 拍：\(e_{k-1}=e_{k-2}=u=0\)，

\[
\Delta u=33(1-0)+10(1-0+0)=43,
\qquad u=43.
\]

第 2 拍：\(e_{k-1}=1\)，\(e_{k-2}=0\)，

\[
\Delta u=33(1-1)+10(1-2\cdot 1+0)=-10,
\qquad u=33.
\]

第 3 拍二阶差分为 0，\(\Delta u=0\)，\(u\) 停在 33。因为 \(K_i=0\)，常值误差的稳态输出就是 \(K_p e\)，没有把静差积到 0 的机制。误差若一步跳到 10，

\[
\Delta u=33\cdot 10+10\cdot 10=430,
\]

被限幅到 150。这四条就是 `self_check_controller` 里的断言。

\(K_d\) 作用在二阶差分上。误差来回跳时微分项反号，用来压超调；传感器噪声会直接进 \(e_k-2e_{k-1}+e_{k-2}\)，所以直道上的 PWM 会有小抖动。

### 4.3 急弯分支

这不是第二套 \(K_p,K_i,K_d\)。固件在 `loop` 里看原始模拟量：模拟值大于 1000 的路数 \(n_{\mathrm{work}}\ge 3\)，或者 \(|e_{\mathrm{sensor}}|\ge 2\)，则这一拍送给电机的副本放大，基础 PWM 降低：

\[
(u_{\mathrm{cmd}},\, b)=
\begin{cases}
\big(\mathrm{clip}(1.5\, u_k,\,-150,\,150),\, 60\big), & \text{急弯},\\[4pt]
(u_k,\, 100), & \text{否则}.
\end{cases}
\]

\(1.5\) 倍只作用在这一拍的电机指令上，不写回 \(u_k\)。下一拍仍从未经 1.5 倍的 \(u_k\) 上累加增量。\(|e_{\mathrm{sensor}}|=2\) 换成米是

\[
2\sigma_x=2\cdot\frac{0.12}{7}\approx 0.0343\,\mathrm{m}.
\]

左右 PWM：

\[
\begin{aligned}
\mathrm{PWM}_L&=\mathrm{int}\,\mathrm{clip}(b+u_{\mathrm{cmd}},\,0,\,255),\\
\mathrm{PWM}_R&=\mathrm{int}\,\mathrm{clip}(b-u_{\mathrm{cmd}},\,0,\,255).
\end{aligned}
\]

\(u_{\mathrm{cmd}}>0\) 时左轮更快，由第 2.1 节，$\omega<0$，车向右转，用来纠正 \(e_{\mathrm{sensor}}>0\)（车偏左）。

固件把左电机 PWM 写在后退引脚、右电机写在前进引脚，两轮物理上都向前转。仿真只保留差速数值。

急弯且 \(|u_k|\) 较大时，一侧 PWM 会顶到 0。例如 \(b=60\)、\(u_{\mathrm{cmd}}=-150\)：左轮 PWM = 0，右轮 PWM = 210。对应轮速 0 和 \(0.003\times 210=0.63\,\mathrm{m/s}\)，

\[
v=0.315\,\mathrm{m/s},
\qquad
\omega=\frac{0.63}{0.16}=3.94\,\mathrm{rad/s},
\qquad
R_{\mathrm{turn}}=\frac{L}{2}=0.08\,\mathrm{m}.
\]

不饱和的估算 \(R\approx L b/(2|u|)\) 会给出约 3 cm，那个式子假设 \(b-u\) 仍为正。实际 \(b-u\) 被 0 截断，半径回到 \(L/2\)。

### 4.4 丢线之后 PID 会怎样

\(\sum w=0\) 时 \(e_{\mathrm{sensor}}\) 被置 0。增量式 PID 看到误差从某个值跳到 0，微分项先打出一拍反向增量，随后 \(e_k=e_{k-1}=e_{k-2}=0\)，\(\Delta u=0\)，\(u_k\) 停在接近 0 的数上。两轮 PWM 回到同一个 \(b\)，车几乎直行，没有寻线。这是这套固件的行为。

## 5. 误差动力学

MPC 不直接优化世界坐标，而优化“离路径有多远、车头歪了多少”。下面从定义推到代码里的 \(A,B\)。

### 5.1 Frenet 误差

参考点 \(q_r(s)=(x_r,y_r,\theta_r)\) 处的切向和左法向是

\[
\mathbf{t}=(\cos\theta_r,\sin\theta_r),
\qquad
\mathbf{n}=(-\sin\theta_r,\cos\theta_r).
\]

轴心相对参考点的位移 \((x-x_r,\, y-y_r)\) 在这两个方向上的分量：

\[
\begin{aligned}
e_x&=\cos\theta_r\,(x-x_r)+\sin\theta_r\,(y-y_r),\\
e_y&=-\sin\theta_r\,(x-x_r)+\cos\theta_r\,(y-y_r).
\end{aligned}
\]

\(e_y>0\) 表示车在路径左侧。\(\theta_r=0\)、参考点在原点时 \(e_y=y\)，与第 1.4 节的 \(d_{\mathrm{left}}\) 是同一个投影。航向误差

\[
e_\theta=\theta-\theta_r.
\]

代码里把它包到 \((-\pi,\pi]\)：

\[
\mathrm{wrap}(\alpha)=(\alpha+\pi)\bmod 2\pi-\pi.
\]

### 5.2 对 \(e_y\) 求时间导数

\(\theta_r\)、\(x_r\)、\(y_r\) 都随参考点移动。对 \(e_y\) 逐项求导：

\[
\begin{aligned}
\dot e_y
&=-\dot\theta_r\cos\theta_r\,(x-x_r)
-\sin\theta_r\,(\dot x-\dot x_r)\\
&\quad-\dot\theta_r\sin\theta_r\,(y-y_r)
+\cos\theta_r\,(\dot y-\dot y_r).
\end{aligned}
\]

含 \(\dot\theta_r\) 的两项收成纵向误差：

\[
\dot e_y
=-\omega_r e_x
-\sin\theta_r\,\dot x+\cos\theta_r\,\dot y
+\sin\theta_r\,\dot x_r-\cos\theta_r\,\dot y_r,
\]

其中 \(\omega_r=\dot\theta_r\)。代入车辆运动学 \(\dot x=v\cos\theta\)、\(\dot y=v\sin\theta\)：

\[
\begin{aligned}
-\sin\theta_r\cdot v\cos\theta+\cos\theta_r\cdot v\sin\theta
&=v\sin(\theta-\theta_r)
=v\sin e_\theta.
\end{aligned}
\]

参考点沿自身切线运动，\(\dot x_r=v_r\cos\theta_r\)，\(\dot y_r=v_r\sin\theta_r\)，法向分量为 0：

\[
\sin\theta_r\cdot v_r\cos\theta_r-\cos\theta_r\cdot v_r\sin\theta_r=0.
\]

于是

\[
\boxed{\dot e_y=-\omega_r e_x+v\sin e_\theta}.
\]

航向误差的导数就是两个角速度之差：

\[
\boxed{\dot e_\theta=\dot\theta-\dot\theta_r=\omega-\omega_r}.
\]

### 5.3 丢掉纵向误差

严格的误差状态是 \((e_x,e_y,e_\theta)\)。循迹只惩罚横向和航向。本仓库使用的二维模型是令 \(-\omega_r e_x\approx 0\)：

\[
\boxed{
\begin{aligned}
\dot e_y&=v\sin e_\theta,\\
\dot e_\theta&=\omega-\omega_r.
\end{aligned}
}
\]

这一步的含义是：纵向偏差与曲率的乘积当作小量，或者任务不关心沿路径走快还是走慢。直角弯上 \(\omega_r\) 很大，若 \(e_x\) 也有几厘米，这一项并不小。第一版没有把 \(e_x\) 放进状态。

\(v\sin e_\theta\) 是非线性的。\(e_\theta\) 大时，同样的车速会让横向误差涨得更快。PID 的输入里没有 \(e_\theta\)，只能等 \(e_y^{\mathrm{sensor}}\) 变大后再加差速。MPC 的模型里有这一项，优化器可以通过 \(\omega\) 先把 \(e_\theta\) 按住。

### 5.4 在参考点附近线性化

二次规划要求预测是决策变量的仿射函数。在

\[
e_y\approx 0,\qquad e_\theta\approx 0,\qquad v\approx v_r,\qquad \omega\approx\omega_r
\]

处做一阶展开。令 \(f(v,e_\theta)=v\sin e_\theta\)。

\[
\frac{\partial f}{\partial e_\theta}=v\cos e_\theta,
\qquad
\frac{\partial f}{\partial v}=\sin e_\theta.
\]

在 \((v_r,0)\)：

\[
f\approx v_r\cdot e_\theta+\sin 0\cdot(v-v_r)=v_r e_\theta.
\]

速度偏差的系数是 \(\sin 0=0\)，一阶意义上不进入 \(\dot e_y\)。\(\dot e_\theta=\omega-\omega_r\) 已经是线性的。定义决策变量

\[
\Delta u=\omega-\omega_r,
\]

得到

\[
\boxed{
\begin{aligned}
\dot e_y&=v_r e_\theta,\\
\dot e_\theta&=\Delta u.
\end{aligned}
}
\qquad
\dot e=A_c e+B_c\Delta u,
\]

\[
A_c=\begin{bmatrix}0&v_r\\0&0\end{bmatrix},
\qquad
B_c=\begin{bmatrix}0\\1\end{bmatrix},
\qquad
e=\begin{bmatrix}e_y\\ e_\theta\end{bmatrix}.
\]

\(A_c\) 里的 \(v_r\) 是线性化点的信息。\(v_r\) 越大，同样的航向误差把横向误差推开得越快，优化器就越不愿意留下 \(e_\theta\)。

本仓库只优化 \(\omega\)。线速度不进决策变量，按路段提前给定。若同时优化 \(v-v_r\)，\(B_c\) 会多一列，而且因为 \(\partial f/\partial v\) 在 \(e_\theta=0\) 处为 0，这一列在当前线性化点上对 \(\dot e_y\) 没有一阶贡献。

### 5.5 前向欧拉离散

控制周期 \(T_s=0.02\,\mathrm{s}\)。前向欧拉

\[
e_{k+1}=e_k+T_s\dot e_k
=(I+A_c T_s)e_k+B_c T_s\,\Delta u_k.
\]

\[
\boxed{
A=\begin{bmatrix}1& v_r T_s\\ 0& 1\end{bmatrix},
\qquad
B=\begin{bmatrix}0\\ T_s\end{bmatrix},
\qquad
e_{k+1}=A e_k+B\Delta u_k.
}
\]

展开就是代码 `_ab` 里的两行：

\[
\begin{aligned}
e_{y,k+1}&=e_{y,k}+v_r T_s\, e_{\theta,k},\\
e_{\theta,k+1}&=e_{\theta,k}+T_s\Delta u_k.
\end{aligned}
\]

\(v_r=0.30\,\mathrm{m/s}\) 时 \(v_r T_s=0.006\,\mathrm{m/rad}\)。若 \(e_\theta=0.1\,\mathrm{rad}\) 且不纠正，一拍横向误差增加 0.6 mm；预测 10 拍（0.2 s）且 \(e_\theta\) 保持不变时，累加约 6 mm。前向欧拉的局部截断误差是 \(O(T_s^2)\)。

### 5.6 参考速度和参考角速度

PID 没有显式的 \(v_r\)。MPC 用与 `base_speed` 相同的几何规则：

\[
v_r=
\begin{cases}
0.18\,\mathrm{m/s}, & \text{当前投影落在急弯或直角弯},\\
0.30\,\mathrm{m/s}, & \text{直道、缓弯、S 弯}.
\end{cases}
\]

\(0.30\) 对应基础 PWM 100，\(0.18=0.003\times 60\) 对应急弯基础 PWM 60。参考角速度

\[
\omega_r=\kappa_r v_r.
\]

直线上 \(\omega_r=0\)。急弯 \(\kappa_r=4\,\mathrm{m}^{-1}\)，低速参考下 \(\omega_r=0.72\,\mathrm{rad/s}\)。直角弯 \(\kappa_r=40\,\mathrm{m}^{-1}\)，

\[
\omega_r=40\times 0.18=7.2\,\mathrm{rad/s}.
\]

角速度上限由 PWM 反推。一侧轮速最大、另一侧为 0 时

\[
|\omega|_{\max}=\frac{k_v\cdot 255}{L}=\frac{0.765}{0.16}=4.78125\,\mathrm{rad/s}.
\]

\(7.2>4.781\)，所以即便忽略第 2.1 节的半径下限，按 0.18 m/s 的计划速度，直角弯需要的角速度也超出左右轮 PWM 能产生的范围。跟踪误差在这里是约束不可行，不是求解器没收敛。

变化率约束取这个上限的一半：

\[
|\omega_k-\omega_{k-1}|\le \Delta\omega_{\max}=2.390625\,\mathrm{rad/s}.
\]

### 5.7 预测时域内 \(A,B\) 固定

一般的线性时变 MPC 会在第 \(k\) 步用前方弧长 \(s+v_r k T_s\) 处的 \(v_r,\omega_r\) 各做一套 \(A_k,B_k\)。那样的话，未来曲率在优化问题建立的时候就已经进了 \(\omega_{r,k}\) 和 \(A_k\)。

本仓库的第一版不这样做。`freeze_linearization = True`，预测时域内的 \(A,B\) 和 \(\omega_r\) 都冻结成当前投影点的值。`_prediction_maps` 只用一个 \(v_r\) 生成 \(A\) 的幂。因此优化器看不到“前方 0.2 s 曲率会跳到 40”。弯道内 \(\Delta u\neq 0\)，是在修已经进入状态的 \(e_y,e_\theta\)，以及当前这一点的 \(\omega_r\) 前馈。

当前 \(\omega_r\) 仍然有前馈作用：决策变量是 \(\Delta u=\omega-\omega_r\)，施加到车上的是 \(\omega_r+\Delta u_0\)。车一旦投影到急弯上，\(\omega_r\) 在同一拍从 0 跳到 \(0.72\,\mathrm{rad/s}\)，指令可以跟着跳。这是投影点切换带来的前馈，不是预测时域里的未来曲率序列。

传感器前视会让 \(e_y^{\mathrm{sensor}}\) 在轴心进弯之前就变大。那是第 3.4 节的几何，PID 也享有。

### 5.8 测量方程

控制器看到的不是几何 \(e_y\)，而是

\[
\begin{aligned}
e_y^{\mathrm{meas}}&=\sigma_x\, e_{\mathrm{sensor}},\\
e_\theta^{\mathrm{meas}}&=\mathrm{wrap}(\theta-\theta_r)+n_\theta,
\qquad n_\theta\sim N(0,0.01^2).
\end{aligned}
\]

\(e_{\mathrm{sensor}}\) 来自带噪声的 8 路模拟量。\(e_\theta\) 在仿真里用真值加噪声。实物上 8 路红外测不到航向，上实物需要估计器或 IMU。丢线时 \(e_{\mathrm{sensor}}=0\)，优化器会以为横向误差消失了。

## 6. 线性 MPC

预测步数与控制步数相同，\(N=10\)，时域长度 \(0.2\,\mathrm{s}\)。

### 6.1 把 \(N\) 步写成矩阵

冻结的 \(A,B\) 下，

\[
\begin{aligned}
e_1&=A e_0+B\Delta u_0,\\
e_2&=A^2 e_0+AB\Delta u_0+B\Delta u_1,\\
&\;\vdots\\
e_N&=A^N e_0+\sum_{j=0}^{N-1} A^{N-1-j} B\,\Delta u_j.
\end{aligned}
\]

堆叠 \(E=(e_1^\top,\ldots,e_N^\top)^\top\)、\(\Delta U=(\Delta u_0,\ldots,\Delta u_{N-1})^\top\)：

\[
E=\Phi e_0+\Gamma \Delta U.
\]

\(\Phi\) 是 \(2N\times 2\)，第 \(k\) 个 \(2\times 2\) 块是 \(A^k\)（\(k=1,\ldots,N\)）。\(\Gamma\) 是 \(2N\times N\)，第 \(k\) 行块、第 \(j\) 列（\(j=0,\ldots,k-1\)）是 \(A^{k-1-j}B\)，未来的输入列为 0。以 \(N=2\) 写成完整矩阵：

\[
\Phi=\begin{bmatrix}A\\ A^2\end{bmatrix},
\qquad
\Gamma=\begin{bmatrix}B&0\\ AB&B\end{bmatrix}.
\]

`_prediction_maps` 用 \(A\) 的幂填这两块。\(B\) 的形状是 \(2\times 1\)，所以每一列对应一个标量 \(\Delta u_j\)。

### 6.2 cvxpy 后端的目标函数

记 \(\Delta u_{-1}=\Delta u_{\mathrm{prev}}\) 为上一拍真正执行的 \(\Delta u\)。cvxpy 问题 `_build_cvx_problem` 的代价是

\[
\begin{aligned}
J
&=e_0^\top Q e_0
+\sum_{k=0}^{N-2} e_{k+1}^\top Q e_{k+1}
+e_N^\top P e_N\\
&\quad+\sum_{k=0}^{N-1}
\Big(
R\,\Delta u_k^2
+S(\Delta u_k-\Delta u_{k-1})^2
\Big).
\end{aligned}
\]

\(e_0^\top Q e_0\) 不含决策变量，不改变最优解，写进去只是为了让第 0 拍的代价和后面的形式一致。\(e_1,\ldots,e_{N-1}\) 用 \(Q\)，终端 \(e_N\) 用 \(P\)，终端不再加一份 \(Q\)。

\[
Q=\mathrm{diag}(5000,\,80),
\qquad
P=\mathrm{diag}(10000,\,200),
\qquad
R=0.1,
\qquad
S=1.
\]

\(Q\) 的单位要和状态的单位一起看。\(e_y\) 是米。\(0.02^2\times 5000=2\)，和 \(R(\Delta u)^2\) 在 \(\Delta u\sim 1\,\mathrm{rad/s}\) 时同一量级。若误用 \(Q=\mathrm{diag}(10,1)\)，\(0.02^2\times 10=0.004\)，横向惩罚会被 \(R\) 和 \(S\) 淹没，车几乎不纠偏。航向权重 80 小于横向权重 5000，因为任务直接关心的是别离开黑线；航向是通过 \(e_{y,k+1}=e_{y,k}+v_r T_s e_{\theta,k}\) 间接起作用的。

\(S(\Delta u_k-\Delta u_{k-1})^2\) 惩罚控制增量的变化。PID 没有这一项。它压的是“这一拍猛打、下一拍猛打回去”。

\(P\) 是终端代价，用来减轻“把误差推到第 \(N\) 步之外”的短视。这里的 \(P\) 是经验对角阵，没有做终端代价等于无限时域代价、也没有做终端集不变性的证明。

### 6.3 默认 OSQP 后端的二次型

安装了 `osqp` 和 `scipy` 时，求解器是 `osqp_native`，不走上面的 cvxpy 问题。它把预测误差的二次项收成

\[
E^\top \bar Q E
=(\Phi e_0+\Gamma\Delta U)^\top \bar Q\,(\Phi e_0+\Gamma\Delta U),
\]

其中 \(\bar Q\) 是块对角阵，前 \(N-1\) 块为 \(Q\)，最后一块为 \(P\)。展开后与 \(\Delta U\) 有关的部分是

\[
\Delta U^\top(\Gamma^\top\bar Q\Gamma)\,\Delta U
+2(\Phi e_0)^\top\bar Q\Gamma\,\Delta U.
\]

控制正则写成 \(\Delta U^\top H_u\Delta U\)，再加线性项 \(-2S\,\Delta u_{\mathrm{prev}}\Delta u_0\)。OSQP 的标准形是 \(\tfrac12 x^\top P_{\mathrm{osqp}} x+q^\top x\)，所以代码取

\[
P_{\mathrm{osqp}}=2H,
\qquad
H=\Gamma^\top\bar Q\Gamma+H_u,
\qquad
q=2\Gamma^\top\bar Q\Phi e_0,
\]

并把 \(q_0\) 减去 \(2S\,\Delta u_{\mathrm{prev}}\)。\(H\) 在送进求解器前被对称化。

\(H_u\) 由 `_hu` 填出。\(N\ge 2\)、\(S>0\) 时它的结构是

\[
\begin{aligned}
(H_u)_{00}&=R+2S,\\
(H_u)_{ii}&=R+3S, && i=1,\ldots,N-2,\\
(H_u)_{N-1,N-1}&=R+2S,\\
(H_u)_{i,i-1}=(H_u)_{i-1,i}&=-S, && i=1,\ldots,N-1.
\end{aligned}
\]

cvxpy 那份和式 \(R\sum\Delta u_k^2+S\sum(\Delta u_k-\Delta u_{k-1})^2\) 对应的 Hessian 对角是 \((R+2S,\ R+2S,\ \ldots,\ R+2S,\ R+S)\)。`_hu` 从第 2 个控制量开始比这份和式多一份 \(S\)。默认后端是 `osqp_native`，数值结果以 `_hu` 为准。cvxpy 只在没有原生 OSQP 绑定时启用。两边都是同一个 \(A,B\)、同一组约束和同一套 \(Q,R,S,P\)，差在控制增量二次项的对角权重。

### 6.4 约束

每一拍的角速度

\[
\omega_{\min}\le \omega_r+\Delta u_k\le\omega_{\max},
\qquad
\omega_{\min}=-4.78125,\ 
\omega_{\max}=4.78125.
\]

变化率约束作用在 \(\omega\) 上，不是直接作用在 \(\Delta u\) 的差分上。时域内 \(\omega_r\) 被冻成同一个数，所以 \(k\ge 1\) 时

\[
|(\omega_r+\Delta u_k)-(\omega_r+\Delta u_{k-1})|=|\Delta u_k-\Delta u_{k-1}|\le\Delta\omega_{\max}.
\]

第 0 拍与上一拍真正执行的角速度比：

\[
|(\omega_r+\Delta u_0)-\omega_{\mathrm{prev}}|\le\Delta\omega_{\max}.
\]

可选的横向软约束是

\[
|e_{y,k}|\le e_{y,\max}+\varepsilon_k,
\qquad
\varepsilon_k\ge 0,
\qquad
e_{y,\max}=2\sigma_x\approx 0.0343\,\mathrm{m},
\]

代价里再加 \(M\varepsilon_k\)，\(M=10^4\)。`use_soft_ey` 默认是 `False`，这组约束不进 QP。\(e_{y,\max}\) 仍画在 MPC 误差图上，对应 PID 的 \(|e_{\mathrm{sensor}}|\ge 2\)。

### 6.5 只执行第一步

QP 给出 \(\Delta u_0^\star,\ldots,\Delta u_{N-1}^\star\)。车上只施加

\[
\omega_{\mathrm{cmd}}=\mathrm{clip}(\omega_r+\Delta u_0^\star,\,\omega_{\min},\,\omega_{\max}),
\]

并把执行后的 \(\Delta u_0=\omega_{\mathrm{cmd}}-\omega_r\) 和 \(\omega_{\mathrm{cmd}}\) 存成下一拍的 \(\Delta u_{\mathrm{prev}}\)、\(\omega_{\mathrm{prev}}\)。然后重新测量、重新投影、重新求解。这是滚动时域。

求解失败时的后备律是

\[
\Delta u_0=\mathrm{clip}\!\left(\frac{-2 e_y}{\max(v_r,10^{-3})},\,-\Delta\omega_{\max},\,\Delta\omega_{\max}\right),
\]

再按同样的方式把 \(\omega_r+\Delta u_0\) 夹进角速度极限。\(e_y>0\) 时后备 \(\Delta u_0<0\)，车向右转。

### 6.6 从 \((v,\omega)\) 回到 PWM

计划线速度就是本节的 \(v_r\)，不另做速度闭环。

\[
v_L=v_r-\tfrac12\omega_{\mathrm{cmd}} L,
\qquad
v_R=v_r+\tfrac12\omega_{\mathrm{cmd}} L.
\]

这是第 2.1 节的反解。每只轮子：

\[
\mathrm{PWM}=
\begin{cases}
0, & v_{\mathrm{wheel}}\le 0,\\[4pt]
\mathrm{clip}\big(\max(v_{\mathrm{wheel}}/k_v,\,30),\,0,\,255\big), & \text{否则}.
\end{cases}
\]

再 `int` 截断。速度为正但反解 PWM 低于死区时，代码把它抬到 30，而不是放成 0。植物模型则是“PWM < 30 则轮速为 0”。抬到死区边界是为了避免指令落进死区后轮速突然变 0。

符号自检：\(e_y=0.02\,\mathrm{m}\)、\(e_\theta=0\)、\(v_r=0.30\)、\(\omega_r=0\) 时，\(\Delta u_0\) 必须为负。`self_check_mpc_direction` 检查这一点。

### 6.7 权重是怎么定的

\(N=10\)。\(Q\) 按米制误差重标定过：横向权重大于航向。\(R=0.1\) 允许偏离 \(\omega_r\)。\(S=1\) 用来抑制 \(\Delta u\) 在急弯上的一拍跳变。\(P=2Q\) 量级，没有 Lyapunov 方程或 LQR 代数黎卡提的解作为依据。

这些数是控制器参数。车辆、传感器、路径、种子不随这组数改变。

## 7. 闭环在一个周期里做的事

PID 与 MPC 的植物相同，差别只在“误差 → PWM”那一段。

1. 把轴心投影到路径，得到 \(s\)、几何 \(e_y\)、\(\theta_r\)、\(\kappa_r\)。
2. 计算 8 路传感器的世界坐标和到线距离，抽模拟量噪声，得到 \(e_{\mathrm{sensor}}\)。
3. PID：把 \(e_{\mathrm{sensor}}\) 送进增量式 PID，再按急弯规则混合 PWM。MPC：\(e_y^{\mathrm{meas}}=\sigma_x e_{\mathrm{sensor}}\)，\(e_\theta^{\mathrm{meas}}=\mathrm{wrap}(\theta-\theta_r)+n_\theta\)，按当前路段取 \(v_r\)，\(\omega_r=\kappa_r v_r\)，解 QP，反解 PWM。
4. PWM 经死区变成轮速指令，再经 \(\tau=0.18\,\mathrm{s}\) 的精确离散滞后。
5. 由轮速计算 \(v,\omega\)，RK4 推进一步。
6. 轴心离路径超过 0.55 m 并持续 1.6 s，则再记一拍离线位姿后停止。否则直到轴心进入终点前 \(\ell+0.08\,\mathrm{m}\) 且到线距离小于 8 cm，或时间超过路径长度按 0.20 m/s 估计的上限加 2.5 s。

终点提前停，是为了不让车头传感器冲出黑线末端、被记成一次假丢线。

## 8. 对比指标怎么算

对比脚本不重跑动力学。它读两份原始时序，用同一条路径重算几何量。

- \(e_y\)：轴心投影的 \(d_{\mathrm{left}}\)，单位 m，左侧为正。
- \(e_\theta\)：\(\mathrm{wrap}(\theta-\theta_r)\)，由时序里的 \(\theta\) 和弧长上的路径切线重算。
- \(\omega_{\mathrm{cmd}}\)：MPC 用控制器输出。PID 没有 \(\omega\) 指令，于是把左右 PWM 按第 2.3 节的死区映射成轮速，再用 \((v_R-v_L)/L\) 反算滞后之前的角速度。
- \(\omega_r=\kappa_r v_r\)，分段速度与第 5.6 节相同。

分区：

- 直道/缓弯：段类型是 straight 或 gentle，且弧长还在直角弯前视窗 \(s_0-\ell\) 之前。避免丢线后投影落到直道5上，把离线误差算进直道。
- S 弯：段类型 `s`。
- 急弯：段类型 `sharp`。
- 直角弯指标窗口：从弧长进入 \(s_0-0.075\,\mathrm{m}\) 开始，直到连续 0.2 s（10 拍）满足“已在下一段直道、到线距离 \(<0.05\,\mathrm{m}\)、\(|e_y|<0.03\,\mathrm{m}\)”。回不去就计到仿真结束。
- 图上色带比指标窗口短。急弯色带是投影落在 sharp 段。直角弯色带是弧长落在 \([s_0-0.075,\, s_1+0.12]\,\mathrm{m}\) 且到线距离 \(\le 0.18\,\mathrm{m}\)。色带取两车时间区间的并集。

丢线有两个定义，连续不少于 3 拍才计一次：

- 几何丢线：\(|e_y|>0.06\,\mathrm{m}\)（阵列半宽）。
- 传感器丢线：\(\sum w=0\)。

控制量变化率是相邻拍 \(\omega_{\mathrm{cmd}}\) 之差除以 \(T_s\) 之后的 RMS。直角弯的曲率阶跃会抬高弯道内的这个数；直道/缓弯上的变化率用来看持续抖动。

入口提前量：在 \(\omega_r\) 发生曲率阶跃之前的 0.55 s 内，同方向 \(\omega_{\mathrm{cmd}}\) 第一次超过 \(0.5\,\mathrm{rad/s}\) 的时刻。直道起伏大约 \(\pm 0.2\,\mathrm{rad/s}\)，0.5 用来避开这些起伏。阶跃前一拍仍不低于这个幅度，记为“保持”；否则记为“已回落”。和阶跃同一拍才抬起来的，提前量为 0。

标注图用的是另一套更松的读数，只用于三张大字图，不替换上面的指标：

- 急弯峰值：该段上 \(|e_y|\) 的最大值，画成厘米。
- 打角时刻：急弯入口 \(|\kappa_r|\) 达到 \(4\,\mathrm{m}^{-1}\) 量级之前，\(|\omega_{\mathrm{cmd}}|\) 最后一次低于 \(0.40\,\mathrm{rad/s}\) 的下一拍。
- 末段回正：直道5 上，从最后一次 \(|e_y|\ge 2\,\mathrm{cm}\) 到第一次 \(|e_y|<0.5\,\mathrm{cm}\) 的时间。回不到 0.5 cm 就记“未回正”。

## 9. 种子 42 上看到的现象

下面的数字来自种子 42、路径总长 9.424 m 的一次闭环，口径是 `comparison/compare.py`，写在 `results/data/comparison_metrics.md`。PID 跑了 26.08 s 后因离线结束，MPC 跑了 32.08 s 并走完直道5。同一次求解里 QP 后端是 `osqp_native`，单步平均 0.18 ms，最大 3.26 ms。

传感器丢线在这张表里只累计连续不少于 3 拍的事件，PID 合计 3.22 s。`pid_metrics.md` 把每一拍 \(\sum w=0\) 都算进丢线时长，同一段轨迹是 176 拍、3.52 s，事件次数仍是 18。两边没有改控制律，差在统计窗口。

| 指标 | PID | MPC |
| --- | ---: | ---: |
| 直道/缓弯 RMS \(e_y\) | 2.2 mm | 3.0 mm |
| S 弯 RMS \(e_y\) | 3.4 mm | 2.7 mm |
| 急弯最大 \(\|e_y\|\) | 8.6 mm | 5.5 mm |
| 急弯 RMS \(e_y\) | 5.3 mm | 2.7 mm |
| 急弯平均速度 | 0.297 m/s | 0.193 m/s |
| 直角弯最大 \(\|e_y\|\) | 0.889 m | 0.094 m |
| 直角弯 RMS \(e_y\) | 0.489 m | 0.055 m |
| 全程 \(\omega_{\mathrm{cmd}}\) 变化率 RMS | 43.7 rad/s² | 7.7 rad/s² |
| 直道/缓弯变化率 RMS | 4.74 rad/s² | 4.74 rad/s² |
| 几何丢线 | 1 次，3.84 s | 1 次，1.08 s |
| 传感器丢线 | 18 次，3.22 s | 3 次，0.58 s |
| 因离线结束 | 是 | 否 |
| 末段直道回到 0.5 cm 以内 | 未回正 | 约 2.7 s |

直道上 PID 的 RMS 比 MPC 小约 0.8 mm。两边都在毫米级，相对差看起来大，绝对差小于传感器噪声和线宽。直道变化率都是 4.74 rad/s²，抖动幅度同一量级。

急弯上 MPC 的峰值约 5.5 mm，PID 约 8.6 mm，MPC 低 36%。同一段上 MPC 平均速度 0.19 m/s，PID 仍接近 0.30 m/s。急弯误差更小，有一部分来自 MPC 按路段把参考速度降到 0.18 m/s，不全是反馈律的差别。按第 5.7 节的 0.5 rad/s 判据，MPC 的 \(\omega_{\mathrm{cmd}}\) 是和 \(\omega_r\) 同一拍抬起来的，阶跃前 0.55 s 内没有超过 0.5 rad/s。标注图用 0.40 rad/s 去读同一入口，提前量也是 0.00 s。

直角弯之后 PID 的几何误差一直走到约 0.89 m，仿真结束，末段直道没有回到 1 cm。MPC 偏离约 9.4 cm，穿过 6 cm 的几何丢线线，然后在直道5上拉回来。两车都会在这里离开中心线；差别是 MPC 回得来。

PID 在直角弯入口按 0.5 rad/s 判据有约 0.26 s 的同向指令，和传感器前视时间同一量级。那是横杆已经压到弯上、加权误差变大之后的反馈，加上急弯分支把 \(u\) 乘 1.5、基础 PWM 降到 60。MPC 在直角弯阶跃前出现过约 1 rad/s 的同向鼓包，但阶跃前一拍已经回落，按这套定义不算保持住的预打角。

## 10. 和固件、和“理想 MPC”不一致的地方

| 项目 | 本仓库的处理 |
| --- | --- |
| 注释写“小于 250 则权重为 10” | 跟可执行代码，权重为 0 |
| 有的说明写“转弯换第二套 PID” | 固件里没有第二组增益，只有 \(u\times 1.5\) 和基础 PWM = 60 |
| 控制周期 | 固件由串口包驱动，没有固定 `delay`。仿真用 20 ms |
| 左轮接线 | 只保留差速 |
| \(e_\theta\) | 仿真用真值加 \(N(0,0.01^2)\)。实物 8 路红外测不到航向 |
| 预测模型 | 时域内 \(A,B,\omega_r\) 冻结在当前投影点 |
| 决策变量 | 只优化 \(\omega\)，\(v\) 按路段给定 |
| 横向软约束 | 默认关闭，图上仍标 \(\pm e_{y,\max}\) |
| 终端代价 | 对角经验值，无稳定性证明 |
| 默认 QP | `osqp_native` 的 \(H_u\) 与 cvxpy 和式的 Hessian 对角不完全相同 |
| 传感器曲线 | 高斯加噪声，不是标定表 |
| 直角弯 | 半径 2.5 cm，小于 \(L/2=8\,\mathrm{cm}\)，且 \(v_r=0.18\) 时所需 \(\omega_r=7.2\,\mathrm{rad/s}\) 超过 4.78 rad/s 的上限 |

## 11. 代码与公式的对应

| 公式 | 位置 |
| --- | --- |
| 直线 / 圆弧位姿 | `core/path.py` 的 `pose_on_segment` |
| 投影与 \(d_{\mathrm{left}}\) | `ReferencePath.project` |
| 加权误差 | `core/car_model.py` 的 `position_error` |
| PWM 死区、轮速滞后、差速、RK4、模拟量 | `core/car_model.py` |
| 增量式 PID、急弯分支、左右 PWM | `controllers/pid.py` |
| \(A,B\)、预测矩阵、\(H_u\)、约束、滚动 | `controllers/mpc.py` 的 `LinearMPC` |
| \((v,\omega)\to\mathrm{PWM}\) | `speed_omega_to_pwm` |
| 闭环顺序 | `simulators/pid_sim.py`、`simulators/mpc_sim.py` 的 `simulate` |
| 几何对比指标 | `comparison/compare.py` |
| 标注图 | `comparison/generate_plots.py` |

PID 入口会先跑误差公式、增量式 PID 和“车在路径左侧时误差为正”的自检。MPC 入口额外检查 \(e_y>0\) 时 \(\Delta u<0\)。自检失败会直接抛出 `AssertionError`，不会继续画图。

## 许可

MIT License，见 `LICENSE`。
