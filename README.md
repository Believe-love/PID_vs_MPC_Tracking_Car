# 智能小车循迹：PID 与 MPC

同一辆差速小车、同一条黑线、同一套八路红外噪声，分别用增量式 PID 和线性模型预测控制闭环。PID 复现实物固件里已经在跑的控制律。MPC 只替换控制器，用来回答一个具体问题：把横向误差和航向误差写成可预测的状态之后，二次规划能不能在急弯和直角弯上比纯反馈少偏离、并且在丢线之后回到线上。

本文从轮子怎么产生速度写到二次规划怎么一次解出未来 $N$ 步的角速度。公式与 `controllers/mpc.py`、`controllers/pid.py`、`core/car_model.py` 一致。和“理想 MPC”不一致的地方单独写明，不把代码没做的预瞄写成实验结果。

种子 42 的一次闭环已放在 `results/`。复现命令是 `py -3 run_all.py`。

## 1. 项目背景与硬件限制

### 1.1 实物在测什么、在驱动什么

小车是差速驱动。左右轮各有一个电机，PWM 取值 $0$ 到 $255$，没有倒车通道。轮距


$$
L = 0.16\,\mathrm{m}.
$$


车体俯视坐标以前向为 $x_b$、左侧为 $y_b$。世界坐标 $x$ 向右、$y$ 向上，航向 $\theta$ 从 $x$ 轴逆时针量到车头。下文的位姿都指左右轮轴线中点，不是车头。

循迹传感器是车头前方一根横杆上的 8 路红外。固件给每路一个横坐标


$$
\mathrm{SENSOR\_X} = [-3.5,\ -2.5,\ -1.5,\ -0.5,\ 0.5,\ 1.5,\ 2.5,\ 3.5].
$$


黑线压在某一路下面时，该路模拟量变大，白地则变小。固件不建地图，每个控制周期只收到这一组模拟量，算出一个横向误差，再写成左右轮 PWM。仿真里横杆在轴心前方 $\ell = 0.075\,\mathrm{m}$，最外侧两路中心相距 $0.12\,\mathrm{m}$。这两个长度是传感器几何假设，固件源码里没有。

实物 `loop()` 由串口数据包到达驱动，没有固定的 `delay`。仿真把控制周期取成


$$
T_s = 0.02\,\mathrm{s}
$$


（50 Hz），使两边可对齐。随机种子固定为 42。模拟量噪声只在生成 8 路读数时抽取；MPC 若还要航向噪声，抽在这 8 个数之后，因此两边的红外噪声序列相同。

### 1.2 板上的增量式 PID 做了什么

误差是 8 路读数的加权重心 $e_{\mathrm{sensor}}$（第 4.1 节）。增量式 PID 为


$$
\begin{aligned}
\Delta u_k
&= K_p(e_k-e_{k-1})
+ K_i e_k
+ K_d(e_k-2e_{k-1}+e_{k-2}),\\
u_k
&= \mathrm{clip}(u_{k-1}+\Delta u_k,\ -150,\ 150).
\end{aligned}
$$



$$
K_p = 33,\qquad K_i = 0,\qquad K_d = 10.
$$


$K_i$、$K_d$ 在固件注释里已经含采样周期，公式中不再乘、除 $T_s$。$K_i=0$，没有把常值偏差积掉的通道。

急弯分支不是第二套 PID。模拟值大于 1000 的路数不少于 3，或者 $|e_{\mathrm{sensor}}|\ge 2$ 时，这一拍送到电机的副本乘 $1.5$ 后再限幅，基础 PWM 从 100 降到 60。$1.5$ 不写回 $u_k$。左右轮


$$
\mathrm{PWM}_L = \mathrm{int}\,\mathrm{clip}(b+u_{\mathrm{cmd}},0,255),\qquad
\mathrm{PWM}_R = \mathrm{int}\,\mathrm{clip}(b-u_{\mathrm{cmd}},0,255).
$$


$u_{\mathrm{cmd}}>0$ 时左轮更快，车向右转。固件约定 $e_{\mathrm{sensor}}>0$ 表示车偏左，需要右转，符号是闭合的。

### 1.3 这套 PID 过不了哪一类弯

它的信息只有当前这一拍的传感器重心，加上两拍历史，用来做差分。结构上有四条限制。

1. 反馈是纯输出反馈。状态里没有航向误差 $e_\theta$。车头已经歪了、但黑线仍压在中间两路时，$e_{\mathrm{sensor}}$ 可以仍然很小，差速要等重心偏到一侧才出现。
2. 没有路径曲率。直道和圆弧使用同一组 $K_p,K_d$。急弯分支只在误差或亮灯路数已经越限之后放大 $u$、降低基础 PWM，是事后切换，不是按前方曲率给出的 $\omega_r=\kappa v$。
3. 丢线把误差清零。8 路都低于阈值时 $e_{\mathrm{sensor}}=0$。增量式 PID 看到误差跳到 0，一两拍之后 $\Delta u=0$，$u_k$ 停在 0 附近，两轮 PWM 相同，车沿当前航向直行，没有搜索黑线的动作。
4. 约束是事后裁剪。$u$ 和 PWM 的限幅发生在控制量算完之后。优化问题里没有“未来 $N$ 步都不得超过角速度上限”这种约束，饱和只影响已经发出的这一拍。

种子 42 上，直道、缓弯和半径 $0.25\,\mathrm{m}$ 的急弯仍能贴住，轴心偏离在 1 cm 量级。半径 $0.025\,\mathrm{m}$ 的直角弯之后，几何横向误差走到 $0.89\,\mathrm{m}$，仿真因离线结束，末段长直道没有回到 1 cm 以内。这就是引入 MPC 要面对的失败样本，不是把 PID 增益调好看之后的曲线。

### 1.4 为什么用误差状态上的 MPC

把“离线多远、车头歪多少”写成状态 $e=(e_y,e_\theta)$，再写出 $\dot e$ 与角速度的关系，未来几步的误差就成为当前误差和未来角速度的函数。二次规划可以在这一步同时要求：预测误差小、角速度不要离开参考值太远、角速度不要一拍反向猛打，并且每一拍的 $\omega$ 落在左右轮 PWM 所能产生的区间里。解出来的序列只执行第一拍，下一拍用新测量重解。这是滚动时域。

它多出来的能力依赖三件 PID 没有的东西：航向误差、当前投影点的曲率前馈 $\omega_r=\kappa_r v_r$、以及一个显式的预测模型。第 6 节写明这三件在实物上分别缺测量、缺地图和缺算力。本仓库的第一版还把预测时域内的线性化冻结在当前拍（`freeze_linearization = True`），因此它还没有使用“前方路段的曲率序列”。实验结果必须按这个实现来读。

## 2. 代码架构与运行指南

早期工作分在三个目录里：PID 仿真、MPC 仿真、读取两边时序做对比。本仓库把它们收成一棵树，植物模型和路径只有一份，避免两套参数漂移。

```text
PID_vs_MPC_Tracking_Car/
├── README.md
├── LICENSE
├── requirements.txt
├── run_all.py                      # PID → MPC → 对比图 → 标注图
├── config/params.py                # 车辆、传感器、路径仿真参数，以及 MPC_CONFIG
├── core/
│   ├── car_model.py                # 差速、滞后、RK4、红外、加权误差
│   └── path.py                     # 直线/圆弧路径与投影
├── controllers/
│   ├── pid.py                      # 增量式 PID 与急弯分支
│   └── mpc.py                      # 线性 MPC
├── simulators/
│   ├── pid_sim.py                  # PID 闭环
│   ├── mpc_sim.py                  # MPC 闭环
│   ├── visualize_pid.py
│   └── visualize_mpc.py
├── comparison/
│   ├── compare.py                  # 指标与四张对比图
│   └── generate_plots.py           # 三张带数字标注的图
└── results/
    ├── data/
    └── figures/
```

| 作用 | 本仓库 | 早期独立目录 |
| --- | --- | --- |
| PID 闭环 | `simulators/pid_sim.py` | `tracking_car_sim/` |
| MPC 闭环 | `simulators/mpc_sim.py` | `mpc_sim/` |
| 对比与标注图 | `comparison/` | `comparison_report/` |

需要 Python 3.9 及以上。

```text
cd PID_vs_MPC_Tracking_Car
py -3 -m pip install -r requirements.txt
py -3 run_all.py
```

`python` 可用时把 `py -3` 换成 `python`。依赖为 `numpy`、`matplotlib`、`Pillow`、`pandas`、`cvxpy`、`osqp`、`scipy`。

分步运行：

```text
py -3 simulators/pid_sim.py
py -3 simulators/mpc_sim.py
py -3 comparison/compare.py
py -3 comparison/generate_plots.py
```

后两步读 `results/data/pid_timeseries.csv` 和 `results/data/mpc_timeseries.csv`。对比脚本把加工后的表写成 `pid_frame.csv`、`mpc_frame.csv`，不覆盖原始时序。图题优先用微软雅黑，没有中文字体时改为英文。

| 输出 | 内容 |
| --- | --- |
| `results/data/pid_timeseries.csv`、`pid_metrics.md` | PID 时序与分段指标 |
| `results/data/mpc_timeseries.csv`、`mpc_metrics.md` | MPC 时序、QP 耗时 |
| `results/figures/simulation.gif`、`error_vs_time.png`、`trajectory_compare.png`、`control_signals.png` | PID 图 |
| `results/figures/mpc_simulation.gif`、`mpc_error_vs_time.png`、`mpc_trajectory.png`、`mpc_control_signals.png` | MPC 图 |
| `results/figures/compare_trajectory.png`、`compare_error.png`、`compare_omega.png`、`compare_preview.png` | 对比 |
| `results/data/comparison_metrics.md` | 对比指标与文字结论 |
| `results/figures/compare_*_annotated.png`、`results/data/metrics_updated.md` | 答辩用的三个数字 |

## 3. 核心理论推导

符号先固定，后面不再改口。

| 符号 | 含义 |
| --- | --- |
| $(x,y,\theta)$ | 驱动轴中点的世界位姿 |
| $v_L,v_R$ | 左、右轮前进速度，不能为负 |
| $v,\omega$ | 轴心线速度、角速度，$\omega>0$ 为左转 |
| $s$ | 参考路径弧长 |
| $\kappa$ | 曲率，左转为正，$\kappa=\mathrm{d}\theta_r/\mathrm{d}s$ |
| $\theta_r,v_r,\omega_r$ | 参考航向、参考线速度、参考角速度 |
| $e_x,e_y$ | 相对参考点的纵向、横向误差，横向以路径左侧为正 |
| $e_\theta$ | 航向误差 $\theta-\theta_r$ |
| $e_{\mathrm{sensor}}$ | 8 路红外的加权重心，单位是 `SENSOR_X` |
| $\Delta u$ | $\omega-\omega_r$ |
| $T_s$ | 控制周期，0.02 s |
| $N$ | 预测步数，10 |

有两个不同的“横向误差”，后文不混用。

- 几何横向误差：轴心到路径垂足的左法向分量。对比图里的 $e_y$ 是这一个，时序列名是 `lat`。
- 控制器横向误差：$e_y^{\mathrm{meas}}=\sigma_x e_{\mathrm{sensor}}$。MPC 的优化器吃的是这一个。丢线时它被置 0，几何误差不会被置 0。

### 3.1 车辆运动学模型

#### 3.1.1 独轮车

把车看成平面上的一个点，速度方向被航向锁死，不能横向滑动。轴心速度的世界分量就是 $v$ 在 $\theta$ 方向上的投影，航向的变化率就是角速度：


$$
\boxed{
\dot x = v\cos\theta,
\qquad
\dot y = v\sin\theta,
\qquad
\dot\theta = \omega.
}
\tag{1}
$$


$v$ 的单位是 m/s，沿车头为正。本车没有倒车，进入植物的 $v$ 由左右轮速度平均得到，轮速非负时 $v\ge 0$。$\omega$ 的单位是 rad/s，逆时针为正。$\theta$ 是 rad，不是度。

这三条是运动学约束，不是牛顿第二定律。式中没有质量、没有转动惯量。电机跟不上 PWM 的现象不放进 (1)，放进第 3.1.3 节的轮速滞后。

若一个控制周期内 $v$ 和 $\omega$ 都当作常数，且 $\omega\neq 0$，(1) 可以积出来：


$$
\begin{aligned}
\theta(T_s) &= \theta_0+\omega T_s,\\
x(T_s) &= x_0+\frac{v}{\omega}\big(\sin(\theta_0+\omega T_s)-\sin\theta_0\big),\\
y(T_s) &= y_0-\frac{v}{\omega}\big(\cos(\theta_0+\omega T_s)-\cos\theta_0\big).
\end{aligned}
$$


代码没有调用这组闭式，而是对 (1) 做一步经典 RK4，步长等于 $T_s$。$v,\omega$ 在步内为常数时，航向的 RK4 与 $\theta_0+\omega T_s$ 相同；$x,y$ 是对 $\cos\theta(t)$、$\sin\theta(t)$ 的四阶近似。$T_s=0.02\,\mathrm{s}$、$|\omega|$ 约 $5\,\mathrm{rad/s}$ 时，一步转角约 $0.1\,\mathrm{rad}$，截断误差远小于红外噪声。仿真推进位姿用的是 RK4，MPC 预测误差用的是后面的欧拉离散，两套离散不要看成同一个公式。

#### 3.1.2 差速如何得到 $v$ 和 $\omega$

左右轮无侧滑，轮距 $L$。短时间 $\mathrm{d}t$ 内左轮走过 $v_L\mathrm{d}t$，右轮走过 $v_R\mathrm{d}t$。轴心位移是二者的平均，位移差形成转角：


$$
\boxed{
v=\frac{v_R+v_L}{2},
\qquad
\omega=\frac{v_R-v_L}{L}.
}
\tag{2}
$$


$v_L>v_R$ 时 $\omega<0$，车向右转。这与 PID 里“$u>0$ 则左轮更快、用来纠正车偏左”是同一条约定。

瞬时转弯半径 $R_{\mathrm{turn}}=v/\omega$（$\omega\neq 0$）。代入 (2)：


$$
R_{\mathrm{turn}}
=\frac{L}{2}\cdot\frac{v_R+v_L}{v_R-v_L}.
$$


PWM 只有 $[0,255]$，轮速不能为负。在 $v_L\ge 0$、$v_R\ge 0$ 中，


$$
\left|\frac{v_R+v_L}{v_R-v_L}\right|\ge 1,
$$


等号在一只轮子速度为 0、另一只为正时取到。因此


$$
\boxed{|R_{\mathrm{turn}}|\ge \frac{L}{2}=0.08\,\mathrm{m}.}
\tag{3}
$$


赛道上的直角弯半径是 $0.025\,\mathrm{m}$，小于 $8\,\mathrm{cm}$。只要两轮都不能倒转，任何前进速度都走不出这条圆弧的曲率。这是机构约束。后面看到两套控制器都在直角弯离开中心线，先用 (3) 解释，再用控制器差别解释谁回得来。

反解也常用。给定轴心的 $v$ 和 $\omega$，


$$
v_L=v-\frac12\omega L,
\qquad
v_R=v+\frac12\omega L.
\tag{4}
$$


MPC 把优化得到的 $\omega_{\mathrm{cmd}}$ 和计划速度 $v_r$ 代进 (4)，再换成 PWM。

#### 3.1.3 从 PWM 到轮速

稳态映射是过原点的直线加硬死区。$k_v=0.30/100=0.003\,\mathrm{m/s}$ 每 PWM，


$$
v_{\mathrm{wheel}}(\mathrm{PWM})=
\begin{cases}
0, & \mathrm{PWM}<30,\\
k_v\cdot\mathrm{PWM}, & \mathrm{PWM}\ge 30.
\end{cases}
\tag{5}
$$


PWM $=100$ 时轮速 $0.30\,\mathrm{m/s}$。死区边界上速度从 0 跳到 $0.09\,\mathrm{m/s}$。PID 的 PWM 在进入 (5) 之前已被 `int` 向 0 截断。

电机和车身对指令的跟踪用一阶滞后。连续时间


$$
\tau\dot v_w=v_{\mathrm{cmd}}-v_w,
\qquad \tau=0.18\,\mathrm{s}.
$$


$v_{\mathrm{cmd}}$ 在一个控制周期内为常数。分离变量：


$$
\int_{v_w(0)}^{v_w(T_s)}\frac{\mathrm{d}v}{v_{\mathrm{cmd}}-v}
=\int_0^{T_s}\frac{\mathrm{d}t}{\tau}.
$$


左边是 $-\ln\big|(v_{\mathrm{cmd}}-v_w(T_s))/(v_{\mathrm{cmd}}-v_w(0))\big|$。整理得精确离散


$$
\boxed{
v_w(T_s)=v_{\mathrm{cmd}}+\big(v_w(0)-v_{\mathrm{cmd}}\big)\,e^{-T_s/\tau}.
}
\tag{6}
$$


$T_s/\tau\approx 0.111$，每步只消去约 $10.5\%$ 的轮速误差。直角弯的弧长只有 $0.025\cdot\pi/2\approx 3.9\,\mathrm{cm}$，按 $0.18\,\mathrm{m/s}$ 也只有大约 $0.2\,\mathrm{s}$，轮速来不及在这段弧上完成大差速。$\tau$ 不改变 $K_p$，也不改变 PWM 到稳态速度的比例。

一侧 PWM 打满 255、另一侧为 0 时，


$$
|\omega|_{\max}=\frac{k_v\cdot 255}{L}=\frac{0.765}{0.16}=4.78125\,\mathrm{rad/s}.
\tag{7}
$$


这是 MPC 角速度约束的来源。它比 (3) 弱：在 $0.18\,\mathrm{m/s}$ 的计划速度下，可跟踪的最小半径约为 $0.18/4.78125\approx 0.038\,\mathrm{m}$，仍然大于 $0.025\,\mathrm{m}$。直角弯同时违反 (3) 和 (7)。

#### 3.1.4 八路红外测到的不是几何 $e_y$

第 $i$ 路在车体横向的位置取


y_{b,i}=-\sigma_x\,\mathrm{SENSOR\_X}_i,
\qquad
\sigma_x=\frac{0.12}{7}\approx 0.017143\,\mathrm{m/单位}.


负号是因为 `SENSOR_X` 向右为正，而 $y_b$ 向左为正。世界坐标


$$
\mathbf{p}_i
=
\begin{bmatrix}x\\ y\end{bmatrix}
+\ell\begin{bmatrix}\cos\theta\\ \sin\theta\end{bmatrix}
+y_{b,i}\begin{bmatrix}-\sin\theta\\ \cos\theta\end{bmatrix}.
$$


该点到路径中心线的欧氏距离记为 $d_i$。仿真用高斯近似黑线和光斑，不是标定曲线：


$$
a_i=8000\exp\!\left(-\frac{d_i^2}{2\sigma_a^2}\right)+n_i,
\qquad
\sigma_a=0.0082\,\mathrm{m},
$$


$n_i\sim N(0,120^2)$，再截断到 $[0,8000]$。

权重 $w_i=0$（当 $a_i<250$），否则 $w_i=a_i$。可执行代码如此；注释里的“小于 250 则权重为 10”不采用。加权重心


$$
e_{\mathrm{sensor}}
=
\begin{cases}
0, & \sum_i w_i=0,\\[4pt]
\dfrac{\sum_i \mathrm{SENSOR\_X}_i\, w_i}{\sum_i w_i}, & \text{否则}.
\end{cases}
\tag{8}
$$


这是一阶矩。只有坐标 $0.5$ 那一路有读数时，$e_{\mathrm{sensor}}=0.5$。全部低于 250 时固件返回 0，控制器会以为车在线的正上方。

横杆在轴心前方 $\ell$。车速 $v$ 时，轴心还要大约 $\ell/v$ 才到达传感器正在看的弧长。$v=0.30\,\mathrm{m/s}$ 时约 $0.25\,\mathrm{s}$，即 12 到 13 个控制周期。这个前视属于传感器安装，PID 和 MPC 都有。

### 3.2 误差动力学（Frenet 坐标系）

世界坐标里的 $(x,y,\theta)$ 不适合直接当调节量：同样 2 cm 的偏差，在弯道切线方向和法线方向的意义完全不同。循迹要罚的是法向偏离和航向偏离。为此在参考点上放一个随路径转动的标架。

#### 3.2.1 参考点与 Frenet 标架

路径用弧长 $s$ 参数化，$q_r(s)=(x_r(s),y_r(s),\theta_r(s))$。$\theta_r$ 是该点切线与 $x$ 轴的夹角。切向和左法向


$$
\mathbf{t}=(\cos\theta_r,\ \sin\theta_r),
\qquad
\mathbf{n}=(-\sin\theta_r,\ \cos\theta_r).
$$


$\mathbf{t}$ 指向路径前进方向，$\mathbf{n}$ 指向路径左侧。曲率按 $\mathrm{d}\mathbf{t}/\mathrm{d}s=\kappa\mathbf{n}$ 定义，于是 $\kappa>0$ 表示向左弯。直接验证：


$$
\frac{\mathrm{d}}{\mathrm{d}s}(\cos\theta_r,\sin\theta_r)
=\kappa(-\sin\theta_r,\cos\theta_r)
=\kappa\mathbf{n}.
$$


再求导一次，$\mathrm{d}\mathbf{n}/\mathrm{d}s=-\kappa\mathbf{t}$。

轴心相对参考点的位移在这两个方向上的投影定义为纵向误差和横向误差：


$$
\boxed{
\begin{aligned}
e_x
&= \mathbf{t}\cdot(x-x_r,\ y-y_r)
= \cos\theta_r\,(x-x_r)+\sin\theta_r\,(y-y_r),\\[4pt]
e_y
&= \mathbf{n}\cdot(x-x_r,\ y-y_r)
= -\sin\theta_r\,(x-x_r)+\cos\theta_r\,(y-y_r).
\end{aligned}
}
\tag{9}
$$


$e_y>0$ 表示车在路径左侧。检验：$\theta_r=0$、参考点在原点时 $e_y=y$。航向误差


$$
\boxed{e_\theta=\theta-\theta_r.}
\tag{10}
$$


$e_\theta>0$ 表示车头相对切线逆时针偏转，也就是相对路径偏向左侧。代码把它包回 $(-\pi,\pi]$：


$$
\mathrm{wrap}(\alpha)=(\alpha+\pi)\bmod 2\pi-\pi.
$$


(9) 与 (10) 合在一起，就是误差状态 $e=(e_y,e_\theta)^\top$。$e_x$ 先保留在推导里，决定丢掉它是下一步的建模选择，不是定义的一部分。

#### 3.2.2 对 $e_y$ 求时间导数

参考点沿路径运动，所以 $\theta_r$、$x_r$、$y_r$ 都是时间的函数。对 (9) 的第二式使用乘积法则。记 $\omega_r=\dot\theta_r$。

第一项 $-\sin\theta_r\cdot(x-x_r)$ 的导数：


$$
-\cos\theta_r\cdot\omega_r\cdot(x-x_r)-\sin\theta_r\cdot(\dot x-\dot x_r).
$$


第二项 $\cos\theta_r\cdot(y-y_r)$ 的导数：


$$
-\sin\theta_r\cdot\omega_r\cdot(y-y_r)+\cos\theta_r\cdot(\dot y-\dot y_r).
$$


两式相加：


$$
\begin{aligned}
\dot e_y
&= -\omega_r\big[\cos\theta_r\,(x-x_r)+\sin\theta_r\,(y-y_r)\big]\\
&\quad -\sin\theta_r\,(\dot x-\dot x_r)
+\cos\theta_r\,(\dot y-\dot y_r).
\end{aligned}
$$


方括号就是 (9) 里的 $e_x$，所以第一行是 $-\omega_r e_x$。把 $\dot x_r$、$\dot y_r$ 移到另一组：


$$
\dot e_y
= -\omega_r e_x
-\sin\theta_r\,\dot x+\cos\theta_r\,\dot y
+\sin\theta_r\,\dot x_r-\cos\theta_r\,\dot y_r.
\tag{11}
$$


现在代入车辆运动学 (1)，$\dot x=v\cos\theta$，$\dot y=v\sin\theta$：


$$
\begin{aligned}
-\sin\theta_r\cdot v\cos\theta+\cos\theta_r\cdot v\sin\theta
&= v\big(\sin\theta\cos\theta_r-\cos\theta\sin\theta_r\big)\\
&= v\sin(\theta-\theta_r)\\
&= v\sin e_\theta.
\end{aligned}
$$


这里用了正弦差角公式。几何含义是：车速在左法向上的分量等于 $v\sin e_\theta$。车头与切线重合时这项为 0，横向误差不因前进本身而增大；车头偏左时 $e_\theta>0$，这项为正，车继续往左离开路径。

参考点若以速率 $v_r$ 沿自身切线运动，则 $\dot x_r=v_r\cos\theta_r$，$\dot y_r=v_r\sin\theta_r$。代入 (11) 的最后两项：


$$
\sin\theta_r\cdot v_r\cos\theta_r-\cos\theta_r\cdot v_r\sin\theta_r=0.
$$


切向运动在法向上的投影为零，参考点自己的平移不直接制造横向误差。于是 (11) 收成


$$
\boxed{\dot e_y=-\omega_r e_x+v\sin e_\theta.}
\tag{12}
$$


对 (10) 求导没有乘积项：


$$
\boxed{\dot e_\theta=\dot\theta-\dot\theta_r=\omega-\omega_r.}
\tag{13}
$$


(12) 和 (13) 是带纵向耦合的误差运动学。$\omega_r e_x$ 的意义是：参考标架一边前进一边旋转时，一段纵向偏差会被旋转带到法向上。弯越急、车在切向上离参考点越远，这一项越大。

#### 3.2.3 为什么循迹模型丢掉 $\omega_r e_x$

任务的调节目标是贴住黑线，也就是把 $e_y$ 和 $e_\theta$ 拉回 0。沿路径快一点或慢一点，只要还投影在线上，就不算丢线。因此控制器的状态取 $(e_y,e_\theta)$，不把 $e_x$ 列进二次型。

丢掉 $-\omega_r e_x$ 之后，


$$
\boxed{
\begin{aligned}
\dot e_y &= v\sin e_\theta,\\
\dot e_\theta &= \omega-\omega_r.
\end{aligned}
}
\tag{14}
$$


这一步可以来自两条不同的几何约定，结果相同，假设不同。

约定 A：参考点按计划速度 $v_r$ 走，$e_x$ 一般不为 0。(12) 里的 $-\omega_r e_x$ 被当作建模时省略的项。省略成立的量级是 $|\omega_r e_x|$ 相对 $|v\sin e_\theta|$ 较小，也就是纵向偏差不大，或者曲率不大。直角弯上 $\omega_r$ 可以到数 rad/s，若 $e_x$ 有几厘米，这一项并不小。第一版没有把 $e_x$ 加回状态。

约定 B：参考点取轴心到路径的垂足。垂足的定义就是位移与切线垂直，因此在每个采样时刻 $e_x=0$。(12) 直接变成 $\dot e_y=v\sin e_\theta$，不需要“小量”论证。把位置写成 $ \mathbf{r}=\mathbf{r}_r(s)+e_y\mathbf{n}(s) $ 再对时间求导，并利用 $\mathrm{d}\mathbf{n}/\mathrm{d}s=-\kappa\mathbf{t}$，得到更完整的正交投影运动学：


$$
\begin{aligned}
\dot s\,(1-\kappa e_y) &= v\cos e_\theta,\\
\dot e_y &= v\sin e_\theta,\\
\dot e_\theta &= \omega-\kappa\dot s.
\end{aligned}
\tag{15}
$$


这里 $\dot e_y=v\sin e_\theta$ 是精确的（在无侧滑、参考点为垂足的前提下），分母 $1-\kappa e_y$ 出现在弧长进度里，不出现在 $\dot e_y$ 里。当 $|\kappa e_y|$ 接近 1 时，进度公式本身会奇异，对应曲率中心落在车轴附近，直角弯附近这个假设很紧。

仿真每一步都把轴心投影到路径上读 $\theta_r$ 和 $\kappa$，几何 $e_y$ 用的是约定 B。MPC 的 $\omega_r$ 却不是 $\kappa\dot s$，而是


$$
\omega_r=\kappa_r v_r,
$$


$v_r$ 由路段类型查表：急弯和直角弯 $0.18\,\mathrm{m/s}$，其余 $0.30\,\mathrm{m/s}$。送进优化器的 $e_y$ 也不是垂足距离，而是 (8) 乘上 $\sigma_x$。所以预测模型是约定 A 的简化式 (14)，测量是红外重心，几何评价用约定 B。三件事在小偏差、传感器仍压在线上时接近；丢线之后红外重心变成 0，几何 $e_y$ 仍在涨，模型和评价就分开了。

(14) 里的 $v\sin e_\theta$ 是非线性的。$|e_\theta|$ 大时，同样的车速把横向误差推开得更快。PID 的输入没有 $e_\theta$，只能等红外重心变大再打差速。MPC 的模型里有这一项，优化器可以通过 $\omega$ 先把 $e_\theta$ 按住。

### 3.3 线性化与离散化

二次规划要求预测是决策变量的仿射函数。(14) 含 $\sin e_\theta$，不能直接进 QP。

#### 3.3.1 在参考点处做一阶泰勒展开

线性化点取参考运动附近：


$$
e_y\approx 0,\qquad e_\theta\approx 0,\qquad v\approx v_r,\qquad \omega\approx\omega_r.
$$


令 $f(v,e_\theta)=v\sin e_\theta$。一元正弦的泰勒展开是


$$
\sin e_\theta=e_\theta-\frac{e_\theta^3}{6}+O(e_\theta^5).
$$


因此


$$
f=v\left(e_\theta-\frac{e_\theta^3}{6}+\cdots\right)
=v_r e_\theta+(v-v_r)e_\theta-v\frac{e_\theta^3}{6}+\cdots.
$$


在 $(v_r,0)$ 处，$f=0$。两个一阶偏导


$$
\frac{\partial f}{\partial e_\theta}=v\cos e_\theta,
\qquad
\frac{\partial f}{\partial v}=\sin e_\theta
$$


在该点的值是 $v_r$ 和 $0$。一阶展开只留下


$$
f(v,e_\theta)\approx v_r e_\theta.
$$


$(v-v_r)e_\theta$ 是两个小量的乘积，$e_\theta^3$ 是三阶项，一阶模型把它们截掉。$\partial f/\partial v$ 在 $e_\theta=0$ 处为 0，所以一阶意义上速度偏差不进入 $\dot e_y$。这就是把 (14) 里的 $v$ 换成 $v_r$、把 $\sin e_\theta$ 换成 $e_\theta$ 的理由：不是“角度看起来不大”这一句口语，而是泰勒公式在该点的一阶项恰好如此。

$|e_\theta|=0.3\,\mathrm{rad}$（约 $17^\circ$）时，$\sin e_\theta\approx 0.296$，相对 $e_\theta$ 的误差约 $1.5\%$，一阶式仍然可用。$|e_\theta|$ 到 $0.8\,\mathrm{rad}$ 时，$\sin e_\theta\approx 0.717$，相对误差超过 $10\%$，线性预测会偏乐观。直角弯丢线时航向误差可以大于这个范围，线性模型和真车会分开。这是模型的适用域，不是求解器的失败。

(14) 的第二式 $\dot e_\theta=\omega-\omega_r$ 已经是线性的。定义决策变量


$$
\Delta u=\omega-\omega_r,
$$


得到线性误差模型


$$
\boxed{\dot e=A_c e+B_c\Delta u}
\tag{16}
$$



$$
\boxed{
A_c=\begin{bmatrix}0& v_r\\ 0& 0\end{bmatrix},
\qquad
B_c=\begin{bmatrix}0\\ 1\end{bmatrix},
\qquad
e=\begin{bmatrix}e_y\\ e_\theta\end{bmatrix}.
}
\tag{17}
$$


$A_c$ 的 $(1,2)$ 元是 $v_r$：车越快，同样的航向误差把横向误差推开得越快，优化器就越不愿意把 $e_\theta$ 留到未来。曲率不出现在 $A_c$ 里。它通过 $\omega_r=\kappa_r v_r$ 进入 $\Delta u$ 的定义：真正加到车上的角速度是 $\omega_r+\Delta u$。当前投影点一旦从直道进入圆弧，$\omega_r$ 在这一拍跳变，指令可以跟着跳。这是当前点的前馈，不是未来曲率序列。

本仓库只优化 $\omega$。$v$ 按路段给定，不进决策变量。若把 $v-v_r$ 也当作输入，$B_c$ 会多一列；在当前线性化点上，该列对 $\dot e_y$ 的一阶系数是 $\sin 0=0$。

#### 3.3.2 前向欧拉离散

计算机每个 $T_s$ 解一次。前向欧拉把导数换成差分：


$$
e_{k+1}=e_k+T_s\dot e_k
=e_k+T_s(A_c e_k+B_c\Delta u_k)
=(I+A_c T_s)e_k+(B_c T_s)\Delta u_k.
$$


若第 $k$ 步的参考速度是 $v_{r,k}$，则


$$
e_{k+1}=A_k e_k+B_k\Delta u_k,
$$



$$
A_k=\begin{bmatrix}1& v_{r,k}T_s\\ 0& 1\end{bmatrix},
\qquad
B_k=\begin{bmatrix}0\\ T_s\end{bmatrix}.
$$


$B_k$ 实际上不随 $k$ 变化，因为 $B_c$ 不含 $v_r$。$A_k$ 随 $v_{r,k}$ 变化。这就是线性时变预测：未来每一拍可以用该拍预瞄点上的 $v_r$ 和 $\omega_r$。

本仓库的第一版不使用时变序列。`freeze_linearization = True`，整个预测时域共用当前拍的 $v_r$，$\omega_r$ 也冻结成当前投影点的值。离散矩阵退回一对常数阵，记为 $A_d,B_d$：


$$
\boxed{
A_d=\begin{bmatrix}1& v_r T_s\\ 0& 1\end{bmatrix},
\qquad
B_d=\begin{bmatrix}0\\ T_s\end{bmatrix},
\qquad
e_{k+1}=A_d e_k+B_d\Delta u_k.
}
\tag{18}
$$


展开就是 `LinearMPC._ab`：


$$
\begin{aligned}
e_{y,k+1}&=e_{y,k}+v_r T_s\, e_{\theta,k},\\
e_{\theta,k+1}&=e_{\theta,k}+T_s\Delta u_k.
\end{aligned}
$$


$v_r=0.30\,\mathrm{m/s}$ 时 $v_r T_s=0.006\,\mathrm{m/rad}$。若 $e_\theta=0.1\,\mathrm{rad}$ 且不纠正，一拍横向误差增加 $0.6\,\mathrm{mm}$；保持 10 拍则约 $6\,\mathrm{mm}$。前向欧拉的局部截断误差是 $O(T_s^2)$。

冻结的后果要写在结果前面：优化问题建立时看不到“0.2 s 之后曲率会跳到 $40\,\mathrm{m}^{-1}$”。急弯入口实测的指令抬起时刻与 $\omega_r$ 同一拍，提前量为 0。这一点放在第 5.2 节，避免和“时变 MPC 可以预瞄”的一般结论混在一起。

### 3.4 滚动优化的目标函数

每个控制周期做三件事：用 (18) 从当前测量往前推 $N$ 步；在约束下最小化一个二次函数；只把第一拍的 $\omega$ 送到 (4)，下一周期全部重来。$N=10$，时域长度 $0.2\,\mathrm{s}$。

记 $\Delta u_{-1}=\Delta u_{\mathrm{prev}}$ 为上一拍真正执行的增量。标准目标函数取


$$
\boxed{
\begin{aligned}
J
&=\sum_{k=0}^{N-1}
\Big(
e_k^\top Q e_k
+R\,\Delta u_k^2
+S(\Delta u_k-\Delta u_{k-1})^2
\Big)
+e_N^\top P e_N.
\end{aligned}
}
\tag{19}
$$


$e_N$ 不重复放进求和里的 $Q$ 项，终端只由 $P$ 惩罚。四项的角色如下。

$e_k^\top Q e_k$ 是跟踪精度。取对角阵 $Q=\mathrm{diag}(q_y,q_\theta)$ 时，


$$
e_k^\top Q e_k=q_y e_{y,k}^2+q_\theta e_{\theta,k}^2.
$$


$q_y$ 直接惩罚离开黑线，$q_\theta$ 惩罚车头歪。$q_\theta$ 不是可有可无的：由 (18)，$e_\theta$ 会在下一拍变成横向误差。$q_\theta$ 太小，优化器可以留下一个缓慢增长的航向偏差；$q_\theta$ 太大，控制器会为了把车头拧正而牺牲横向，直道上更抖。本仓库 $q_y=5000$，$q_\theta=80$，横向权重大于航向。单位必须和状态一起看。$e_y$ 是米，$0.02^2\times 5000=2$。若误用 $q_y=10$，同样 2 cm 的误差只贡献 $0.004$，会被 $R$ 和 $S$ 淹没，车几乎不纠偏。

$R\Delta u_k^2$ 惩罚角速度离开 $\omega_r$。$R$ 大时控制器贴着参考角速度走，弯道前馈强、纠偏弱；$R$ 小时允许大的 $\Delta u$ 去压误差。这里 $R=0.1$。

$S(\Delta u_k-\Delta u_{k-1})^2$ 惩罚增量的变化，也就是惩罚“这一拍猛打、下一拍反向猛打”。PID 没有对应项，它的差分只作用在误差上，不作用在控制量的变化率上。$S=1$。直角弯之后 PID 角速度指令的变化率远大于 MPC，主要就是丢线后误差被置 0、再被噪声打穿，增量式 PID 来回加 $\Delta u$；MPC 的 $S$ 和变化率约束把这种往返压住。直道上两边的变化率 RMS 相同，不能把 $S$ 理解成“全线都比 PID 更安静”。

$e_N^\top P e_N$ 是终端代价。没有它，优化器可以把误差推到第 $N$ 步之外，时域内的代价看起来很小。$P=\mathrm{diag}(10000,200)$，量级约为 $2Q$。这是对角经验阵，没有把 $P$ 取成无限时域代价的解，也没有证明终端集不变。文档和代码都把它标成未做稳定性证明的参数。

$e_0^\top Q e_0$ 不含本拍的决策变量。写进 (19) 不改变最优的 $\Delta u$ 序列，只给代价一个绝对数值。cvxpy 形式里保留了这一项。

软约束可以写成 $|e_{y,k}|\le e_{y,\max}+\varepsilon_k$，$\varepsilon_k\ge 0$，再在 $J$ 里加 $M\varepsilon_k$。$e_{y,\max}=2\sigma_x\approx 0.0343\,\mathrm{m}$，对应 PID 的 $|e_{\mathrm{sensor}}|\ge 2$。默认 $M=10^4$，但 `use_soft_ey = False`，这组约束不进求解。图上仍画出 $\pm e_{y,\max}$。

硬约束来自电机，而不是来自手调的舒适区：


$$
\boxed{\omega_{\min}\le \omega_r+\Delta u_k\le \omega_{\max}}
\tag{20}
$$



$$
\omega_{\min}=-4.78125\,\mathrm{rad/s},\qquad \omega_{\max}=4.78125\,\mathrm{rad/s},
$$


即 (7)。变化率约束作用在 $\omega$ 上：


$$
\boxed{|\omega_k-\omega_{k-1}|\le \Delta\omega_{\max}=2.390625\,\mathrm{rad/s}.}
\tag{21}
$$


时域内 $\omega_r$ 被冻结时，$k\ge 1$ 等价于 $|\Delta u_k-\Delta u_{k-1}|\le\Delta\omega_{\max}$。第 0 拍与上一拍真正执行的 $\omega_{\mathrm{prev}}$ 比较：


$$
|(\omega_r+\Delta u_0)-\omega_{\mathrm{prev}}|\le\Delta\omega_{\max}.
$$


直角弯上 $\kappa_r=40\,\mathrm{m}^{-1}$，$v_r=0.18\,\mathrm{m/s}$，


$$
\omega_r=7.2\,\mathrm{rad/s}>4.78125\,\mathrm{rad/s}.
$$


(20) 在这一拍就不可行：不存在 $\Delta u$ 使 $\omega$ 既等于所需的 $7.2$、又落在 $\pm 4.78$ 里。求解器会把 $\omega$ 顶在上限上。跟踪误差在这里是约束不可行，再加大 $q_y$ 也变不出更小的转弯半径。(3) 还说明，即便放宽 (20)，不能倒车的差速机构也走不出半径 $2.5\,\mathrm{cm}$ 的圆弧。

求解失败时的后备是


$$
\Delta u_0=\mathrm{clip}\!\left(\frac{-2e_y}{\max(v_r,10^{-3})},\,-\Delta\omega_{\max},\,\Delta\omega_{\max}\right),
$$


再把 $\omega_r+\Delta u_0$ 夹进 (20)。$e_y>0$（车在左侧）时后备 $\Delta u_0<0$，车向右转。`self_check_mpc_direction` 用 $e_y=0.02\,\mathrm{m}$、$e_\theta=0$、$v_r=0.30$、$\omega_r=0$ 检查最优的 $\Delta u_0$ 为负。

滚动的最后一步：QP 给出 $\Delta u_0^\star,\ldots,\Delta u_{N-1}^\star$ 之后，只执行


$$
\omega_{\mathrm{cmd}}=\mathrm{clip}(\omega_r+\Delta u_0^\star,\ \omega_{\min},\ \omega_{\max}),
$$


把 $\Delta u_0=\omega_{\mathrm{cmd}}-\omega_r$ 和 $\omega_{\mathrm{cmd}}$ 存成下一拍的 $\Delta u_{\mathrm{prev}}$、$\omega_{\mathrm{prev}}$。然后重新测量、重新投影、重新线性化、重新求解。

### 3.5 把 $N$ 步递推写成一次矩阵乘法

(18) 是递推。QP 需要 $e_1,\ldots,e_N$ 一次性表示成 $\Delta u_0,\ldots,\Delta u_{N-1}$ 的仿射函数。冻结 $A_d,B_d$ 时可以逐项代入。为了缩短记号，下面把 $A_d,B_d$ 写成 $A,B$。


$$
\begin{aligned}
e_1&=Ae_0+B\Delta u_0,\\
e_2&=Ae_1+B\Delta u_1
=A^2e_0+AB\Delta u_0+B\Delta u_1,\\
e_3&=A^3e_0+A^2B\Delta u_0+AB\Delta u_1+B\Delta u_2.
\end{aligned}
$$


一般项是


$$
\boxed{
e_k=A^k e_0+\sum_{j=0}^{k-1}A^{k-1-j}B\,\Delta u_j,
\qquad k=1,\ldots,N.
}
\tag{22}
$$


把状态按时间堆起来，


$$
E=\begin{bmatrix}e_1\\ e_2\\ \vdots\\ e_N\end{bmatrix}\in\mathbb{R}^{2N},
\qquad
U=\begin{bmatrix}\Delta u_0\\ \Delta u_1\\ \vdots\\ \Delta u_{N-1}\end{bmatrix}\in\mathbb{R}^{N}.
$$


(22) 写成


$$
\boxed{E=\Phi e_0+\Gamma U.}
\tag{23}
$$


$\Phi$ 是 $2N\times 2$。它的第 $k$ 个 $2\times 2$ 块是 $A^k$，$k$ 从 1 到 $N$。没有任何控制输入时，自由响应就是 $\Phi e_0$。

$\Gamma$ 是 $2N\times N$。第 $k$ 个行块、第 $j$ 列（$j$ 从 0 数起）在 $j<k$ 时等于 $A^{k-1-j}B$，否则为 0。每一列是“只在第 $j$ 拍打一个单位 $\Delta u$、其余为 0”所激起的状态脉冲响应。因为系统是因果的，第 $j$ 拍的输入不影响 $e_1,\ldots,e_j$。

$N=2$ 时可以写全：


$$
\Phi=\begin{bmatrix}A\\ A^2\end{bmatrix}
=\begin{bmatrix}
1& v_r T_s\\
0& 1\\[4pt]
1& 2v_r T_s\\
0& 1
\end{bmatrix},
\qquad
\Gamma=\begin{bmatrix}
B& 0\\
AB& B
\end{bmatrix}
=\begin{bmatrix}
0& 0\\
T_s& 0\\[4pt]
v_r T_s^2& 0\\
T_s& T_s
\end{bmatrix}.
$$


这里用了 $A^2=\begin{bmatrix}1& 2v_r T_s\\ 0& 1\end{bmatrix}$ 和 $AB=\begin{bmatrix}v_r T_s^2\\ T_s\end{bmatrix}$。$N=10$ 时规律不变，只是块数增加。代码 `_prediction_maps` 先累乘得到 $I,A,A^2,\ldots,A^N$，再按上面的下标填 $\Gamma$。

若将来改成时变模型，(22) 里的 $A^k$ 要换成有序乘积 $A_{k-1}\cdots A_0$，$\Gamma$ 的每一列换成从施加时刻传到当前时刻的状态转移阵乘当时的 $B_j$。冻结线性化就是把所有这些 $A_k$ 设成同一个 $A_d$。

把 (23) 代入只含 $e_1,\ldots,e_N$ 的二次项。令 $\bar Q$ 为 $2N\times 2N$ 的块对角阵，前 $N-1$ 块等于 $Q$，最后一块等于 $P$（与“阶段用 $Q$、终端用 $P$”一致；$e_0$ 的 $Q$ 是常数，不进 $\bar Q$）。则


$$
E^\top\bar Q E
=(\Phi e_0+\Gamma U)^\top\bar Q(\Phi e_0+\Gamma U).
$$


展开后与 $U$ 有关的部分是


$$
U^\top(\Gamma^\top\bar Q\Gamma)\,U+2(\Phi e_0)^\top\bar Q\Gamma\, U.
$$


再加上 $U$ 自己的二次型 $U^\top H_u U$ 以及与 $\Delta u_{\mathrm{prev}}$ 的线性交叉项，就得到标准二次规划


$$
\min_U\ \frac12 U^\top P_{\mathrm{qp}} U+q^\top U
\quad\text{s.t.}\quad
l\le A_{\mathrm{con}}U\le u.
\tag{24}
$$


OSQP 的目标是 $\tfrac12 U^\top P_{\mathrm{qp}}U+q^\top U$。令 $H=\Gamma^\top\bar Q\Gamma+H_u$，则 $P_{\mathrm{qp}}=2H$，$q=2\Gamma^\top\bar Q\Phi e_0$，再把 $q$ 的第 0 个分量减去 $2S\Delta u_{\mathrm{prev}}$，用来产生 $-2S\Delta u_{\mathrm{prev}}\Delta u_0$。约束 (20)、(21) 写成 $U$ 的线性不等式，填进 $l,u,A_{\mathrm{con}}$。这一步之后，$N$ 步递推不再在求解器内部滚动，求解器看到的是一个稠密 QP。

cvxpy 路径不手写 $\Phi,\Gamma$。它为 $e_0,\ldots,e_N$ 和 $\Delta u_0,\ldots,\Delta u_{N-1}$ 建变量，把 (18) 写成等式约束，目标直接写 (19) 的和式，由求解器自己消元。两条路径的 $A,B$、$Q,R,S,P$ 和 (20)、(21) 相同。控制增量二次型的具体矩阵在第 4.3 节对照。

## 4. 仿真参数与实现细节

### 4.1 PID

| 参数 | 值 | 作用 |
| --- | --- | --- |
| $K_p,K_i,K_d$ | 33，0，10 | 增量式 PID，$K_i$ 保持为 0 |
| $u$ 限幅 | $[-150,150]$ | 累加之后裁剪 |
| 红外阈值 | 250 | 低于此值的权重为 0 |
| 急弯：模拟量 / 路数 / $\|e\|$ | 1000 / 3 / 2 | 满足其一即进入急弯分支 |
| 急弯放大 | $\times 1.5$ | 只作用于这一拍的电机指令 |
| 基础 PWM | 100，急弯时 60 | 对应计划速度 0.30 与 0.18 m/s |
| PWM | $[0,255]$ | `int` 向 0 截断 |

从位置式减去上一拍，可以得到增量式，便于核对代码。位置式若写成


$$
u_k^{\mathrm{pos}}=K_p e_k+K_i\sum_{j=0}^{k}e_j+K_d(e_k-e_{k-1}),
$$


则


$$
u_k^{\mathrm{pos}}-u_{k-1}^{\mathrm{pos}}
=K_p(e_k-e_{k-1})+K_i e_k+K_d(e_k-2e_{k-1}+e_{k-2}).
$$


这就是 `IncrementalPID.update` 的 $\Delta u$。饱和加在累加之后：下一拍从被削过的 $u_k$ 继续加，而不是从无饱和的位置式继续加。$K_i=0$，没有积分器可风起。

手算：误差从 0 跳到 1 并保持。第 1 拍 $\Delta u=33+10=43$。第 2 拍 $\Delta u=10(1-2)=-10$，$u=33$。第 3 拍二阶差分为 0，$u$ 停在 33，等于 $K_p e$。常值误差不会被积到 0。

### 4.2 MPC

| 参数 | 值 |
| --- | --- |
| $N$ | 10 |
| $Q$ | $\mathrm{diag}(5000,\ 80)$ |
| $R$ | 0.1 |
| $S$ | 1.0 |
| $P$ | $\mathrm{diag}(10000,\ 200)$ |
| $\omega$ | $[-4.78125,\ 4.78125]\,\mathrm{rad/s}$ |
| $\Delta\omega_{\max}$ | $2.390625\,\mathrm{rad/s}$ |
| $e_{y,\max}$ | $0.034286\,\mathrm{m}$（图上标注；软约束默认关闭） |
| $v_r$ | 直道、缓弯、S 弯 0.30 m/s；急弯、直角弯 0.18 m/s |
| $e_\theta$ 噪声 | $N(0,0.01^2)\,\mathrm{rad}$，加在真值上 |
| 线性化 | 时域内冻结为当前拍 |

$v_r$ 的两个数就是 PID 基础 PWM 100 和 60 经过 (5) 的稳态轮速。这样速度切换的规则与 PID 的几何分段对齐：PID 在误差已经大了之后才降速，MPC 在投影进入急弯或直角弯时就使用低速参考。急弯上 MPC 更慢，是对比时必须分开的因素，见第 5.1 节。

### 4.3 求解器

安装了 `osqp` 和 `scipy` 时，默认后端是原生 OSQP 稠密 QP（`osqp_native`），不经过 cvxpy。没有原生绑定、但有 cvxpy 时，用 cvxpy，并优先选择它的 OSQP 接口。再没有则用 cvxpy 的默认求解器。最后退到 `scipy.optimize.minimize` 的 SLSQP。种子 42 的这次运行是 `osqp_native`，单步平均 $0.18\,\mathrm{ms}$，最大 $3.26\,\mathrm{ms}$，低于 $T_s=20\,\mathrm{ms}$。

cvxpy 问题按 (19) 的和式写，等式约束是 (18)。原生 OSQP 走 (24)。它的控制 Hessian $H_u$ 由 `_hu` 填出。$N=10$、$R=0.1$、$S=1$ 时对角为


$$
(2.1,\ 3.1,\ 3.1,\ \ldots,\ 3.1,\ 2.1)
=(R+2S,\ R+3S,\ \ldots,\ R+3S,\ R+2S),
$$


相邻副对角为 $-S$。把 (19) 里的 $R\sum\Delta u_k^2+S\sum(\Delta u_k-\Delta u_{k-1})^2$ 展开，对角应是 $(R+2S,\ R+2S,\ \ldots,\ R+2S,\ R+S)$。从第二个控制量起，`_hu` 比这份和式多一份 $S$。默认数值结果以 `_hu` 为准。用 (19) 重新实现时，不要期望与 `osqp_native` 逐位相同。

### 4.4 一个控制周期的公共顺序

1. 把轴心投影到路径，得到弧长、几何 $e_y$、$\theta_r$、$\kappa_r$。
2. 计算 8 路传感器位置和到线距离，抽取模拟量噪声，用 (8) 得到 $e_{\mathrm{sensor}}$。
3. PID：把 $e_{\mathrm{sensor}}$ 送进增量式 PID，再走急弯分支。MPC：$e_y^{\mathrm{meas}}=\sigma_x e_{\mathrm{sensor}}$，$e_\theta^{\mathrm{meas}}=\mathrm{wrap}(\theta-\theta_r)+n_\theta$，按路段取 $v_r$，$\omega_r=\kappa_r v_r$，解 (24)，用 (4) 得到 PWM。
4. PWM 经 (5)、(6) 变成轮速，再经 (2) 得到 $v,\omega$。
5. 用 RK4 积分 (1) 一个周期。
6. 轴心离路径超过 $0.55\,\mathrm{m}$ 并持续 $1.6\,\mathrm{s}$ 则停止。否则直到接近终点且到线距离小于 $8\,\mathrm{cm}$，或时间超过按 $0.20\,\mathrm{m/s}$ 估计的上限。

### 4.5 参考路径

路径由直线和圆弧串成，曲率左转为正。直角弯半径以段表为准，是 $0.025\,\mathrm{m}$（$\kappa=40\,\mathrm{m}^{-1}$）。`path.py` 文件头有一句过时注释写着 $0.05\,\mathrm{m}$，仿真不使用那一句。

| 名称 | 半径 | 转角 | 曲率 |
| --- | ---: | --- | ---: |
| 直道1，1.20 m | $\infty$ | 0 | 0 |
| 缓弯 | 0.80 m | 左转 $90^\circ$ | $1.25\,\mathrm{m}^{-1}$ |
| 直道2，0.70 m | $\infty$ | 0 | 0 |
| S 弯三段 | 0.60 m | 右 $45^\circ$、左 $90^\circ$、右 $45^\circ$ | $\mp 1.667\,\mathrm{m}^{-1}$ |
| 直道3，0.55 m | $\infty$ | 0 | 0 |
| 急弯 | 0.25 m | 左转 $90^\circ$ | $4\,\mathrm{m}^{-1}$ |
| 直道4，0.50 m | $\infty$ | 0 | 0 |
| 直角弯 | 0.025 m | 左转 $90^\circ$ | $40\,\mathrm{m}^{-1}$ |
| 直道5，2.90 m | $\infty$ | 0 | 0 |

总长 $9.424\,\mathrm{m}$。S 弯三段转角之和为 0。直道5 用来看丢线之后还有没有距离可回正。初始时刻车在路径左侧 $8\,\mathrm{mm}$，航向误差为 0。

## 5. 对比结果

数字来自种子 42 的一次 `run_all.py`。几何 $e_y$ 是轴心垂足的左法向分量，单位在表中已换成容易读的厘米或米。PID 跑了 $26.08\,\mathrm{s}$（1305 步）后因离线结束。MPC 跑了 $32.08\,\mathrm{s}$（1605 步）并走完直道5。

传感器丢线若只累计连续不少于 3 拍的事件，PID 为 18 次、共 $3.22\,\mathrm{s}$，MPC 为 3 次、共 $0.58\,\mathrm{s}$。`pid_metrics.md` 把每一拍 $\sum w=0$ 都加进时长，同一段 PID 轨迹是 176 拍、$3.52\,\mathrm{s}$，事件次数仍是 18。控制律相同，差在统计窗口。

| 指标 | PID | MPC |
| --- | ---: | ---: |
| 直道/缓弯 RMS $e_y$ | 2.2 mm | 3.0 mm |
| S 弯 RMS $e_y$ | 3.4 mm | 2.7 mm |
| 急弯最大 $\|e_y\|$ | 8.6 mm | 5.5 mm |
| 急弯 RMS $e_y$ | 5.3 mm | 2.7 mm |
| 急弯平均速度 | 0.297 m/s | 0.193 m/s |
| 直角弯最大 $\|e_y\|$ | 0.889 m | 0.094 m |
| 直角弯 RMS $e_y$ | 0.489 m | 0.055 m |
| 全程 $\omega_{\mathrm{cmd}}$ 变化率 RMS | 43.66 rad/s² | 7.74 rad/s² |
| 直道/缓弯变化率 RMS | 4.74 rad/s² | 4.74 rad/s² |
| 急弯段变化率 RMS | 4.86 rad/s² | 5.94 rad/s² |
| 急弯+直角弯变化率 RMS | 91.96 rad/s² | 14.87 rad/s² |
| 几何丢线（$\|e_y\|>6\,\mathrm{cm}$ 连续 ≥ 3 拍） | 1 次，3.84 s | 1 次，1.08 s |
| 末段回到 0.5 cm 以内 | 未回正 | 2.74 s |

图：`compare_trajectory.png`（全程与弯道放大）、`compare_error.png`、`compare_omega.png`、`compare_preview.png`，以及三张 `compare_*_annotated.png`。

### 5.1 轨迹精度

急弯（半径 $0.25\,\mathrm{m}$）上，几何误差的峰值从 $0.86\,\mathrm{cm}$ 降到 $0.55\,\mathrm{cm}$，


$$
\frac{0.86-0.55}{0.86}=36.2\%.
$$


同一段的 RMS 从 $5.3\,\mathrm{mm}$ 降到 $2.7\,\mathrm{mm}$。这段曲线上两边都还在 $\pm 1\,\mathrm{cm}$ 内，没有丢线。不能把 $36\%$ 全部记成反馈律的功劳：MPC 在投影进入急弯时就把 $v_r$ 设为 $0.18\,\mathrm{m/s}$，实测平均速度 $0.193\,\mathrm{m/s}$；PID 多数时间基础 PWM 仍是 100，平均速度 $0.297\,\mathrm{m/s}$。同样的航向误差，(14) 里的 $v\sin e_\theta$ 在高速下把横向误差推得更快。急弯变好，是“低速参考 + $\omega_r$ 前馈 + 误差状态反馈”合在一起的结果。

直道和缓弯上 PID 的 RMS 反而小 $0.8\,\mathrm{mm}$（2.2 mm 对 3.0 mm）。两边都在毫米级，小于线宽和红外噪声的等效尺度。S 弯上 MPC 的 RMS 小 $0.7\,\mathrm{mm}$。这些差距说明 MPC 不是在所有路段都更贴线。

直角弯之后差距才成为分米级。PID 的几何误差走到 $0.889\,\mathrm{m}$ 后仿真结束，末段直道没有回到 1 cm。MPC 偏离到 $0.094\,\mathrm{m}$，穿过 6 cm 的几何丢线线，然后在直道5上拉回：从最后一次仍大于 2 cm，到第一次小于 0.5 cm，用了 $2.74\,\mathrm{s}$，结束时约 $-0.24\,\mathrm{mm}$。两车都会离开这条半径 $2.5\,\mathrm{cm}$ 的圆弧，因为 (3) 和 (7) 都不允许贴着圆心走。差别是丢线之后 PID 把误差清零并停在 $u\approx 0$，MPC 在红外短暂恢复或几何上仍能形成 $e_\theta$ 信息时还能打方向。MPC 的 $e_y^{\mathrm{meas}}$ 在完全丢线时同样是 0，所以它也不是“看见了 9 cm 的偏差再拉回来”；它能回来，是因为偏离过程中航向通道和尚未饱和的红外读数还在，并且低速参考留出了直线距离。

全程 RMS 从 $0.203\,\mathrm{m}$ 降到 $0.0176\,\mathrm{m}$，主要被直角弯之后那段 PID 离线样本拉开，不代表直道跟踪精度差了一个数量级。

### 5.2 预瞄：实测提前量是 0

标注图用的定义是：急弯入口 $|\kappa|$ 进入 $4\,\mathrm{m}^{-1}$ 这一档时，$\omega_r$ 开始上升的时刻为 $t_1$；在此之前，$|\omega_{\mathrm{cmd}}|$ 最后一次低于 $0.40\,\mathrm{rad/s}$ 的下一拍为 $t_2$。阈值 $0.40$ 高于直道上大约 $\pm 0.2\,\mathrm{rad/s}$ 的起伏。这次运行


$$
t_1=t_2=18.74\,\mathrm{s},
\qquad
t_1-t_2=0.00\,\mathrm{s}.
$$


`compare_preview.png` 里，急弯入口的 $\omega_{\mathrm{cmd}}$ 和 $\omega_r$ 一起抬起。更严的 $0.5\,\mathrm{rad/s}$ 判据（阶跃前 $0.55\,\mathrm{s}$ 内、同向、且要保持到阶跃前一拍）给出同一个结论：MPC 在急弯入口没有提前抬起。

这与“时变 MPC 会提前打角”不矛盾，因为本实现不是时变的。第 3.3.2 节已经写明：预测时域内的 $A$、$B$ 和 $\omega_r$ 都冻结在当前投影点。前方曲率没有进入 (23)。当前点的 $\omega_r=\kappa_r v_r$ 仍是前馈，但它和轴心投影进入弯道同一拍才跳。跳之前模型认为自己还在直道上。

传感器前视约 $0.25\,\mathrm{s}$ 会让红外重心在轴心进弯之前变化。这是第 3.1.4 节的几何，PID 同样拥有。用 $0.5\,\mathrm{rad/s}$ 的判据，PID 在急弯入口提前约 $120\,\mathrm{ms}$ 并保持到阶跃前，在直角弯入口提前约 $260\,\mathrm{ms}$。这两段时间和 $\ell/v$ 同一量级，来自横杆已经压到弯上之后的反馈，加上急弯分支把 $u$ 乘 $1.5$。把它叫成“PID 有模型预瞄”是错的。

直角弯入口上，MPC 曾在阶跃前约 $200\,\mathrm{ms}$ 把同向指令抬到约 $1\,\mathrm{rad/s}$，但阶跃前一拍已经回到负值，按“必须保持到阶跃前”的定义不算完成的预打角。随后 $\omega_{\mathrm{cmd}}$ 与 $\omega_r$ 一起打到约 $4.78\,\mathrm{rad/s}$ 的上限，而几何上需要的 $\omega_r$ 是 $7.2\,\mathrm{rad/s}$，超出约束，也超出预瞄图的纵轴。

若要让提前量来自模型而不是来自车头那 $7.5\,\mathrm{cm}$，需要在构造 $\Phi,\Gamma$ 时使用未来弧长上的 $\kappa_{r,k}$ 和 $v_{r,k}$，也就是关掉冻结、改用第 3.5 节末尾说的时变乘积。那是下一版的改动，不是这一版图上的 $0.00\,\mathrm{s}$。

### 5.3 控制平顺性

全程 $\omega_{\mathrm{cmd}}$ 变化率 RMS 从 $43.66\,\mathrm{rad/s}^2$ 降到 $7.74\,\mathrm{rad/s}^2$。这个全程数字容易误读成“MPC 在所有路段都更平滑”。分段之后不是这样。

直道和缓弯上，两边都是 $4.74\,\mathrm{rad/s}^2$。红外噪声进 (8)，PID 的二阶差分和 MPC 的 $e_\theta$ 噪声都会让 PWM 抖动，幅度在这一段看不出差别。急弯段上 MPC 略高：$5.94$ 对 $4.86\,\mathrm{rad/s}^2$。$S$ 没有把这段振掉。

大的差别在直角弯之后。急弯加直角弯窗口里，PID 为 $91.96\,\mathrm{rad/s}^2$，MPC 为 $14.87\,\mathrm{rad/s}^2$。`compare_omega.png` 的下半张把纵轴放大到 $\pm 2\,\mathrm{rad/s}$：直道上红线和蓝线的起伏幅度接近；直角弯之后红线往返打到很大，蓝线打到上限、停住，再收回。PID 的往返对应第 1.3 节：丢线把 $e_{\mathrm{sensor}}$ 置 0，噪声又偶尔让某一路超过 250，增量式 PID 在 0 和某个残差之间来回加 $\Delta u$，没有 $S$，也没有 (21)。MPC 的 $S$、(21) 和角速度上限把指令留在一个方向上，直到几何误差和航向误差一起缩小。

因此，“消除 PID 震荡”应限定为：直角弯丢线之后的大幅度往返被压低了。直道上的小抖动还在，急弯段的变化率没有下降。

## 6. 工程反思与局限性

仿真回答的是：在红外、轮速滞后和这条路径上都对 PID 公平的前提下，误差状态上的线性 MPC 能否少离线、并在离线后回来。它能，而且直角弯上的差别是定性的。把它装回现在这块板子之前，有三道缺口。缺任何一道，第 3 节的链条就在实物上断掉。

### 6.1 航向误差在车上测不到

(10) 需要 $\theta$ 和 $\theta_r$。仿真里 $\theta$ 是积分 (1) 得到的真值，再加 $N(0,0.01^2)$。8 路红外的重心主要携带横向信息，在直道上对小航向角不敏感，也给不出世界系的 $\theta$。$\theta_r$ 还要求知道车投影到了路径的哪一点。

实物上 $e_\theta$ 需要额外的航向测量或估计：IMU 的航向角，再减去由定位或视觉给出的路径切线；或者用短时间内横向误差的变化反推 $\sin e_\theta\approx \dot e_y/v$，这又要求 $e_y$ 和 $v$ 本身可信。没有这一路，MPC 退化为只用 $e_y^{\mathrm{meas}}$ 的单状态预测，(18) 的第二行没有测量可以校正，滚动时域里的航向会漂。PID 今天能跑，正是因为它不需要 $e_\theta$。

### 6.2 当前实现也还没有“看见前方的弯”

第 5.2 节的提前量是 0，原因在算法而不在偶然的随机种子。冻结线性化时，曲率只在轴心的投影点进入新段的那一拍进入 $\omega_r$。红外前视 $7.5\,\mathrm{cm}$ 是两边共有的，不是 MPC 的地图。

实物若希望模型在进弯之前就把 $\omega_r$ 抬起来，预测里必须出现未来的 $\kappa_{r,k}$。黑胶带没有事先的矢量地图时，这要求车头看见更远的线：摄像头、或者更长的红外阵列，再把未来若干段的曲率送进时变的 $A_k$ 和 $\omega_{r,k}$。只把求解器换上、仍只喂当前 8 路的重心，得到的是带约束和平滑项的反馈加“当前点前馈”，不是路段预瞄。

另外，红外在完全离开黑线时把 $e_y^{\mathrm{meas}}$ 置 0。MPC 和 PID 在这一拍都会收到“误差消失了”的测量。仿真里 MPC 能回到直道5，不能外推成“实物丢线后 MPC 一定找得回线”。找线需要丢线检测之后的另一套行为，例如沿上次的 $e_\theta$ 继续转、或主动搜索，而不是把测量 0 送进 (19)。

### 6.3 算力与实时性

这次在 PC 上，$N=10$、软约束关闭、原生 OSQP 的平均求解时间是 $0.18\,\mathrm{ms}$，最大 $3.26\,\mathrm{ms}$。相对 $20\,\mathrm{ms}$ 的仿真周期还有余量。这个数字不是板上的周期。

现有固件跑在接收红外串口包的 Arduino 类控制器上，控制律是十几次乘加和几次比较。板上没有 OSQP，也没有把 cvxpy 问题实时编译的环境。把 (24) 原样放进当前芯片，周期里装不下稠密 QP 的分解，串口包到达时会错过下一帧红外。

落到车上的路线因此不是“把 `controllers/mpc.py` 拷进固件”。可行的降阶包括：在 PC 或协处理器上解 QP，主控只收 $\omega_{\mathrm{cmd}}$；把工作区域内的 $(e_y,e_\theta,\kappa)$ 预先打成表，板上插值，表的分辨率和边界要单独验证；或者把 (18) 收成一个低维显式反馈，只在饱和时切换。无论哪一条，都要重新测量第 5 节的急弯峰值、直角弯回正和直道抖动，因为显式律和查表都会再引入近似。$P$ 也还没有终端不变集的证明，增益不能靠“仿真没散”推广到所有初始条件。

机构限制不会被求解器去掉。半径 $2.5\,\mathrm{cm}$ 的弯小于 $L/2=8\,\mathrm{cm}$。真车上若必须过这种折角，要么允许一侧轮子反转，要么接受切弯，要么把胶带铺成大于 $8\,\mathrm{cm}$ 的圆角。仿真里 MPC 的价值是切弯之后还能在长直道上收回来；PID 在同样的测量归零规则下收不回来。

## 许可

MIT License，见 `LICENSE`。
