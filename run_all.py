# -*- coding: utf-8 -*-
"""一键跑通：PID 仿真 → MPC 仿真 → 对比图与 PPT 标注图。

在项目根目录执行::

    py -3 -m pip install -r requirements.txt
    py -3 run_all.py
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main():
    print("========== 1/4  PID 仿真 ==========", flush=True)
    from simulators.pid_sim import main as pid_main

    pid_main()

    print("========== 2/4  MPC 仿真 ==========", flush=True)
    from simulators.mpc_sim import main as mpc_main

    mpc_main()

    print("========== 3/4  对比图与指标 ==========", flush=True)
    from comparison.compare import main as compare_main

    compare_main()

    print("========== 4/4  PPT 标注图 ==========", flush=True)
    from comparison.generate_plots import main as plot_main

    plot_main()

    print("========== 全流程完成 ==========", flush=True)
    print(f"数据：{ROOT / 'results' / 'data'}", flush=True)
    print(f"图片：{ROOT / 'results' / 'figures'}", flush=True)


if __name__ == "__main__":
    main()
