from __future__ import annotations

"""
tests/test_case_cold_start.py

冷启动测试：
- 不施加持续外扰（disturbance = 0）
- 固定垃圾组分
- 将焚烧炉初始状态设为低温/低速，观察系统是否能回到稳态工作点

使用前提
--------
本脚本假定 tests/sim_common.py 中的 SimConfig 与 run_case 已存在，并且：
1) run_case(...) 会读取 cfg 中的冷启动初值字段（如果有）
2) 若当前 sim_common.py 还没有这些字段，请按下方“需要的最小补丁”添加

推荐总时长：
- 6000 s 起步
- 若要看更充分收敛，可到 12000 s

运行：
python tests/test_case_cold_start.py
"""

from tests.sim_common import SimConfig, plot_history, run_case


# -----------------------------
# 1) 固定来料：用一组中间偏常见的组分
#    你后续可以按需要换成别的配方
# -----------------------------
def composition_schedule(t: float) -> list[float]:
    # [菜叶, 西瓜皮, 橙子皮, 肉, 杂项混合, 米饭]
    return [0.20, 0.15, 0.15, 0.10, 0.20, 0.20]


# -----------------------------
# 2) 冷启动：不加外扰
#    关键是“低初值”，不是持续负偏置
# -----------------------------
def disturbance_schedule(t: float) -> float:
    return 0.0


def main() -> None:
    cfg = SimConfig(
        total_time_s=6000.0,

        # 仍沿用你当前项目里常用的时间尺度
        dt_meas_s=0.1,
        dt_ctrl_s=1.0,
        dt_opt_s=2.0,

        # ---------------------------------------------
        # 关键：冷启动初值
        # 这些字段如果你的 SimConfig 还没有，需要给它加进去
        # ---------------------------------------------
        furnace_init_mode="cold",
        T_avg_init_C=20.0,
        T_stack_init_C=20.0,
        v_stack_init_mps=0.1,

        # 可选：若你希望预热炉也从“更冷/未建立”状态起步，可先用较保守初值
        # 如果 sim_common 里没有这些字段，可以先忽略
        preheater_init_mode="warm",
        # actual omega_out 初值，先给一组接近常温未充分预热的值
        omega_out_init=0.60,

        case_name="cold_start",
    )

    hist = run_case(
        "cold_start",
        composition_schedule,
        disturbance_schedule,
        cfg,
    )

    out_png = "tests/cold_start.png"
    plot_history(
        hist,
        out_png,
        title="Closed loop test: cold start without external disturbance",
    )
    print(f"Saved plot to {out_png}")


if __name__ == "__main__":
    main()


# ======================================================================
# 如果你当前 tests/sim_common.py 还不支持“冷启动初值”，请加下面这两个最小补丁
# ======================================================================
#
# [补丁 A] 在 SimConfig 里增加字段（若不存在）
#
# furnace_init_mode: str = "warm"   # "warm" or "cold"
# T_avg_init_C: float = 872.99
# T_stack_init_C: float = 960.0
# v_stack_init_mps: float = 18.0
# preheater_init_mode: str = "warm"
# omega_out_init: float = 0.32
#
#
# [补丁 B] 在 FurnaceDyn.initialize(...) 或 run_case(...) 里根据 cfg 设置初值
#
# if cfg.furnace_init_mode == "cold":
#     furnace.initialize(
#         T_avg_init_C=cfg.T_avg_init_C,
#         T_stack_init_C=cfg.T_stack_init_C,
#         v_stack_init_mps=cfg.v_stack_init_mps,
#     )
# else:
#     furnace.initialize_near_reference(...)
#
# 预热炉如果暂时不想做真正冷启动，可先仍用 warm 模式；
# 这样测试的重点就是：焚烧炉从低温初态出发，控制链是否最终能把它拉回稳态。
#
