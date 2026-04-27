from __future__ import annotations

"""
test_optimizer.py

简单测试脚本：
1. 将若干组 6 类垃圾组分翻译为等效性质
2. 向 optimizer.py 发送静态优化请求
3. 检查输出是否满足基本预期
4. 生成一份示例离线控制表

运行
----
python test_optimizer.py
"""

from pathlib import Path

from cleanser.cleanser import composition_to_equivalent_properties, print_cleansed_result
from optimizer.optimizer import (
    Config,
    DynamicTargetBand,
    OptimizerRequest,
    ResourceBoundary,
    build_lookup_table,
    optimize_static_slot,
)


def assert_basic_physical_consistency(result) -> None:
    tol = 1e-2
    assert result.Tg_star_C >= 100.0, "Tg 低于设备下限"
    assert 3.0 <= result.vg_star_mps <= 12.0, "vg 超出设备边界"
    assert result.Tm_star_C <= 250.0 + 1e-6, "Tm 超出上限"
    assert result.tau_target_min <= result.tau_r_min + tol, "停留时间约束未满足"
    assert result.Qsup_kW >= result.Qreq_kW - tol, "供热约束未满足"
    assert result.mdot_preheater_kgps <= result.mdot_stack_cap_kgps + tol, "烟气质量流量约束未满足"
    assert result.max_constraint_violation <= tol, "整体约束违背过大"



def main() -> None:
    cfg = Config()

    # -----------------------------------------------------
    # 场景 1：典型工况，严格稳态燃烧约束
    # -----------------------------------------------------
    comp1 = [0.20, 0.15, 0.15, 0.10, 0.20, 0.20]
    cleansed1 = composition_to_equivalent_properties(comp1)
    print_cleansed_result(cleansed1)

    req1 = OptimizerRequest(
        slot_id="strict_typical",
        props=cleansed1.equivalent,
        omega_tar=0.320,
        resource=ResourceBoundary(T_stack_cap_C=930.0, v_stack_cap_mps=18.0),
        dyn_band=DynamicTargetBand(omega_min=0.300, omega_max=0.340),
        burn_policy="strict",
    )
    res1 = optimize_static_slot(req1, cfg)
    assert res1.success, "场景 1 未得到数值可行解"
    assert res1.feasible_under_strict_burn, "场景 1 应满足严格稳态燃烧约束"
    assert abs(res1.omega_opt - res1.omega_tar_projected) < 1e-3, "场景 1 未正确跟踪目标"
    assert_basic_physical_consistency(res1)
    print("\n场景 1 通过：严格模式下可行，且输出位于稳态带附近。")

    # -----------------------------------------------------
    # 场景 2：瞬态恢复型请求，允许目标带低于稳态最优带
    # burn_policy 使用 advisory，检验优化器可在更宽目标带内工作
    # -----------------------------------------------------
    comp2 = [0.25, 0.25, 0.20, 0.05, 0.15, 0.10]
    cleansed2 = composition_to_equivalent_properties(comp2)

    req2 = OptimizerRequest(
        slot_id="advisory_recovery",
        props=cleansed2.equivalent,
        omega_tar=0.280,
        resource=ResourceBoundary(T_stack_cap_C=980.0, v_stack_cap_mps=20.0),
        dyn_band=DynamicTargetBand(omega_min=0.260, omega_max=0.320),
        burn_policy="advisory",
    )
    res2 = optimize_static_slot(req2, cfg)
    assert res2.success, "场景 2 未得到数值可行解"
    assert abs(res2.omega_opt - res2.omega_tar_projected) < 1e-3, "场景 2 未正确跟踪目标"
    assert res2.omega_opt < cfg.OMEGA_STEADY_MIN, "场景 2 应落在稳态带下方，以模拟瞬态恢复请求"
    assert_basic_physical_consistency(res2)
    print("场景 2 通过：advisory 模式下可追踪低于稳态带的动态目标。")

    # -----------------------------------------------------
    # 场景 3：目标投影测试
    # 请求目标超出动态允许带，检查是否被正确投影
    # -----------------------------------------------------
    req3 = OptimizerRequest(
        slot_id="project_target",
        props=cleansed1.equivalent,
        omega_tar=0.360,
        resource=ResourceBoundary(T_stack_cap_C=900.0, v_stack_cap_mps=17.0),
        dyn_band=DynamicTargetBand(omega_min=0.290, omega_max=0.330),
        burn_policy="advisory",
    )
    res3 = optimize_static_slot(req3, cfg)
    assert res3.success, "场景 3 未得到数值可行解"
    assert abs(res3.omega_tar_projected - 0.330) < 1e-9, "场景 3 的目标未被正确投影到动态允许带上界"
    assert abs(res3.omega_opt - 0.330) < 1e-3, "场景 3 未正确跟踪投影后的目标"
    assert_basic_physical_consistency(res3)
    print("场景 3 通过：目标投影逻辑正常。")

    # -----------------------------------------------------
    # 生成一份示例控制表
    # -----------------------------------------------------
    out_csv = Path(__file__).with_name("optimizer_lookup_demo.csv")
    rows = build_lookup_table([req1, req2, req3], cfg, csv_path=str(out_csv))
    assert len(rows) == 3, "控制表行数不正确"
    print("\n已生成示例控制表: ./optimizer_lookup_demo.csv")
    print("\n测试全部通过。")


if __name__ == "__main__":
    main()
