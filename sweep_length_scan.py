#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立扫描脚本：扫描预热炉管长 L 对最优解的影响

固定求解参数
------------
- 上一时刻平均炉温 T_avg_prev = 886 °C
- 外层反馈增益 kappa = 0.5
- 垃圾组分：西瓜皮 = 1，其余 = 0
- 管长 L 从 6.8 m 开始，每次缩短 0.4 m，直到 2.8 m

说明
----
1. 本脚本会动态加载同目录下的 solve_inner_preheater.py
2. 对每个 L 创建独立的 Config(L=L_val)
3. 调用 solve_inner_lexicographic(...) 执行单点求解
4. 将关键结果打印到终端，并写出 CSV 文件
"""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
from typing import Any, Dict, List


def load_solver_module():
    """从同目录动态加载求解器模块。"""
    here = Path(__file__).resolve().parent
    solver_path = here / "solve_inner_preheater.py"
    if not solver_path.exists():
        raise FileNotFoundError(f"未找到求解器脚本: {solver_path}")

    spec = importlib.util.spec_from_file_location("solve_inner_preheater", solver_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载模块规范: {solver_path}")

    module = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def build_length_grid(start: float = 6.8, stop: float = 2.8, step: float = 0.4) -> List[float]:
    """生成 [6.8, 6.4, ..., 2.8] 形式的长度扫描序列。"""
    vals = []
    n = int(round((start - stop) / step))
    for i in range(n + 1):
        vals.append(round(start - i * step, 1))
    return vals


def to_row(L_val: float, result: Dict[str, Any]) -> Dict[str, Any]:
    cfg = result["config"]
    outer = result["outer"]
    overall = result["overall"]
    opt = result["optimal"]
    stages = result["stages"]

    qreq = opt["Qreq_kW"]
    qsup = opt["Qsup_kW"]
    tau_r = opt["tau_r_min"]
    tau_need = opt["tau_target_min"]

    return {
        "L_m": L_val,
        "A_m2": cfg.A,
        "M_H_kg": cfg.M_H,
        "tau_r_min": tau_r,
        "w_tar_k_pct": outer["w_tar_k"],
        "Tg_C": opt["Tg"],
        "vg_m_per_s": opt["vg"],
        "Tm_C": opt["Tm"],
        "w_opt_pct": opt["w_opt"],
        "dw_minus": opt["d_w_minus"],
        "dw_plus": opt["d_w_plus"],
        "Qsup_kW": qsup,
        "Qreq_kW": qreq,
        "Q_margin_kW": qsup - qreq,
        "Q_ratio": (qsup / qreq) if qreq != 0 else float("nan"),
        "tau_need_min": tau_need,
        "tau_margin_min": tau_r - tau_need,
        "Power_kW": opt["Power_kW"],
        "sigma_burn": opt["sigma_burn"],
        "mdot_preheater_kg_per_s": opt["mdot_preheater"],
        "Tmin_burn_C": opt["Tmin_burn"],
        "Tavg_burn_C": opt["Tavg_burn"],
        "Tmax_burn_C": opt["Tmax_burn"],
        "deltaT_C": opt["deltaT"],
        "stage1_obj": stages["stage1_objective"],
        "stage2_obj": stages["stage2_objective"],
        "stage3_obj": stages["stage3_objective"],
        "max_violation": overall["max_constraint_violation"],
        "feasible": overall["numerically_feasible"],
    }


def print_table(rows: List[Dict[str, Any]]) -> None:
    """简要打印核心指标，便于快速查看。"""
    headers = [
        "L_m", "A_m2", "tau_r_min", "w_tar_k_pct", "Tg_C", "vg_m_per_s",
        "Tm_C", "w_opt_pct", "Qsup_kW", "Qreq_kW", "Q_margin_kW",
        "tau_need_min", "tau_margin_min", "Power_kW", "feasible"
    ]

    print("=" * 160)
    print("管长扫描结果（核心字段）")
    print("=" * 160)
    print(" | ".join(f"{h:>14s}" for h in headers))
    print("-" * 160)

    for row in rows:
        vals = []
        for h in headers:
            v = row[h]
            if isinstance(v, bool):
                vals.append(f"{str(v):>14s}")
            elif isinstance(v, (int, float)):
                vals.append(f"{v:14.4f}")
            else:
                vals.append(f"{str(v):>14s}")
        print(" | ".join(vals))

    print("=" * 160)


def save_csv(rows: List[Dict[str, Any]], csv_path: Path) -> None:
    if not rows:
        raise ValueError("没有可写出的结果行。")

    fieldnames = list(rows[0].keys())
    with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    solver = load_solver_module()

    # 固定输入参数
    T_avg_prev = 886.0
    kappa = 0.5
    x = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]   # 西瓜皮 = 1，其余 = 0
    L_list = build_length_grid(6.8, 2.8, 0.4)

    rows: List[Dict[str, Any]] = []
    failed: List[Dict[str, Any]] = []

    for L_val in L_list:
        try:
            cfg = solver.Config(L=L_val)
            result = solver.solve_inner_lexicographic(
                x=x,
                T_avg_prev=T_avg_prev,
                cfg=cfg,
                kappa=kappa,
                use_outer_feedback=True,
            )
            rows.append(to_row(L_val, result))
        except Exception as e:
            failed.append({"L_m": L_val, "error": str(e)})

    print()
    print("固定参数：")
    print(f"  T_avg_prev = {T_avg_prev:.1f} °C")
    print(f"  kappa      = {kappa:.2f}")
    print(f"  x          = {x}  (仅西瓜皮)")
    print(f"  L_list     = {L_list}")
    print()

    if rows:
        print_table(rows)

    out_dir = Path(__file__).resolve().parent
    csv_path = out_dir / "sweep_length_scan_results.csv"
    if rows:
        save_csv(rows, csv_path)
        print(f"结果 CSV 已写出：{csv_path}")

    if failed:
        print()
        print("以下长度求解失败：")
        for item in failed:
            print(f"  L = {item['L_m']:.1f} m -> {item['error']}")


if __name__ == "__main__":
    main()
