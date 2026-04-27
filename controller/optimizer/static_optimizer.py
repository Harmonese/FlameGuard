from __future__ import annotations

"""
optimizer.py

静态运筹学优化器：未来物料槽位的预热炉设定值生成器

定位
----
本模块不承担外层动态控制、状态估计或时延传播。
它只解决一个静态问题：
在给定未来槽位等效物料性质、目标出口含水率、当前可用烟气资源边界，
以及动态允许目标带的前提下，求一组最优、可行的预热炉设定值。

输入
----
- EquivalentProperties: 等效物料性质 (omega0, tref, slope)
- omega_tar: 目标出口含水率（湿基小数）
- ResourceBoundary: 当前可用烟气温度/速度边界
- DynamicTargetBand: 当前动态允许目标带（湿基小数）
- burn_policy: "advisory" 或 "strict"

说明
----
在线动态闭环推荐使用 advisory：焚烧炉稳态代理量仅作为诊断输出。
strict 仅保留给离线筛选、极端保守场景或模型验证。

输出
----
- 最优设定值: Tg*, vg*, Tm*, omega_opt, w_opt_percent
- 诊断量: 约束核验、热负荷、资源占用、稳态燃烧代理量等

依赖
----
pip install numpy scipy
"""

from dataclasses import asdict, dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple
import csv
import math

import numpy as np
from scipy.optimize import minimize


# =========================================================
# 0) 数据类
# =========================================================


@dataclass(frozen=True)
class EquivalentProperties:
    """未来槽位等效物料性质。

    omega0: 入口初始湿基含水率（0~1）
    tref_min: 175°C 下达到 20% 含水率所需时间（min）
    slope_min_per_c: 温度敏感性斜率（min/°C，通常为负）
    """

    omega0: float
    tref_min: float
    slope_min_per_c: float


@dataclass(frozen=True)
class ResourceBoundary:
    """当前可用烟气资源边界。"""

    T_stack_cap_C: float
    v_stack_cap_mps: float


@dataclass(frozen=True)
class DynamicTargetBand:
    """当前动态允许目标带（湿基小数）。"""

    omega_min: float
    omega_max: float


@dataclass(frozen=True)
class OptimizerRequest:
    """静态优化请求。"""

    props: EquivalentProperties
    omega_tar: float
    resource: ResourceBoundary
    dyn_band: DynamicTargetBand
    slot_id: str = ""
    burn_policy: str = "advisory"  # advisory / strict; online dynamic control should prefer advisory


@dataclass
class OptimizerResult:
    success: bool
    slot_id: str
    burn_policy: str
    omega_tar_requested: float
    omega_tar_projected: float
    omega_band_applied_min: float
    omega_band_applied_max: float
    Tg_star_C: float
    vg_star_mps: float
    Tm_star_C: float
    omega_opt: float
    w_opt_percent: float
    d_omega_minus: float
    d_omega_plus: float
    tau20_min: float
    tau_target_min: float
    tau_r_min: float
    Qsup_kW: float
    Qreq_kW: float
    power_kW: float
    mdot_stack_cap_kgps: float
    mdot_preheater_kgps: float
    Tavg_burn_C: float
    Tmin_burn_C: float
    Tmax_burn_C: float
    sigma_burn_C: float
    steady_band_violation_percent: float
    feasible_under_strict_burn: bool
    stage1_objective: float
    stage2_objective: float
    stage3_objective: float
    max_constraint_violation: float
    message: str = ""


@dataclass
class Config:
    # 参考温度与环境温度
    T_REF: float = 175.0
    T0: float = 20.0
    T_AMB: float = 20.0

    # 处理量与设备尺寸
    MDOT_W: float = 20000.0 / 86400.0  # kg/s
    D: float = 1.2
    L: float = 3.2
    PHI: float = 0.14
    DTUBE: float = 0.168
    N_TUBES: int = 6
    R_DUCT: float = 0.4
    R_STACK: float = 0.3
    RHO_BULK: float = 450.0

    # 传热模型
    U0: float = 18.97
    K_U: float = 20.09
    N_U: float = 0.65

    # 物性参数
    CPG: float = 1.05
    CS: float = 1.70
    CW: float = 4.1844
    LAMBDA: float = 2257.9
    RHO_G_REF: float = 0.78
    T_G_REF_K: float = 450.0

    # 设备边界
    VG_MIN: float = 3.0
    VG_MAX: float = 12.0
    TG_MIN: float = 100.0
    TG_SAFE_MAX: float = 2000.0
    TM_MAX: float = 250.0
    DELTA_T_MIN: float = 0.0

    # 线性动力学模型的适用下限（基于实验定义到 20%）
    OMEGA_MODEL_MIN: float = 0.20
    OMEGA_MODEL_MAX: float = 0.98

    # 焚烧炉稳态代理模型（最新工程口径，单位 °C / m/s）
    TAVG_A: float = -13.109632
    TAVG_B: float = 1294.871365
    TMIN_A: float = -13.422320
    TMIN_B: float = 1330.692379
    TMAX_A: float = -16.072025
    TMAX_B: float = 1589.019616
    SIGMA_A: float = -0.189997
    SIGMA_B: float = 18.331239

    # 稳态燃烧硬约束（仅 strict 模式下作为硬约束）
    TMIN_BURN_MIN: float = 850.0
    TAVG_BURN_MIN: float = 850.0
    TAVG_BURN_MAX: float = 1100.0
    TMAX_BURN_MAX: float = 1100.0

    # 当前最新稳态最优带（由主焚烧炉静态代理模型反推）
    OMEGA_STEADY_MIN: float = 0.3042675804697915
    OMEGA_STEADY_MAX: float = 0.3393469511577442

    # 数值求解设置
    MAXITER: int = 120
    STAGE_TOL: float = 1e-6
    FEAS_TOL: float = 1e-2
    BIG_SLACK: float = 1e6

    def __post_init__(self) -> None:
        self.A_D = math.pi * self.R_DUCT ** 2
        self.A_S = math.pi * self.R_STACK ** 2
        self.A = math.pi * self.D * self.L + self.N_TUBES * math.pi * self.DTUBE * self.L
        self.M_H = self.RHO_BULK * self.PHI * (math.pi * self.D ** 2 / 4.0) * self.L
        self.TAU_R_SEC = self.M_H / self.MDOT_W
        self.TAU_R_MIN = self.TAU_R_SEC / 60.0


IDX_TG = 0
IDX_VG = 1
IDX_TM = 2
IDX_OMEGA = 3
IDX_DOMM = 4
IDX_DOMP = 5


# =========================================================
# 1) 基本物理与代理函数
# =========================================================


def validate_equivalent_properties(props: EquivalentProperties) -> EquivalentProperties:
    if not (0.0 < props.omega0 < 1.0):
        raise ValueError("omega0 必须位于 (0, 1) 区间。")
    if props.tref_min <= 0.0:
        raise ValueError("tref_min 必须为正数。")
    return props


def validate_resource_boundary(resource: ResourceBoundary, cfg: Config) -> ResourceBoundary:
    if resource.T_stack_cap_C < cfg.TG_MIN:
        raise ValueError(
            f"当前烟气温度上限仅 {resource.T_stack_cap_C:.3f}°C，低于设备最小可运行温度 {cfg.TG_MIN:.1f}°C。"
        )
    if resource.v_stack_cap_mps <= 0.0:
        raise ValueError("烟囱平均速度必须为正数。")
    return resource


def validate_dynamic_target_band(band: DynamicTargetBand) -> DynamicTargetBand:
    if band.omega_min >= band.omega_max:
        raise ValueError("动态允许目标带必须满足 omega_min < omega_max。")
    return band


def ceq_from_props(props: EquivalentProperties, cfg: Config) -> float:
    return (1.0 - props.omega0) * cfg.CS + props.omega0 * cfg.CW


def rho_g(T_degC: float, cfg: Config) -> float:
    return cfg.RHO_G_REF * cfg.T_G_REF_K / (T_degC + 273.15)


def q_sup_kW(Tg: float, vg: float, Tm: float, cfg: Config) -> float:
    U = cfg.U0 + cfg.K_U * (vg ** cfg.N_U)
    return U * cfg.A * (Tg - Tm) / 1000.0


def evap_water_per_kg_wet(omega0: float, omega_target: float) -> float:
    if omega_target >= 1.0:
        raise ValueError("目标湿基含水率必须小于 1。")
    if omega0 <= omega_target:
        return 0.0
    return (omega0 - omega_target) / (1.0 - omega_target)


def q_req_kW(Tm: float, props: EquivalentProperties, omega_target: float, cfg: Config) -> float:
    ceq = ceq_from_props(props, cfg)
    m_evap = evap_water_per_kg_wet(props.omega0, omega_target)
    return cfg.MDOT_W * (ceq * (Tm - cfg.T0) + cfg.LAMBDA * m_evap)


def tau20(Tm: float, props: EquivalentProperties, cfg: Config) -> float:
    return props.tref_min + props.slope_min_per_c * (Tm - cfg.T_REF)


def tau_target(Tm: float, props: EquivalentProperties, omega_target: float, cfg: Config) -> float:
    denom = props.omega0 - cfg.OMEGA_MODEL_MIN
    if denom < 0:
        raise ValueError("omega0 < 0.20，当前线性缩放动力学模型不适用。")
    if abs(denom) <= 1e-12:
        if abs(props.omega0 - omega_target) <= 1e-12:
            return 0.0
        raise ValueError("omega0 == 0.20 时仅允许 omega_target == 0.20。")
    return tau20(Tm, props, cfg) * (props.omega0 - omega_target) / denom


def power_kW(Tg: float, vg: float, cfg: Config) -> float:
    return rho_g(Tg, cfg) * cfg.A_D * vg * cfg.CPG * (Tg - cfg.T_AMB)


def mdot_stack_cap(resource: ResourceBoundary, cfg: Config) -> float:
    return rho_g(resource.T_stack_cap_C, cfg) * cfg.A_S * resource.v_stack_cap_mps


def mdot_preheater(Tg: float, vg: float, cfg: Config) -> float:
    return rho_g(Tg, cfg) * cfg.A_D * vg


def T_avg_proxy(w_percent: float, cfg: Config) -> float:
    return cfg.TAVG_A * w_percent + cfg.TAVG_B


def T_min_proxy(w_percent: float, cfg: Config) -> float:
    return cfg.TMIN_A * w_percent + cfg.TMIN_B


def T_max_proxy(w_percent: float, cfg: Config) -> float:
    return cfg.TMAX_A * w_percent + cfg.TMAX_B


def sigma_proxy(w_percent: float, cfg: Config) -> float:
    return cfg.SIGMA_A * w_percent + cfg.SIGMA_B


def strict_burn_feasible(omega: float, cfg: Config) -> bool:
    w = 100.0 * omega
    return bool(
        T_min_proxy(w, cfg) >= cfg.TMIN_BURN_MIN - 1e-6
        and T_avg_proxy(w, cfg) >= cfg.TAVG_BURN_MIN - 1e-6
        and T_avg_proxy(w, cfg) <= cfg.TAVG_BURN_MAX + 1e-6
        and T_max_proxy(w, cfg) <= cfg.TMAX_BURN_MAX + 1e-6
    )


def steady_band_violation_percent(omega: float, cfg: Config) -> float:
    w = 100.0 * omega
    lo = 100.0 * cfg.OMEGA_STEADY_MIN
    hi = 100.0 * cfg.OMEGA_STEADY_MAX
    if w < lo:
        return lo - w
    if w > hi:
        return w - hi
    return 0.0


# =========================================================
# 2) 目标带与边界处理
# =========================================================


def intersect_target_band(
    props: EquivalentProperties,
    dyn_band: DynamicTargetBand,
    cfg: Config,
) -> Tuple[float, float]:
    """动态允许带与物理可实现带的交集。

    物理可实现带：
    - 下界不能低于动力学模型定义的 20%
    - 上界不能高于入口初始含水率（预热炉不会“加水”）
    """
    phys_min = cfg.OMEGA_MODEL_MIN
    phys_max = min(props.omega0, cfg.OMEGA_MODEL_MAX)
    lo = max(dyn_band.omega_min, phys_min)
    hi = min(dyn_band.omega_max, phys_max)
    if lo > hi:
        raise ValueError(
            "动态允许目标带与物理可实现带没有交集。"
            f" dyn=[{dyn_band.omega_min:.4f},{dyn_band.omega_max:.4f}],"
            f" phys=[{phys_min:.4f},{phys_max:.4f}]"
        )
    return lo, hi


def project_omega_target(
    omega_tar: float,
    omega_band: Tuple[float, float],
) -> float:
    lo, hi = omega_band
    return max(lo, min(hi, omega_tar))


# =========================================================
# 3) SLSQP 求解核心
# =========================================================


def build_initial_guess(Tg: float, vg: float, Tm: float, omega: float, omega_tar: float) -> np.ndarray:
    if omega > omega_tar:
        domm, domp = 0.0, omega - omega_tar
    else:
        domm, domp = omega_tar - omega, 0.0
    return np.array([Tg, vg, Tm, omega, domm, domp], dtype=float)


def initial_guesses(
    props: EquivalentProperties,
    resource: ResourceBoundary,
    omega_tar: float,
    omega_band: Tuple[float, float],
    cfg: Config,
) -> List[np.ndarray]:
    lo, hi = omega_band
    Tg_hi = max(cfg.TG_MIN, min(resource.T_stack_cap_C, cfg.TG_SAFE_MAX))
    Tg_candidates = sorted(set([
        cfg.TG_MIN,
        min(300.0, Tg_hi),
        Tg_hi,
    ]))
    vg_candidates = [3.5, 8.0, 11.0]
    Tm_candidates = [55.0, 90.0]
    omega_candidates = sorted(set([
        lo,
        min(max(omega_tar, lo), hi),
        hi,
    ]))
    seeds: List[np.ndarray] = []
    for Tg in Tg_candidates:
        for vg in vg_candidates:
            for Tm in Tm_candidates:
                if Tm <= cfg.TM_MAX and Tg - Tm >= cfg.DELTA_T_MIN:
                    for omega in omega_candidates:
                        seeds.append(build_initial_guess(Tg, vg, Tm, omega, omega_tar))
    return seeds


def bounds(
    resource: ResourceBoundary,
    omega_band: Tuple[float, float],
    cfg: Config,
) -> List[Tuple[float, float]]:
    lo, hi = omega_band
    Tg_hi = min(resource.T_stack_cap_C, cfg.TG_SAFE_MAX)
    if Tg_hi < cfg.TG_MIN:
        raise ValueError(
            f"烟气温度上限 {Tg_hi:.3f}°C 低于设备最小可运行温度 {cfg.TG_MIN:.1f}°C。"
        )
    return [
        (cfg.TG_MIN, Tg_hi),
        (cfg.VG_MIN, cfg.VG_MAX),
        (cfg.T0, cfg.TM_MAX),
        (lo, hi),
        (0.0, cfg.BIG_SLACK),
        (0.0, cfg.BIG_SLACK),
    ]


def eq_constraints(z: np.ndarray, omega_tar: float) -> np.ndarray:
    omega = z[IDX_OMEGA]
    domm = z[IDX_DOMM]
    domp = z[IDX_DOMP]
    return np.array([omega + domm - domp - omega_tar], dtype=float)


def ineq_constraints(
    z: np.ndarray,
    props: EquivalentProperties,
    resource: ResourceBoundary,
    cfg: Config,
    burn_policy: str,
) -> np.ndarray:
    Tg, vg, Tm, omega = z[IDX_TG], z[IDX_VG], z[IDX_TM], z[IDX_OMEGA]

    g_tau = cfg.TAU_R_MIN - tau_target(Tm, props, omega, cfg)
    g_heat = q_sup_kW(Tg, vg, Tm, cfg) - q_req_kW(Tm, props, omega, cfg)
    g_resource_mdot = mdot_stack_cap(resource, cfg) - mdot_preheater(Tg, vg, cfg)
    g_tm_tg = Tg - Tm - cfg.DELTA_T_MIN
    g_no_add_water = props.omega0 - omega

    vals = [g_tau, g_heat, g_resource_mdot, g_tm_tg, g_no_add_water]

    if burn_policy == "strict":
        w = 100.0 * omega
        vals.extend([
            T_min_proxy(w, cfg) - cfg.TMIN_BURN_MIN,
            T_avg_proxy(w, cfg) - cfg.TAVG_BURN_MIN,
            cfg.TAVG_BURN_MAX - T_avg_proxy(w, cfg),
            cfg.TMAX_BURN_MAX - T_max_proxy(w, cfg),
        ])
    elif burn_policy != "advisory":
        raise ValueError("burn_policy 只能是 'advisory' 或 'strict'。")

    return np.array(vals, dtype=float)


def objective_stage1(z: np.ndarray) -> float:
    return z[IDX_DOMM] + z[IDX_DOMP] + 1e-8 * (z[IDX_TG] + z[IDX_VG] + z[IDX_TM])


def objective_stage2(z: np.ndarray, cfg: Config) -> float:
    w = 100.0 * z[IDX_OMEGA]
    return sigma_proxy(w, cfg)


def objective_stage3(z: np.ndarray, cfg: Config) -> float:
    return power_kW(z[IDX_TG], z[IDX_VG], cfg)


def max_constraint_violation(
    z: np.ndarray,
    props: EquivalentProperties,
    resource: ResourceBoundary,
    omega_tar: float,
    cfg: Config,
    burn_policy: str,
    extra_ineqs: Optional[Sequence] = None,
) -> float:
    extra_ineqs = list(extra_ineqs or [])
    eqv = np.max(np.abs(eq_constraints(z, omega_tar)))
    ineqv = np.max(np.maximum(-ineq_constraints(z, props, resource, cfg, burn_policy), 0.0))
    extra = 0.0
    if extra_ineqs:
        vals = np.array([f(z) for f in extra_ineqs], dtype=float)
        extra = np.max(np.maximum(-vals, 0.0))
    return max(eqv, ineqv, extra)


def solve_stage(
    props: EquivalentProperties,
    resource: ResourceBoundary,
    omega_tar: float,
    omega_band: Tuple[float, float],
    cfg: Config,
    objective,
    burn_policy: str,
    extra_ineqs: Optional[Sequence] = None,
    seeds_override: Optional[Sequence[np.ndarray]] = None,
):
    extra_ineqs = list(extra_ineqs or [])
    seeds = list(seeds_override) if seeds_override is not None else initial_guesses(props, resource, omega_tar, omega_band, cfg)
    example_seed = seeds[0]
    n_ineq = ineq_constraints(example_seed, props, resource, cfg, burn_policy).shape[0]

    cons = [{"type": "eq", "fun": lambda z: eq_constraints(z, omega_tar)[0]}]
    for i in range(n_ineq):
        cons.append({
            "type": "ineq",
            "fun": lambda z, idx=i: ineq_constraints(z, props, resource, cfg, burn_policy)[idx],
        })
    for f in extra_ineqs:
        cons.append({"type": "ineq", "fun": f})

    best_res = None
    best_obj = np.inf
    for x0 in seeds:
        try:
            res = minimize(
                objective,
                x0,
                method="SLSQP",
                bounds=bounds(resource, omega_band, cfg),
                constraints=cons,
                options={"maxiter": cfg.MAXITER, "ftol": 1e-10, "disp": False},
            )
        except Exception:
            continue

        if res.x is None:
            continue

        vio = max_constraint_violation(res.x, props, resource, omega_tar, cfg, burn_policy, extra_ineqs)
        obj = objective(res.x)
        if vio <= 1e-5 and obj < best_obj:
            best_obj = obj
            best_res = res
        elif best_res is None and (res.success or vio < 1e-2):
            best_obj = obj
            best_res = res

    if best_res is None:
        raise RuntimeError("求解失败：未找到可行解。建议检查目标带、资源边界或 burn_policy。")
    return best_res


# =========================================================
# 4) 公开接口：单槽位静态优化
# =========================================================


def optimize_static_slot(
    request: OptimizerRequest,
    cfg: Optional[Config] = None,
) -> OptimizerResult:
    cfg = cfg or Config()
    props = validate_equivalent_properties(request.props)
    resource = validate_resource_boundary(request.resource, cfg)
    dyn_band = validate_dynamic_target_band(request.dyn_band)

    omega_band = intersect_target_band(props, dyn_band, cfg)
    omega_tar_projected = project_omega_target(request.omega_tar, omega_band)

    res1 = solve_stage(
        props,
        resource,
        omega_tar_projected,
        omega_band,
        cfg,
        objective=objective_stage1,
        burn_policy=request.burn_policy,
    )
    stage1_star = objective_stage1(res1.x)

    extra2 = [lambda z, cap=stage1_star + cfg.STAGE_TOL: cap - objective_stage1(z)]
    res2 = solve_stage(
        props,
        resource,
        omega_tar_projected,
        omega_band,
        cfg,
        objective=lambda z: objective_stage2(z, cfg),
        burn_policy=request.burn_policy,
        extra_ineqs=extra2,
        seeds_override=[res1.x],
    )
    stage2_star = objective_stage2(res2.x, cfg)

    extra3 = [
        lambda z, cap=stage1_star + cfg.STAGE_TOL: cap - objective_stage1(z),
        lambda z, cap=stage2_star + cfg.STAGE_TOL: cap - objective_stage2(z, cfg),
    ]
    res3 = solve_stage(
        props,
        resource,
        omega_tar_projected,
        omega_band,
        cfg,
        objective=lambda z: objective_stage3(z, cfg),
        burn_policy=request.burn_policy,
        extra_ineqs=extra3,
    )

    z = res3.x
    Tg, vg, Tm, omega = z[IDX_TG], z[IDX_VG], z[IDX_TM], z[IDX_OMEGA]
    w = 100.0 * omega

    final_vio = max_constraint_violation(
        z, props, resource, omega_tar_projected, cfg, request.burn_policy, extra3
    )

    result = OptimizerResult(
        success=bool(final_vio <= cfg.FEAS_TOL),
        slot_id=request.slot_id,
        burn_policy=request.burn_policy,
        omega_tar_requested=float(request.omega_tar),
        omega_tar_projected=float(omega_tar_projected),
        omega_band_applied_min=float(omega_band[0]),
        omega_band_applied_max=float(omega_band[1]),
        Tg_star_C=float(Tg),
        vg_star_mps=float(vg),
        Tm_star_C=float(Tm),
        omega_opt=float(omega),
        w_opt_percent=float(w),
        d_omega_minus=float(z[IDX_DOMM]),
        d_omega_plus=float(z[IDX_DOMP]),
        tau20_min=float(tau20(Tm, props, cfg)),
        tau_target_min=float(tau_target(Tm, props, omega, cfg)),
        tau_r_min=float(cfg.TAU_R_MIN),
        Qsup_kW=float(q_sup_kW(Tg, vg, Tm, cfg)),
        Qreq_kW=float(q_req_kW(Tm, props, omega, cfg)),
        power_kW=float(power_kW(Tg, vg, cfg)),
        mdot_stack_cap_kgps=float(mdot_stack_cap(resource, cfg)),
        mdot_preheater_kgps=float(mdot_preheater(Tg, vg, cfg)),
        Tavg_burn_C=float(T_avg_proxy(w, cfg)),
        Tmin_burn_C=float(T_min_proxy(w, cfg)),
        Tmax_burn_C=float(T_max_proxy(w, cfg)),
        sigma_burn_C=float(sigma_proxy(w, cfg)),
        steady_band_violation_percent=float(steady_band_violation_percent(omega, cfg)),
        feasible_under_strict_burn=bool(strict_burn_feasible(omega, cfg)),
        stage1_objective=float(stage1_star),
        stage2_objective=float(stage2_star),
        stage3_objective=float(objective_stage3(z, cfg)),
        max_constraint_violation=float(final_vio),
        message=("advisory online mode" if request.burn_policy == "advisory" else "strict steady-burn mode")
    )
    return result


# =========================================================
# 5) 离线制表辅助
# =========================================================


def result_to_row(result: OptimizerResult, request: OptimizerRequest) -> Dict[str, Any]:
    row = asdict(result)
    row.update({
        "omega0": request.props.omega0,
        "tref_min": request.props.tref_min,
        "slope_min_per_c": request.props.slope_min_per_c,
        "T_stack_cap_C": request.resource.T_stack_cap_C,
        "v_stack_cap_mps": request.resource.v_stack_cap_mps,
        "dyn_band_omega_min": request.dyn_band.omega_min,
        "dyn_band_omega_max": request.dyn_band.omega_max,
    })
    return row


def build_lookup_table(
    requests: Iterable[OptimizerRequest],
    cfg: Optional[Config] = None,
    csv_path: Optional[str] = None,
) -> List[Dict[str, Any]]:
    cfg = cfg or Config()
    rows: List[Dict[str, Any]] = []
    for req in requests:
        try:
            result = optimize_static_slot(req, cfg)
            rows.append(result_to_row(result, req))
        except Exception as exc:
            rows.append({
                "slot_id": req.slot_id,
                "burn_policy": req.burn_policy,
                "omega0": req.props.omega0,
                "tref_min": req.props.tref_min,
                "slope_min_per_c": req.props.slope_min_per_c,
                "omega_tar_requested": req.omega_tar,
                "T_stack_cap_C": req.resource.T_stack_cap_C,
                "v_stack_cap_mps": req.resource.v_stack_cap_mps,
                "dyn_band_omega_min": req.dyn_band.omega_min,
                "dyn_band_omega_max": req.dyn_band.omega_max,
                "success": False,
                "message": f"{type(exc).__name__}: {exc}",
            })

    if csv_path is not None and rows:
        fieldnames = sorted({key for row in rows for key in row.keys()})
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
    return rows


__all__ = [
    "EquivalentProperties",
    "ResourceBoundary",
    "DynamicTargetBand",
    "OptimizerRequest",
    "OptimizerResult",
    "Config",
    "optimize_static_slot",
    "build_lookup_table",
    "strict_burn_feasible",
    "steady_band_violation_percent",
    "T_avg_proxy",
    "T_min_proxy",
    "T_max_proxy",
    "sigma_proxy",
]
