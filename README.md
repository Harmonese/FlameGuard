# 稳燃宝（FlameGuard）

稳燃宝（FlameGuard）是一个面向垃圾焚烧预热炉的闭环模型预测控制项目。系统利用主焚烧炉可提供的烟气余热，对进入主焚烧炉前的垃圾进行预热和脱水，并动态优化预热炉换热侧的烟气入口温度 `Tg` 与循环速度 `vg`，使焚烧炉温度、预热炉出口含水率、补热需求、辅助循环需求和风机代价保持在可接受范围内。

项目的默认可运行对象是 Python 低阶 plant。控制器内部维护独立的 Python predictor，用于 NMPC 的快速滚动预测。Plant 与 predictor 是两套实现：plant 表示被控对象或其代理，predictor 表示控制器脑内模型。二者只通过 `domain/` 中的数据结构和接口语义连接。

核心控制目标是：

```text
主焚烧炉平均温度 T_avg ≥ 850 °C
主温度目标 T_target = 873 °C
预热炉出口湿基含水率 omega_b 与炉膛干基质量流率共同决定焚烧炉静态响应
补热、补风和辅助循环作为允许的运行手段计入代价，而不是直接视为失败
```

---

## 目录结构

```text
FlameGuard-main/
├── domain/                 稳定数据契约与接口协议
│   ├── types.py
│   └── interfaces.py
│
├── plant/                  被控对象 backend
│   ├── factory.py           PlantBackend 装配入口
│   ├── python_model/        默认 Python 低阶 plant
│   ├── comsol/              COMSOL backend 占位
│   └── hardware/            实机 backend 占位
│
├── controller/             控制器
│   ├── factory.py           estimator / operator / executor 装配入口
│   ├── estimator/           状态估计与扰动估计
│   ├── predictor/           控制器内部预测模型
│   ├── operator/            NMPC / fallback 决策器
│   └── executor/            控制设定到执行命令的安全适配
│
├── runtime/                闭环仿真、测试场景、telemetry、绘图
│   ├── simulator.py
│   ├── telemetry.py
│   ├── plotting.py
│   └── tests/
│
├── scripts/                建模与拟合脚本
│   ├── fit_furnace_static_surrogate.py
│   ├── furnace_static_surrogate_fit.json
│   └── data/furnace_static_comsol/
│
├── README.md
└── CHANGELOG.md
```

---

## 架构边界

### `domain/`

`domain/` 定义跨层共享的数据语言，不包含物理公式或控制算法。主要类型包括：

| 类型 | 作用 |
|---|---|
| `FeedstockObservation` | 入口垃圾物性观测，不暴露固定垃圾类别 |
| `EquivalentProperties` | plant/predictor 内部使用的等效物料性质 |
| `PreheaterState` | 20 槽预热炉状态 |
| `PreheaterOutput` | 轻量预热炉出口信息，可用于没有完整 20 槽状态的 backend |
| `FurnaceFeed` | 进入焚烧炉的质量流率与含水率输入 |
| `FurnaceObservation` | 焚烧炉观测：`T_avg_C`, `T_stack_C`, `v_stack_mps` |
| `PlantStepInput` | runtime 发给 plant backend 的一步输入 |
| `PlantSnapshot` | plant backend 返回的观测快照 |
| `StateEstimate` | 控制器内部状态估计 |
| `PredictorBundle` | estimator 管理的 controller-side predictor 对象集合 |
| `OperatorContext` | operator 每次决策所需的完整上下文 |
| `ControlSetpoint` | operator 输出给 executor 的参考控制设定 |
| `ActuatorCommand` | executor 输出给 plant 的实际命令 |
| `MPCDecision` | NMPC 决策结果与预测摘要 |

### `plant/`

`plant/` 表示外部被控对象。当前默认可运行 backend 是 `plant/python_model/`。COMSOL 与实机接入时，应实现同样的 `PlantBackend` 语义：

```text
PlantStepInput(command, feedstock, dt)
        ↓
PlantBackend.step(...)
        ↓
PlantSnapshot(furnace_obs, optional preheater_state, optional preheater_output, resource, feedback, health)
```

Plant backend 只负责表达外部对象的输入输出。它不实现 NMPC，不持有 controller predictor，也不直接调用 controller operator。

### `controller/`

`controller/` 表示控制器自身。它由四部分组成：

| 模块 | 职责 |
|---|---|
| `estimator/` | 从 `PlantSnapshot` 生成 `StateEstimate`，维护 controller-side predictor 状态，估计炉膛扰动 |
| `predictor/` | 为 NMPC 提供可克隆、可 rollout 的预热炉、焚烧炉、执行器和资源模型 |
| `operator/` | 根据 `OperatorContext` 做控制决策，默认主控为 block-SLSQP NMPC |
| `executor/` | 将 `ControlSetpoint` 转换为带限幅、速率、一阶执行器动态和资源诊断的 `ActuatorCommand` |

### `runtime/`

`runtime/` 负责实验编排：构建场景，调用 `plant.factory.make_plant_backend()` 与 `controller.factory` 装配闭环，推进仿真，记录 telemetry，输出 CSV、metrics 和总览图。Runtime 可以了解测试场景，但不再手动实例化 Python plant 的 preheater/furnace/resource 内部模型，也不把 plant 内部模型当作 controller predictor 使用。

---

## 闭环顺序

一次闭环采样周期的主流程为：

```text
Scenario composition schedule
        ↓  测试适配器
FeedstockObservation
        ↓
PlantStepInput(previous ActuatorCommand, feedstock, dt)
        ↓
PythonPlantBackend.step(...)
        ↓
PlantSnapshot
        ↓
ControllerStateEstimator.update(...)
        ↓
StateEstimate + PredictorBundle
        ↓
NonlinearMPCController.step_context(OperatorContext)
        ↓
MPCDecision / ControlSetpoint
        ↓
ControlExecutor.translate_setpoint(...)
        ↓
ActuatorCommand
        ↓
下一周期 PlantBackend.step(...)
```

NMPC 内部使用 `controller/predictor/` 做未来 600 s 左右的 rollout。每次优化求得未来多个控制块的 `Tg_ref_C` 和 `vg_ref_mps`，闭环只执行当前时刻的第一段设定。

---

## 入口垃圾协议

控制协议使用垃圾物性，而不是固定垃圾类别。`FeedstockObservation` 包含：

```python
FeedstockObservation(
    time_s,
    moisture_wb,
    drying_time_ref_min,
    drying_sensitivity_min_per_C,
    bulk_density_kg_m3=None,
    wet_mass_flow_kgps=None,
    source="...",
    confidence=1.0,
    raw={...},
)
```

字段含义：

| 字段 | 含义 |
|---|---|
| `moisture_wb` | 入口湿基含水率，范围 0~1 |
| `drying_time_ref_min` | 参考干燥时间，单位 min |
| `drying_sensitivity_min_per_C` | 干燥时间对温度的敏感性，单位 min/°C，通常为负 |
| `bulk_density_kg_m3` | 垃圾堆积密度，可缺省 |
| `wet_mass_flow_kgps` | 入口湿垃圾质量流率，可缺省 |
| `source` | 物性来源，例如 `scenario`, `vision_ai`, `manual`, `lab_measurement` |
| `confidence` | 物性估计可信度 |
| `raw` | 原始识别、人工记录或调试数据，只用于日志和排查 |

`runtime/tests/` 中仍使用 6 类垃圾 composition 构造标准场景。composition 只属于测试适配器，进入 plant 和 controller 前会被转换为 `FeedstockObservation`。

---

## 质量流率语义

预热炉入口湿质量流率以日处理量换算为连续质量流：

```text
wet_mass_flow_kgps = daily_wet_feed_kg / 86400
```

默认配置为：

```text
daily_wet_feed_kg = 20000 kg/day
wet_mass_flow_kgps ≈ 0.2315 kg/s
```

`SimConfig.wet_mass_flow_override_kgps` 可用于扫描或 what-if 实验。没有 override 时，runtime 使用日处理量公式生成 `FeedstockObservation.wet_mass_flow_kgps`。

预热炉出口质量流率由模型库存计算：

```text
dry_out_kgps   = 出口干基质量流率
water_out_kgps = 出口水分质量流率
wet_out_kgps   = dry_out_kgps + water_out_kgps
omega_out      = water_out_kgps / wet_out_kgps
```

焚烧炉输入使用工程变量 `FurnaceFeed`：

```python
FurnaceFeed(
    omega_b,
    mdot_d_kgps,
    mdot_water_kgps,
    mdot_wet_kgps,
    rd,
)
```

其中：

```text
omega_b = 焚烧炉入口湿基含水率
mdot_d_kgps = 进入焚烧炉的干基垃圾质量流率
rd = mdot_d_kgps / mdot_d_ref_kgps
```

`rd` 是 COMSOL 静态代理模型的归一化坐标和诊断量。工程分析和图表优先使用 `mdot_d_kgps`。

---

## Python plant 建模

`plant/python_model/` 包含默认被控对象模型。

### 预热炉

预热炉由 `PreheaterForwardModel` 表示，是一个 20 槽轴向分布模型。每个 cell 保存：

```text
omega
T_solid_C
omega0
tref_min
slope_min_per_C
dry_mass_kg
water_mass_kg
bulk_density_kg_m3
residence_left_s
```

主要过程包括：

1. feed delay：入口垃圾物性存在约 5 s 进入延迟；
2. 轴向输送：用 residence time 将垃圾从 cell 0 推向 cell 19；
3. 逆流烟气换热：默认烟气从出口侧向入口侧流动；
4. 显热升温与潜热蒸发：100°C 以下优先升温，到达蒸发条件后脱水；
5. 出口质量流率诊断：输出干基、水分和湿基质量流率。

当前低阶预热炉代理模型允许垃圾继续深度干燥，残余湿基含水率下限为 `omega_min = 0.05`。这个下限只用于避免数值上出现完全无水或负水分，并不再把出口含水率硬限制在 20%。plant 和 controller predictor 使用相同默认值；未来如果需要更保守或更激进的干燥能力，应同时调整两侧配置。

### 焚烧炉

焚烧炉由静态 COMSOL surrogate 加低阶动态壳组成。

静态输入为：

```text
omega_b                 焚烧炉入口湿基含水率
mdot_d_furnace_kgps     进入焚烧炉的干基质量流率
```

内部将 `mdot_d_furnace_kgps` 转换为：

```text
rd = mdot_d_furnace_kgps / mdot_d_ref_kgps
```

静态输出包括：

```text
T_avg_C
T_stack_C
v_stack_mps
T_surface_min_C
T_surface_max_C
T_surface_std_C
```

动态壳保留：

```text
5 s dead time
0.223 s fast lag
75.412 s slow lag
```

这使焚烧炉不会瞬时跳到静态代理输出，而是保留温度和烟囱响应的低阶动态。

### 烟气资源

`ResourceModel` 从 `FurnaceObservation` 计算自然烟气资源：

```text
T_stack_available_C
v_stack_available_mps
mdot_stack_available_kgps
```

执行器使用这些资源做补热和辅助循环诊断。当前语义为：

| 指标 | 含义 |
|---|---|
| `aux_heat_enable` | 自然烟气温度不足，需要辅助补热 |
| `Q_aux_heat_kW` | 补热功率估计 |
| `mdot_aux_flow_kgps` | 自然烟气质量流量不足时的辅助循环需求 |
| `fan_circulation_power_kW` | 循环速度对应的风机/循环功率诊断 |
| `aux_resource_required` | 存在补热或辅助循环需求的综合标记 |

`aux_resource_required` 不等价于控制失败。厨余垃圾高湿工况下，补热、补风和辅助循环可以是正常运行方式。Metrics 中同时输出 `aux_heat_required_ratio`、`aux_circulation_required_ratio` 和 `command_hard_clipped_by_resource_ratio`，用于区分运行成本与命令硬削弱。

---

## Controller predictor

`controller/predictor/` 是控制器内部模型，包含与 plant 相同类别的模块：预热炉、焚烧炉、执行器、资源、物料和热工公式。它们是 controller 自己的实现，不从 `plant/python_model/` import。

Predictor 的职责是：

```text
从 StateEstimate 初始化控制器内部状态
在 NMPC 中克隆
按候选 Tg/vg 控制序列 rollout
提供未来 T_avg、omega_out、资源需求和经济代价
```

Plant 与 predictor 使用相同接口语义，但允许参数和实现逐步分化。模型失配通过 estimator、扰动观测器和闭环反馈处理。

---

## State estimator

`controller/estimator/state_estimator.py` 定义 `ControllerStateEstimator`。它负责：

```text
PlantSnapshot → StateEstimate
```

它始终向 operator 提供可预测的 20 槽 `preheater_state_est`。

如果 plant snapshot 包含完整 `PreheaterState`，estimator 将其同步到 controller predictor。若没有完整 20 槽状态，estimator 使用 controller-side predictor digital twin，根据上周期实际命令或 actuator feedback 推进内部状态。

Estimator 还维护：

```text
preheater_predictor
furnace_predictor
resource_model
disturbance_observer
```

并通过 `get_predictor_bundle()` 交给 operator 使用。

---

## NMPC operator

主控器为 `controller/operator/nmpc_operator.py` 中的 `NonlinearMPCController`。它是 control-blocking NMPC，默认设置包括：

| 参数 | 默认值 |
|---|---:|
| prediction horizon | 600 s |
| decision grid | 20 s |
| rollout substep | 5 s |
| control blocks | 0-120, 120-300, 300-480, 480-600 s |
| reoptimize period | 60 s |
| temperature target | 873 °C |
| compliance lower bound | 850 °C |

决策变量为每个控制块的：

```text
Tg_ref_C
vg_ref_mps
```

Rollout 中每一步执行：

```text
candidate Tg/vg
        ↓
actuator dynamics + resource accounting
        ↓
preheater predictor step
        ↓
FurnaceFeed from preheater output
        ↓
furnace predictor step
        ↓
stage cost
```

代价函数主要包含：

```text
T_avg tracking
reference band penalty
safety band penalty
dynamic omega target penalty
auxiliary heat cost
auxiliary circulation cost
fan/circulation cost
high-vg cost
control movement cost
terminal T_avg penalty
```

安全恢复区间采用状态依赖权重：当预测 `T_avg` 接近或低于 850 °C 合规下限时，NMPC 会显著降低补热、辅助循环、风机和高 `v_g` 的经济惩罚，同时提高低温安全惩罚。这让控制器在低温扰动后更果断地使用 1100 °C 辅助烟气和最大循环速度；回到安全区后再恢复经济运行权衡。

### 动态含水率目标

`omega_target` 由温度目标、当前预测干基质量流率和扰动估计反算：

```text
omega_target_from_T = inverse_surrogate(T_target_C - disturbance_est_Tavg_C, mdot_d_furnace_kgps)
```

因此水分目标随焚烧炉干基负荷和炉内扰动估计改变，而不是固定在某个旧参考含水率。负向永久扰动会要求更低的出口含水率。

NMPC 还记录安全恢复诊断：

```text
safety_reachable           当前优化轨迹的 terminal T_avg 是否高于安全下限
safety_margin_C            terminal predicted T_avg - T_safe_low_C
omega_max_for_safety       在当前扰动估计和干基负荷下，达到安全下限所允许的最大出口含水率
nmpc_pred_min/max_Tavg_C   预测视界内最低/最高炉温
```

当实测 `T_avg` 低于安全下限时，operator 不再无条件持有旧计划到完整 `reoptimize_s`，而是按 `emergency_reoptimize_interval_s` 触发更短间隔的 best-effort re-optimization。低温恢复时，NMPC 的初始候选还会显式加入 `Tg = Tg_max`、`vg = vg_max` 的全力恢复 seed，避免 SLSQP 在早期迭代里缓慢摸索最大恢复动作。

---

## Executor

`controller/executor/executor.py` 将 `ControlSetpoint` 转换为实际 `ActuatorCommand`。执行器包含：

```text
Tg rate limit
vg rate limit
Tg first-order lag
vg first-order lag
Tg/vg bounds
auxiliary heat accounting
auxiliary circulation accounting
fan/circulation power diagnostic
```

Plant 接收的是 executor 输出的 `ActuatorCommand`，不是 operator 原始参考值。

Executor 还包含安全恢复 guard：当 runtime 判定实测炉温低于合规下限，或当前 NMPC 预测安全不可达且温度仍低于目标时，executor 会把参考值提升到 `1100 °C / 12 m/s` 并标记 `recovery_guard_active`。这是控制安全层的兜底动作，用于避免低温事件中等待经济 NMPC 缓慢加热；其触发比例会写入 telemetry 和 metrics。

---

## Runtime 测试与输出

标准测试入口位于 `runtime/tests/`。

运行单个 case：

```bash
python -m runtime.tests.test_case_steady_hold
python -m runtime.tests.test_case_feed_step_change
python -m runtime.tests.test_case_furnace_temp_temporary_disturbance
python -m runtime.tests.test_case_furnace_temp_permanent_disturbance
python -m runtime.tests.test_case_cold_start
```

运行全部标准 case：

```bash
python -m runtime.tests.run_all_cases
```

每个 case 的输出目录为：

```text
runtime/results/<case_name>/
├── overview.png
├── timeseries.csv
├── metrics.csv
├── control_events.csv
├── preheater_diagnostics.csv
└── cell_snapshot.csv
```

`overview.png` 包含 6 个 panel：

```text
furnace thermal response
preheater outlet / heat-transfer diagnostics
operator reference vs executor command
stack resource / auxiliary heat / fan power
disturbance observer
cell snapshots
```

`metrics.csv` 按 full、tail、event、post_event、recovery 等时间片统计：

```text
Tavg mean/min/max/RMSE
ratio_in_ref_band
ratio_T_ge_compliance_min
omega_out mean/min/max
mdot_furnace_dry_mean_kgps
rd_used_mean
omega_target_from_T_mean
aux heat energy
aux flow mean/max
fan energy
operator fallback ratio
recovery guard ratio
resource and auxiliary resource ratios
command total variation
```

---

## COMSOL 焚烧炉静态代理

`scripts/fit_furnace_static_surrogate.py` 用 COMSOL 批跑 CSV 拟合焚烧炉静态代理。

输入数据目录：

```text
scripts/data/furnace_static_comsol/
```

拟合报告：

```text
scripts/furnace_static_surrogate_fit.json
```

CSV 中温度单位为 K。脚本拟合前将温度输出转换为 °C；温度标准差按温差处理，K 与 °C 数值一致。

静态代理使用二维三阶多项式：

```text
y = f(rd, omega_b)
```

COMSOL 扫描范围：

```text
rd      = 0.5 ~ 3.5
w_b     = 0 ~ 50 %
omega_b = w_b / 100
```

当前拟合误差摘要：

| 输出 | train RMSE | LOO RMSE |
|---|---:|---:|
| `T_avg_C` | 0.763 °C | 1.009 °C |
| `T_stack_C` | 2.223 °C | 3.065 °C |
| `v_stack_mps` | 0.021 m/s | 0.032 m/s |
| `T_surface_min_C` | 3.679 °C | 5.260 °C |
| `T_surface_max_C` | 0.708 °C | 0.942 °C |
| `T_surface_std_C` | 0.784 °C | 1.086 °C |

重新拟合：

```bash
python scripts/fit_furnace_static_surrogate.py
```

---

## 配置重点

常用配置位于 `runtime/simulator.py::SimConfig` 和 `controller/operator/nmpc_operator.py::NMPCConfig`。

| 配置 | 含义 | 默认值 |
|---|---|---:|
| `daily_wet_feed_kg` | 日处理湿垃圾质量 | 20000 kg/day |
| `wet_mass_flow_kgps` | 连续湿垃圾质量流率，派生属性 | `daily_wet_feed_kg / 86400` |
| `wet_mass_flow_override_kgps` | 质量流率扫描 override | `None` |
| `T_target_C` | 主温度目标 | 873 °C |
| `T_compliance_min_C` | 合规温度下限 | 850 °C |
| `omega_ref` | 初始化兼容参考含水率 | 0.3218 |
| `preheater_omega_min` / `OMEGA_MODEL_MIN` | 预热炉代理模型允许的残余湿基含水率下限；当前允许深度干燥但保持大于 0 | 0.05 |
| `nominal_Tg_C` | 初始化/nominal 控制温度 | 800 °C |
| `nominal_vg_mps` | 初始化/nominal 循环速度 | 12 m/s |
| `preheater_warmup_s` | 预热炉库存 warmup 时间 | 6 residence times |
| `nmpc_horizon_s` | NMPC 预测视界 | 600 s |
| `nmpc_rollout_dt_s` | NMPC 内部 rollout 步长 | 5 s |
| `nmpc_maxiter` | SLSQP 最大迭代次数 | 20 |
| `emergency_reoptimize_interval_s` | 安全线以下的紧急重优化间隔 | 20 s |
| `recovery_trigger_margin_C` | 预测温度进入安全恢复区间的提前量 | 20 °C |
| `recovery_guard_exit_C` | recovery guard 退出滞回温度；避免在 850 °C 附近反复开关 | 865 °C |
| `recovery_guard_min_hold_s` | recovery guard 进入后的最小保持时间 | 60 s |
| `recovery_economy_multiplier` | 安全恢复区间内补热/补风/高风速经济代价倍率 | 0.02 |
| `aux_Tg_max_C` | 补热后的有效 Tg 上限 | 1100 °C |
| `compute_latency_mode` | 是否把控制器 wall-clock 求解耗时反馈到 plant 仿真时间；`none` 为理想仿真，`profile` 用实测耗时，`fixed` 用固定延迟 | `none` |
| `fixed_compute_latency_s` | `compute_latency_mode="fixed"` 时每次控制更新的命令应用延迟 | 0 s |

---

## 扩展 backend

### COMSOL backend

`plant/comsol/backend.py` 应实现 `PlantBackend` 语义。COMSOL backend 负责：

```text
ActuatorCommand → COMSOL 边界条件
FeedstockObservation → 入口物料边界
transient solve → PlantSnapshot
```

返回的 `PlantSnapshot` 至少应包含 `FurnaceObservation`。若 COMSOL 可以输出预热炉场变量，可投影成 `PreheaterState`；若只输出出口量，可填 `PreheaterOutput`。

### Hardware backend

`plant/hardware/backend.py` 应把 `ActuatorCommand` 写入 PLC / OPC-UA / Modbus / DAQ / TCP 等接口，并从传感器读取：

```text
T_avg_C
T_stack_C
v_stack_mps
optional preheater outlet measurement
actuator feedback
health/status
```

硬件 backend 通常不能提供完整 20 槽状态。Estimator 会用 controller-side predictor digital twin 维护可用于 NMPC 的 `preheater_state_est`。

---

## 开发约束

1. `plant/python_model/` 与 `controller/predictor/` 保持两套实现，不抽 shared physics core。
2. `controller/` 不 import `plant/python_model/`。
3. `plant/` 不 import `controller/`。
4. `runtime/` 负责装配和实验编排，不承担控制算法或物理公式所有权。
5. `FeedstockObservation.raw` 只用于日志和排查，operator 不依赖 raw 字段。
6. `rd` 是 COMSOL surrogate 坐标和诊断量，工程主变量为 `mdot_d_furnace_kgps`。
7. 补热、补风、辅助循环是允许的运行方式，合规评价优先看 `T_avg ≥ 850 °C`、fallback、stale plan 和命令硬削弱。
8. 后台 async NMPC 可以接受 SLSQP `success=False` 但代价有限、动作有界的 best-effort plan；`feasible` 表示优化器收敛状态，不等同于 plan 是否可执行。
9. recovery guard 是 executor 安全层，采用滞回和最小保持时间，避免温度在安全线附近抖动时反复开关。

---

## 快速检查

编译检查：

```bash
python -m compileall domain plant controller runtime scripts
```

预热炉步长诊断：

```bash
python -m runtime.tests.test_preheater_timestep_invariance
```

控制延迟诊断：

```bash
python -m runtime.tests.test_control_latency
```

同步 SLSQP 求解耗时诊断：

```bash
python -m runtime.tests.test_slsqp_solve_latency
```

默认参数使用短视界和低迭代数，便于快速确认 SLSQP 分段耗时；如需测试当前完整 NMPC 配置，可加 `--full-config`。

计算耗时对闭环效果的影响诊断：

```bash
python -m runtime.tests.test_compute_latency_effect --delays 0,2,5,10,30
```

默认 runtime 是理想仿真：控制器求解消耗的真实 wall-clock 时间不会推进 plant。打开 `compute_latency_mode="profile"` 或 `"fixed"` 后，runtime 会在新命令生效前让 plant 继续按旧命令运行相应的模拟时间，并在 CSV/metrics 中输出 `operator_compute_wall_s`、`simulated_compute_latency_s`、`command_apply_delay_s`、`async_*plan*` 等诊断字段。

steady hold：

```bash
python -m runtime.tests.test_case_steady_hold
```

