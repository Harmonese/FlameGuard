# Changelog

## [v0.1.0] - 2026-04-25

This version is drafted as a release to archive the first version of FlameGuard that uses Smith-PID as controlling method and the proxy model is not yet complete and decoupled from the controlling algorithms.
This version also uses the controller -> governor -> transcriber A -> optimizer -> transcriber B -> controller closed loop controlling strategy whose stages are mostly deprecated in newer versions.
Known issues include:

Reacting time is too slow and low frequency hunting are hard to eliminate
The proxy model lacks necessary dynamic functions that makes the outcome unconvincing

## [v0.2.0] - 2026-05-01

FlameGuard 从早期的“规则优化 + PID 修正”原型，升级为面向仿真验证、运行时诊断与工程接入的模块化 NMPC 控制系统。更新重点包括架构重构、COMSOL 炉膛代理模型、低阶 plant/backend 抽象、异步 NMPC、安全恢复保护、计算延迟仿真与完整文档体系。

### Highlights

- 重构项目架构：新增 `domain/`、`plant/`、`controller/`、`runtime/`、`scripts/`，将领域类型、装置模型、控制器、执行器、运行时仿真与数据脚本分层管理。
- 引入 COMSOL 静态炉膛代理：用湿基含水率 `omega` 与干基比例 `rd` 的双输入代理模型替代旧版一维线性稳态模型，并在动态炉膛模型外包两阶滞后与死区时间。
- 升级控制核心：新增非线性模型预测控制（NMPC）与异步 NMPC 执行路径，支持后台求解、陈旧计划丢弃、best-effort 计划接收与运行时状态回传。
- 强化安全恢复：将辅助烟气温度上限提升至 `1100 degC`，新增低温合规恢复模式、执行器级 recovery guard、恢复滞回与最小保持时间，提升炉温跌破 850 degC 时的恢复速度。
- 增强仿真诊断：新增 compute-latency aware runtime，可模拟控制器求解耗时对闭环系统的影响，并输出更完整的 metrics、control events、preheater diagnostics 与 cell snapshots。
- 完善交付材料：新增 MIT License、项目 README、COMSOL 仿真设计手册、控制器技术手册、预热炉设计手册与 FlameGuardWeb 用户手册。

### Breaking Changes

- 旧版根目录模块已移除或迁移：`cleanser/`、`governor/`、`optimizer/`、`proxy_model/`、`transcriber/`、`tests/` 不再作为主运行路径使用。
- 旧版控制链路中的 `omega_req`、`omega_tar`、动态 governor band、lookup-table optimizer 等概念被新的 `StateEstimate`、`OperatorContext`、`MPCDecision`、`ActuatorCommand` 与 runtime telemetry 取代。
- 旧版 `control_types.py` 被拆分并迁移至 `domain/types.py` 与 `domain/interfaces.py`，外部集成应改用新的 domain 类型和接口协议。
- 旧版测试入口迁移为 `runtime.tests.*`；结果默认输出目录从 `tests/results/` 调整为 `runtime/results/`。
- 生成的运行结果不再作为源码交付内容，`runtime/results/` 已加入 `.gitignore`。

### Added

- 新增 `domain/interfaces.py`，定义 `PlantBackend`、`StateEstimator`、`Operator`、`Executor` 与 `FeedstockPreviewProvider` 协议。
- 新增 `plant/factory.py`，统一创建 Python plant backend，并封装预热炉、炉膛、资源边界与扰动调度。
- 新增 `controller/factory.py`，统一装配预测器、状态估计器、NMPC operator 与 executor。
- 新增 `controller/estimator/`，提供状态估计与炉膛扰动观测能力。
- 新增 `controller/operator/nmpc_operator.py`，实现主 NMPC 控制器、SLSQP 求解、滚动预测、扰动感知含水率目标与安全可达性诊断。
- 新增 `controller/operator/async_nmpc_operator.py`，支持后台 NMPC 求解、计划接收 / 丢弃统计、过期计划保护与异步状态遥测。
- 新增 `controller/executor/executor.py`，将控制 setpoint 翻译为执行器命令，并加入低温安全恢复 guard。
- 新增 `plant/python_model/`，提供可独立运行的预热炉、炉膛、资源与执行器低阶模型。
- 新增 COMSOL 静态代理拟合脚本 `scripts/fit_furnace_static_surrogate.py` 与拟合结果 `scripts/furnace_static_surrogate_fit.json`。
- 新增 compute latency 仿真配置：`compute_latency_mode`、`fixed_compute_latency_s`、`compute_latency_scale`、`max_simulated_compute_latency_s`、`compute_latency_step_s`。
- 新增运行时遥测字段：operator wall time、simulated compute latency、command apply delay、async job submit / accept / discard、stale-plan ratio、safety reachability、safety margin、predicted furnace temperature range 等。
- 新增测试与验证入口：`runtime.tests.test_preheater_timestep_invariance`、`runtime.tests.test_compute_latency_effect`、`runtime.tests.test_control_latency`、`runtime.tests.test_slsqp_solve_latency`。

### Changed

- 炉膛稳态模型从旧版单变量线性代理升级为 COMSOL degree-3 多项式代理，输入包含湿基含水率与干基比例。
- 预热炉模型支持更完整的热量、质量、蒸发与气路诊断，并输出 `PreheaterDiagnostics`。
- 默认预热炉残余湿基含水率下限从 `0.20` 降至 `0.05`，允许高温辅助烟气条件下进行更深度干化，同时保留数值下界。
- NMPC 的含水率目标现在会根据炉膛扰动估计进行反推；当存在负向炉温扰动时，控制器会主动要求更低出口含水率以恢复安全炉温。
- NMPC 在接近 850 degC 合规下限时会进入 recovery mode：降低经济性惩罚，提高安全惩罚，并加入最大恢复种子以加快 SLSQP 找到有效恢复计划。
- 异步 NMPC 不再简单丢弃 `success=False` 但有限、有界的 SLSQP 结果；迭代上限导致的 best-effort 计划可被接收，同时保留 optimizer success telemetry。
- 运行时指标改用真实记录的 sample duration 计算，避免 compute latency 推进仿真时间后仍按 nominal `dt` 统计造成失真。
- `.gitignore` 新增 `runtime/results/`，减少生成物污染源码树。

### Fixed

- 修复 runtime 中 feed composition 重复追加的问题。
- 修复 command apply delay 遥测重复追加的问题。
- 修复异步计划陈旧状态下可能继续使用过期结果的问题。
- 修复低温恢复时重优化间隔过长的问题；当炉温低于合规下限时，会使用更短的 emergency reoptimize interval。
- 修复旧版 20% 含水率硬下限导致永久扰动工况下安全可达性不足的问题。

### Performance & Validation

- 新增轻量 `step_fast()` / `step_output()` 预测路径，NMPC 内部滚动不再每步构造完整 20-cell `PreheaterState`。
- 用手动结构化 clone 替代 predictor preheater 的 `deepcopy()`，减少 SLSQP 目标函数评估开销。
- 将预热炉水分同步从逐 cell 能量循环中移出，减少重复计算。
- 缓存 COMSOL 代理的反向含水率目标，避免 SLSQP 有限差分阶段重复 40 步二分。
- 预计算 rollout feed、步长、elapsed time 与 block index，减少每次目标函数调用的重复准备工作。
- 随包场景结果显示，安全恢复时间在多个扰动场景中明显缩短：feed step change 约 `1800s -> 900s`，temporary disturbance 约 `2400s -> 1200s`，permanent disturbance 约 `2434s -> 1055s`，cold start 约 `10200s -> 5100s`；扰动场景 overshoot crossings 也显著下降。

### Documentation

- 新增 / 重写项目 README，面向小型垃圾焚烧设施场景介绍系统背景、技术路线、控制方案、前端平台与效益评估。
- 新增 COMSOL 仿真实验设计手册，记录炉膛与预热炉仿真设计、参数扫描、网格与结果图。
- 新增控制器技术手册，记录 NMPC、预热炉预测、炉膛代理、安全恢复和运行时测试结果。
- 新增预热炉设计手册与 FlameGuardWeb 用户手册。
- 新增 `CHANGELOG.md` 与 `LICENSE`。

### Migration Notes

- 使用新的测试入口：

```bash
python3.14 -m runtime.tests.test_preheater_timestep_invariance
python3.14 -m runtime.tests.run_all_cases
```

- 旧版基于 `optimizer/optimizer_lookup_table.csv`、`governor/governor.py` 或 `controller/controller.py` 的集成，需要迁移到 `controller.factory.make_operator()`、`controller.factory.make_state_estimator()` 与 `controller.factory.make_executor()`。
- 外部 plant 或硬件接入应实现 `domain.interfaces.PlantBackend`，再通过 runtime 与 controller 层对接。
- 仿真输出默认写入 `runtime/results/`，该目录默认不纳入版本控制。

## [v0.2.1] - 2026-05-01

### Updates
- 更新了 .gitignore
- 删除了 `/docs` 路径下的中间文件和冗余文件