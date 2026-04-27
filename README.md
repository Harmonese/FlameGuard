# 稳燃宝（FlameGuard）

稳燃宝（FlameGuard）是一套面向垃圾焚烧预热脱水过程的分层闭环控制系统。系统利用焚烧炉烟气余热对高含水率厨余垃圾进行预热与脱水，使进入主焚烧炉的垃圾含水率维持在适合长期稳定燃烧的区间内，同时尽量降低补热、循环风机和过度干燥带来的运行代价。

当前工程按三层组织：

```text
model/       数学建模与代理模型层
controller/  控制算法、观测器与优化器层
runtime/     仿真运行、测试、绘图与指标统计层
legacy/      历史实现与实验脚本归档
```

其中，`model/` 描述被控对象和物理代理模型，`controller/` 根据模型接口完成预测控制与优化决策，`runtime/` 负责把模型、控制器和测试场景连接成可运行闭环。该结构的核心原则是：**代理模型层和控制算法层解耦**。当前默认使用 Python 低阶代理模型；如果后续接入 COMSOL 联合仿真或实体设备，只要保持相同的数据接口和步进语义，控制层可以继续沿用。

---

## 一、数学建模与 `model/` 层

`model/` 层负责定义系统中的物理对象、代理模型、热工公式、数据结构和可替换模型接口。控制器不直接拥有这些物理公式，而是通过模型层提供的状态、输出和步进接口进行预测与优化。

当前 `model/` 目录主要包括：

```text
model/
├── control_types.py          闭环数据结构：观测、命令、预热炉状态、MPC 决策
├── model_types.py            工程常数、物料等效性质、资源边界、静态优化请求结构
├── thermal_core.py           共享热工公式：烟气密度、质量流量、供热、需热、稳态代理
├── interfaces.py             可替换模型接口协议
├── material_model.py         六类垃圾组分到等效物料性质的映射
├── furnace_dynamic.py        焚烧炉静态/动态代理模型
├── preheater_forward_model.py 20 槽预热炉分布式 forward model
├── resource_model.py         动态烟气资源与补热/循环诊断模型
├── actuator_dynamic.py       rate limit + 一阶惯性执行器模型
├── feed_preview.py           feed preview 供应器
├── slot_drying_model.py      单槽位干燥模型，用于局部逆模型和诊断
└── proxy_data/               COMSOL 回归与瞬态数据
```

### 1. 典型垃圾组分与等效物料模型

系统考虑六类典型厨余/生活垃圾组分：

| 编号 | 类别 | 初始含水率 \(\omega_i\) | 175°C 下达到 20% 含水率时间 \(t_{ref,i}\) / min | 温度敏感性 \(s_i\) / min·°C⁻¹ |
|---|---|---:|---:|---:|
| 1 | 菜叶 | 0.948 | 12.1 | -0.132 |
| 2 | 西瓜皮 | 0.948 | 17.7 | -0.251 |
| 3 | 橙子皮 | 0.817 | 15.3 | -0.189 |
| 4 | 肉 | 0.442 | 11.5 | -0.216 |
| 5 | 杂项混合 | 0.773 | 16.3 | -0.210 |
| 6 | 米饭 | 0.611 | 15.8 | -0.243 |

输入组分向量为：

$$
x=[x_1,x_2,x_3,x_4,x_5,x_6], \qquad \sum_{i=1}^6 x_i=1,\quad x_i\ge 0
$$

混合垃圾的等效初始含水率、参考干燥时间和温度敏感性定义为线性组合：

$$
\omega_0(x)=0.948x_1+0.948x_2+0.817x_3+0.442x_4+0.773x_5+0.611x_6
$$

$$
t_{ref}(x)=12.1x_1+17.7x_2+15.3x_3+11.5x_4+16.3x_5+15.8x_6
$$

$$
s(x)=-0.132x_1-0.251x_2-0.189x_3-0.216x_4-0.210x_5-0.243x_6
$$

混合垃圾在等效平均干燥温度 \(T_m\) 下达到 20% 含水率的参考动力学时间为：

$$
\tau_{20}(T_m,x)=t_{ref}(x)+s(x)(T_m-175)
$$

该模型由 `model/material_model.py` 实现，核心输出为：

```python
EquivalentProperties(
    omega0=...,              # 初始湿基含水率
    tref_min=...,            # 175°C 下达到 20% 的参考时间
    slope_min_per_c=...,     # 温度敏感性斜率
)
```

等效比热采用湿基加权：

$$
c_{eq}(x)=(1-\omega_0)c_s+\omega_0 c_w
$$

其中：

$$
c_s=1.70\ \text{kJ/(kg·K)},\qquad c_w=4.1844\ \text{kJ/(kg·K)}
$$

### 2. 焚烧炉稳态代理模型

焚烧炉代理模型把进入主焚烧炉的垃圾湿基含水率映射为焚烧炉稳态燃烧状态。模型中使用两种含水率记号：

- \(w\)：百分数形式，单位为 `%`；
- \(\omega=w/100\)：湿基小数形式。

COMSOL 稳态回归得到的焚烧炉稳态代理关系为：

$$
T_{stack}(w)=-14.412237w+1423.472316
$$

$$
v_{stack}(w)=-0.215310w+25.332842
$$

$$
T_{avg}(w)=-13.109632w+1294.871365
$$

$$
\sigma(w)=-0.189997w+18.331239
$$

$$
T_{min}(w)=-13.422320w+1330.692379
$$

$$
T_{max}(w)=-16.072025w+1589.019616
$$

其中：

- \(T_{avg}\)：燃烧核心区域平均温度，是控制系统的主要被控输出；
- \(T_{stack}\)：烟囱出口平均温度，用于资源诊断和烟气可用温度估计；
- \(v_{stack}\)：烟囱出口平均速度，用于自然烟气质量流量诊断；
- \(T_{min},T_{max},\sigma\)：用于稳态安全边界与均匀性评估。

当前代码中，这些稳态关系由 `model/thermal_core.py` 与 `model/furnace_dynamic.py` 提供。

### 3. 焚烧炉动态代理模型

焚烧炉动态模型描述进入主焚烧炉的垃圾含水率变化如何经过延迟与惯性反映到三路可测输出：

$$
y_f(t)=
\begin{bmatrix}
T_{avg}(t)\\
T_{stack}(t)\\
v_{stack}(t)
\end{bmatrix}
$$

模型采用统一动态骨架：

$$
G_f(s)=\frac{e^{-\tau_f s}}{(\tau_{f1}s+1)(\tau_{f2}s+1)}
\begin{bmatrix}
K_{avg}\\
K_{stack,T}\\
K_{stack,v}
\end{bmatrix}
$$

其中：

$$
\tau_f=5\ \text{s},\qquad \tau_{f1}=0.223\ \text{s},\qquad \tau_{f2}=75.412\ \text{s}
$$

$$
K_{avg}=-13.109632,\quad K_{stack,T}=-14.412237,\quad K_{stack,v}=-0.215310
$$

`model/furnace_dynamic.py` 将该连续动态离散为：

1. 以秒为物理单位的纯延迟；
2. 快一阶惯性；
3. 慢一阶惯性；
4. 可选外部加性扰动。

在 plant 仿真中，焚烧炉通常以 \(\Delta t=0.1\) s 推进；在 NMPC 预测中，模型允许更粗的预测步长，但延迟仍按物理秒数处理，避免把 5 s 延迟错误放大为多个预测大步。

### 4. 焚烧炉稳定工作区间

工程约束包括：

1. 燃烧面最低温度不低于 850°C；
2. 平均炉温不低于 850°C；
3. 平均炉温不高于 1100°C；
4. 燃烧面最高温度不高于 1100°C。

由：

$$
T_{min}(w)\ge 850
$$

可得：

$$
w\le 35.81
$$

由：

$$
T_{avg}(w)\ge 850
$$

可得：

$$
w\le 33.93
$$

由：

$$
T_{avg}(w)\le 1100
$$

可得：

$$
w\ge 14.86
$$

由：

$$
T_{max}(w)\le 1100
$$

可得：

$$
w\ge 30.43
$$

因此稳态参考含水率区间为：

$$
30.43\%\le w\le 33.93\%
$$

对应湿基小数：

$$
0.30427\le \omega\le 0.33935
$$

稳态工作点取该区间中点：

$$
w_{ref}=32.18\%,\qquad \omega_{ref}=0.3218
$$

对应平均炉温目标：

$$
T_{set}=T_{avg}(32.18)=872.99^\circ\text{C}
$$

参考平均炉温区间为：

$$
850.00^\circ\text{C}\le T_{avg}\le 895.99^\circ\text{C}
$$

### 5. 预热炉结构与处理规模

预热炉设备原型为卧式间接回转滚筒预热脱水炉，设置外夹套与内置导烟管两级传热面，烟气与垃圾不直接接触。主要几何参数为：

| 参数 | 含义 | 数值 |
|---|---|---:|
| \(D\) | 主筒体内径 | 1.2 m |
| \(L\) | 有效长度 | 3.2 m |
| \(\phi\) | 工作充填率 | 0.14 |
| \(d_t\) | 内置导烟管外径 | 0.168 m |
| \(N_t\) | 导烟管数量 | 6 |
| \(r_d\) | 预热炉主烟道半径 | 0.4 m |
| \(r_s\) | 焚烧炉烟囱半径 | 0.3 m |

主烟道截面积：

$$
A_d=\pi r_d^2=\pi\times0.4^2=0.503\ \text{m}^2
$$

烟囱截面积：

$$
A_s=\pi r_s^2=\pi\times0.3^2=0.2827\ \text{m}^2
$$

有效传热面积：

$$
A=\pi DL+6\pi d_tL=22.20\ \text{m}^2
$$

日处理量为 20 t/d，折算连续湿垃圾质量流量：

$$
\dot m_w=\frac{20000}{24\times3600}=0.2315\ \text{kg/s}
$$

湿垃圾体积密度取：

$$
\rho_b=450\ \text{kg/m}^3
$$

炉内平均滞留量：

$$
M_h=\rho_b\phi\frac{\pi D^2}{4}L=228.0\ \text{kg}
$$

平均停留时间：

$$
\tau_r=\frac{M_h}{\dot m_w}=985.0\ \text{s}=16.42\ \text{min}
$$

### 6. 预热炉 20 槽分布式 forward model

`model/preheater_forward_model.py` 将预热炉沿轴向离散为 20 个槽位。每个槽位携带：

```text
omega              当前湿基含水率
T_solid_C          当前固体/垃圾等效温度
omega0             该物料初始等效含水率
tref_min           该物料参考干燥时间
slope_min_per_c    该物料温度敏感性
residence_left_s   该槽位剩余停留时间
```

第 \(i\) 个槽位状态记为：

$$
x_i(t)=\{\omega_i(t),T_{s,i}(t),\omega_{0,i},t_{ref,i},s_i,\tau_{rem,i}\}
$$

#### 6.1 前端 feed delay

入口组分测量点到预热炉入口存在约 5 s 前端延迟：

$$
x_{pre,in}(t)=x_{in}(t-5)
$$

当前实现使用真实时间队列 `feed_delay_buffer`，而不是用轴向 cell residence bucket 近似 5 s 延迟。每次 `step()` 时，模型从队列中取 \(t-\tau_x\) 附近的等效物料性质，并在必要时进行线性插值。

#### 6.2 轴向输送与数值混合

每个槽位时间尺度为：

$$
\tau_{cell}=\frac{\tau_r}{N}=\frac{985}{20}=49.25\ \text{s}
$$

令：

$$
\gamma=\min\left(\frac{\Delta t}{\tau_{cell}},1\right)
$$

下游槽位按上游槽位混入推进：

$$
z_i^{k+1}=(1-\gamma)z_i^k+\gamma z_{i-1}^k
$$

入口槽位则向延迟后的入口物料性质靠近。这里 \(z_i\) 代表 \(\omega_i,T_{s,i},\omega_{0,i},t_{ref,i},s_i\) 等状态量。

#### 6.3 烟气沿程冷却

烟气入口温度为 \(T_{g,in}\)，循环/换热侧速度为 \(v_g\)。烟气从入口依次经过 20 个槽位。第 \(i\) 个槽位入口烟气温度为 \(T_{g,i}\)，传热功率为：

$$
Q_i=U(v_g)A_i\max(T_{g,i}-T_{s,i},0)
$$

其中：

$$
U(v_g)=18.97+20.09v_g^{0.65}
$$

$$
A_i=\frac{A}{N}
$$

烟气质量流量按预热炉换热侧循环速度估算：

$$
\dot m_g=\rho_g(T_{g,in})A_dv_g
$$

烟气沿程降温：

$$
T_{g,i+1}=T_{g,i}-\frac{Q_i}{\dot m_gc_{pg}}
$$

为避免非物理反转，代码中限制：

$$
T_{g,i+1}\ge T_{s,i}+10^{-3}
$$

模型输出 `Tg_profile_C`，包含入口到出口的烟气温度剖面。

#### 6.4 固体升温与脱水

槽位接收热量：

$$
E_i=Q_i\Delta t
$$

固体温度低于 100°C 时，热量优先用于显热升温：

$$
E_{sens}=M_i c_{eq}(100-T_{s,i})
$$

超过蒸发温度后，模型将高温热量的主要部分用于蒸发，当前比例为 0.85：

$$
E_{evap}=0.85E_i
$$

蒸发水量：

$$
\Delta m_{evap}=\frac{E_{evap}}{\lambda}
$$

对应含水率下降量按湿基近似折算，并同时受实验动力学限制：

$$
\Delta\omega_i\le \frac{(\omega_i-0.20)\Delta t}{\tau_{20}(T_{s,i})}
$$

最终含水率限制在模型适用范围：

$$
0.20\le \omega_i\le0.98
$$

固体温度限制为：

$$
20^\circ\text{C}\le T_{s,i}\le250^\circ\text{C}
$$

#### 6.5 模型输出

`PreheaterForwardModel.step(feed,Tg,vg,dt)` 返回 `PreheaterState`：

```text
time_s            当前时间
cells             20 个 PreheaterCellState
omega_out         出口槽位湿基含水率
T_solid_out_C     出口槽位固体温度
Tg_profile_C      烟气沿程温度剖面
```

该输出直接进入焚烧炉动态模型。

### 7. 烟气资源、补热与循环诊断模型

`model/resource_model.py` 将焚烧炉观测量转化为自然烟气资源边界。给定：

```text
FurnaceObservation(time_s,T_avg_C,T_stack_C,v_stack_mps)
```

自然可用温度和速度定义为：

$$
T_{stack,avail}=\max(T_{min},T_{stack}-\Delta T_{loss})
$$

$$
v_{stack,avail}=\max(v_{min},\eta_v v_{stack})
$$

自然烟气质量流量诊断为：

$$
\dot m_{stack,avail}=\rho_g(T_{stack,avail})A_sv_{stack,avail}
$$

资源模型同时给出有效控制边界：

$$
T_{g,cap}^{eff}=T_{aux,max}
$$

$$
v_{g,cap}^{eff}=12\ \text{m/s}
$$

当前控制解释中，\(v_g\) 表示预热炉换热侧循环速度或局部流速，不再被自然烟囱一次通过质量流量硬裁剪。自然烟气质量流量不足时，系统记录循环/补偿需求：

$$
\dot m_{aux}=\max(\dot m_{preheater}-\dot m_{stack,avail},0)
$$

其中：

$$
\dot m_{preheater}=\rho_g(T_g)A_dv_g
$$

当自然烟气温度低于目标入口温度时，补热功率为：

$$
Q_{aux}=\dot m_{preheater}c_{pg}\max(T_g-T_{stack,avail},0)
$$

因此，资源模型区分三类诊断：

1. 自然烟气温度是否足够；
2. 自然烟气一次通过质量流量与换热侧循环流量是否匹配；
3. 补热功率、循环需求与风机经济代价。

当前风机/循环代价采用三次方诊断模型：

$$
P_{fan}=P_{fan,ref}\left(\frac{v_g}{v_{g,max}}\right)^3
$$

其中 \(P_{fan,ref}=18\) kW，\(v_{g,max}=12\) m/s。

### 8. 执行器动态模型

`model/actuator_dynamic.py` 描述控制器参考设定 \(T_{g,ref},v_{g,ref}\) 到实际执行命令 \(T_{g,cmd},v_{g,cmd}\) 的过渡过程。

执行器包含两部分：

1. 变化率限制；
2. 一阶惯性。

温度通道：

$$
G_T(s)=\frac{1}{3s+1}
$$

速度通道：

$$
G_v(s)=\frac{1}{s+1}
$$

离散形式为：

$$
u_{act}^{k+1}=u_{act}^{k}+\alpha(u_{rl}^{k}-u_{act}^{k})
$$

$$
\alpha=1-e^{-\Delta t/\tau}
$$

其中 \(u_{rl}\) 为经过 rate limit 后的中间状态。

默认速率约束为：

$$
\left|\frac{dT_g}{dt}\right|\le20\ ^\circ\text{C/s}
$$

$$
\left|\frac{dv_g}{dt}\right|\le0.4\ \text{m/s}^2
$$

执行器输出 `ActuatorCommand`，包括：

```text
Tg_cmd_C
vg_cmd_mps
Q_heat_deficit_kW
resource_limited
mdot_preheater_kgps
mdot_stack_cap_kgps
mdot_aux_flow_kgps
fan_circulation_power_kW
```

### 9. Feed preview 模型

`model/feed_preview.py` 提供预测视界内的入口垃圾组分序列。

当前实现包括：

```text
ConstantFeedPreview       未来 feed 组分保持当前值
KnownScheduleFeedPreview  根据测试场景 schedule 生成未来 feed
```

接口语义为：

```python
get(time_s, horizon_s, dt_s) -> list[FeedObservation]
```

控制器在 NMPC rollout 中使用该序列，而不是默认未来 feed 永远不变。

---

## 二、控制算法与 `controller/` 层

`controller/` 层负责根据当前观测、模型状态和运行策略求解控制输入。该层只调用模型接口，不拥有物理对象本身。

当前目录结构为：

```text
controller/
├── nmpc_controller.py        主控制器：control-blocking nonlinear MPC
├── mpc_controller.py         fallback lookup-assisted MPC
├── estimators/               扰动观测器
└── optimizer/                NMPC 辅助优化器、context lookup、legacy static optimizer 兼容入口
```

### 1. 闭环控制目标

控制目标是通过调节预热炉烟气入口温度和循环速度：

$$
u(t)=\begin{bmatrix}T_g(t)\\v_g(t)\end{bmatrix}
$$

影响预热炉出口垃圾含水率：

$$
\omega_{out}(t)
$$

再经焚烧炉动态模型影响：

$$
T_{avg}(t),\quad T_{stack}(t),\quad v_{stack}(t)
$$

主要控制目标为：

1. 使 \(T_{avg}\) 接近 \(T_{set}=872.99^\circ\)C；
2. 保持 \(T_{avg}\) 在参考带 \(850\\sim895.99^\circ\)C 内；
3. 避免触碰安全带 \(850\\sim1100^\circ\)C；
4. 减少过高 \(T_g\)、过高 \(v_g\)、补热、循环风机和剧烈动作带来的经济代价；
5. 在湿料阶跃、炉温扰动和冷启动场景中保持恢复能力。

### 2. 扰动观测器

`controller/estimators/furnace_disturbance_observer.py` 实现残差型扰动观测器。仿真中存在真实扰动 \(d(t)\)，但控制器不直接读取测试脚本中的真实扰动，而是通过观测残差估计：

$$
\hat d(k)=(1-\alpha)\hat d(k-1)+\alpha(y_{meas}(k)-y_{nom}(k))
$$

其中：

- \(y_{meas}\)：带扰动 plant 输出；
- \(y_{nom}\)：并行无扰动 furnace twin 输出；
- \(\alpha\)：低通系数，默认 0.05。

估计结果包含：

```text
disturbance_est_Tavg_C
disturbance_est_Tstack_C
disturbance_est_vstack_mps
```

NMPC rollout 使用估计扰动，而不是测试场景中的真实扰动 schedule。

### 3. NMPC 控制器

主控制器为 `controller/nmpc_controller.py` 中的 `NonlinearMPCController`。它采用 control-blocking nonlinear MPC：

- 预测视界：\(H=600\) s；
- 决策网格：\(\Delta t_{pred}=20\) s；
- 内部 rollout 子步：\(\Delta t_{rollout}=5\) s；
- 控制块边界：\(0,120,300,480,600\) s；
- 优化变量：4 个控制块的 \(T_g,v_g\)。

决策向量为：

$$
z=[T_{g,1},v_{g,1},T_{g,2},v_{g,2},T_{g,3},v_{g,3},T_{g,4},v_{g,4}]^T
$$

每个控制块内，参考控制量保持常值。NMPC 每次优化一段未来控制序列，但每次闭环只执行当前时刻对应的首段控制，并在后续时刻滚动更新。

### 4. NMPC 预测流程

对每个候选控制序列 \(z\)，控制器执行以下预测：

```text
复制当前 preheater model
复制当前 furnace model
复制当前 actuator model
生成 feed preview
for k in prediction horizon:
    读取当前 block 的 Tg_ref, vg_ref
    根据预测 furnace output 计算 resource state
    actuator dynamic 推出 Tg_cmd, vg_cmd
    preheater forward model 推进 20 槽状态
    furnace dynamic model 推进 T_avg/T_stack/v_stack
    累加温度误差、安全惩罚、参考带惩罚、含水率偏差、能耗和风机代价
```

该过程直接使用完整 20 槽预热炉 forward model，不再把单个代表槽位作为主决策对象。

### 5. NMPC 目标函数

NMPC 代价函数由以下部分组成：

#### 5.1 平均炉温跟踪项

$$
J_T=\sum_k q_T(T_{avg,k}-T_{set})^2
$$

#### 5.2 参考带惩罚

$$
J_{ref}=\sum_k q_{ref}\left[\max(0,T_{avg,k}-T_{ref,high})^2+\max(0,T_{ref,low}-T_{avg,k})^2\right]
$$

#### 5.3 安全带惩罚

$$
J_{safe}=\sum_k q_{safe}\left[\max(0,T_{avg,k}-T_{safe,high})^2+\max(0,T_{safe,low}-T_{avg,k})^2\right]
$$

#### 5.4 出口含水率辅助项

$$
J_\omega=\sum_k q_\omega(\omega_{out,k}-\omega_{ref})^2
$$

#### 5.5 热功率和补热经济项

$$
J_E=\sum_k r_E P(T_{g,k},v_{g,k})
$$

$$
J_{aux}=\sum_k r_{aux}Q_{aux,k}^2
$$

#### 5.6 高流速与风机/循环代价

为了保留高 \(v_g\) 的应急能力，同时避免参考带内长期高流速运行，引入经济速度与高流速惩罚：

$$
J_{v,eco}=\sum_k r_{eco}\max(v_{g,k}-v_{eco},0)^2
$$

$$
J_{v,high}=\sum_k r_{high}\max(v_{g,k}-v_{high},0)^2
$$

其中默认：

$$
v_{eco}=8.0\ \text{m/s},\qquad v_{high}=8.5\ \text{m/s}
$$

风机/循环功率惩罚为：

$$
J_{fan}=\sum_k r_{fan}P_{fan,k}
$$

参考带内经济权重会放大，使系统在温度已经合格时更倾向于降低 \(v_g\)、补热和循环代价。

#### 5.7 动作变化惩罚

$$
J_{\Delta u}=\sum_j r_{\Delta T}(T_{g,j}-T_{g,j-1})^2+r_{\Delta v}(v_{g,j}-v_{g,j-1})^2
$$

### 6. 优化器与 fallback

NMPC 使用 `scipy.optimize.minimize(method="SLSQP")` 求解控制块序列。优化边界为：

$$
100\le T_g\le930
$$

$$
3\le v_g\le12
$$

优化初值池包括：

1. 上一轮控制序列平移；
2. nominal 控制 \(800^\circ\)C / \(12\) m/s；
3. 上一时刻执行命令；
4. 低温时的高温高流速恢复初值；
5. 高温时的低温低流速降温初值。

当 SLSQP 不可用或求解失败时，系统可调用 `controller/mpc_controller.py` 中的 fallback lookup-assisted MPC。fallback 通过动态查表或在线逆模型给出候选控制，并用完整 forward model 进行评分。

### 7. Context-aware inverse optimizer

`controller/optimizer/` 中保留 context-aware inverse optimizer 和 lookup table 工具。其作用是：

1. 作为 NMPC 初值或 fallback 候选来源；
2. 辅助解释某一模型状态下达到目标含水率所需的入口工况；
3. 支持后续离线制表。

该工具评估控制量时使用完整预热炉 forward rollout，因此能够考虑：

```text
20 槽库存状态
烟气沿程冷却
上游槽位吸热
feed preview
资源边界
```

主闭环决策仍由 `NonlinearMPCController` 完成。

---

## 三、测试流程与 `runtime/` 层

`runtime/` 层负责将模型、控制器和测试场景组合成完整仿真闭环。当前目录为：

```text
runtime/
├── simulator.py       主仿真循环、history、metrics、artifact 保存
├── execution_adapter.py 控制设定到执行器命令的适配
├── telemetry.py       指标与 telemetry 导出入口
├── plotting.py        绘图入口
├── results/           运行后生成的结果目录
└── tests/             场景测试入口
```

### 1. 闭环仿真主流程

`runtime/simulator.py` 的 `run_case()` 是主要仿真入口。一个仿真步内执行：

```text
1. 根据 scenario 读取当前 feed composition 和真实扰动；
2. PreheaterForwardModel 使用上一时刻 ActuatorCommand 推进；
3. FurnaceDyn 根据 omega_out 推进 T_avg/T_stack/v_stack；
4. 并行 disturbance-free furnace twin 生成 nominal observation；
5. FurnaceDisturbanceObserver 更新扰动估计；
6. ResourceModel 根据当前观测生成自然资源与有效资源边界；
7. NMPC 周期性重优化 Tg/vg 控制序列；
8. ExecutionAdapter / TranscriberB 调用 ActuatorDynamic 生成 ActuatorCommand；
9. 记录 telemetry、控制源、资源诊断、补热、循环、feed 和扰动信息。
```

仿真使用三个主要时间尺度：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `dt_meas_s` | 0.1 s | plant 动态积分与测量步长 |
| `dt_opt_s` | 2.0 s | 控制设定更新检查步长 |
| `mpc_dt_s` | 20.0 s | NMPC 决策网格 |
| `nmpc_rollout_dt_s` | 5.0 s | NMPC 内部 forward rollout 子步 |
| `nmpc_reoptimize_s` | 60.0 s | NMPC 重优化周期 |

### 2. 执行适配层

`runtime/execution_adapter.py` 中的 `TranscriberB` 作为执行适配器存在。它接收 `ControlSetpoint`：

```text
Tg_ref_C
vg_ref_mps
T_stack_available_C
v_stack_available_mps
mdot_stack_cap_kgps
```

然后调用 `ActuatorDynamic.step()`，输出 `ActuatorCommand`。该命令是 plant 真实使用的输入。

### 3. 测试场景

测试入口位于：

```text
runtime/tests/
```

当前主要场景包括：

| 文件 | 场景 |
|---|---|
| `test_case_steady_hold.py` | 无扰动长时间稳态保持 |
| `test_case_feed_step_change.py` | 入口垃圾变湿阶跃 |
| `test_case_furnace_temp_temporary_disturbance.py` | 临时炉温扰动 |
| `test_case_furnace_temp_permanent_disturbance.py` | 永久炉温扰动 |
| `test_case_cold_start.py` | 低温初始状态恢复 |
| `test_preheater_timestep_invariance.py` | 预热炉模型时间步长一致性诊断 |

运行单个时间步一致性测试：

```bash
python3.14 -m runtime.tests.test_preheater_timestep_invariance
```

运行完整测试套件：

```bash
python3.14 -m runtime.tests.run_all_cases
```

### 4. 输出文件

默认输出目录：

```text
runtime/results/
```

每个测试场景生成：

```text
<case>_timeseries.csv
<case>_metrics.csv
<case>.png
```

完整套件额外生成：

```text
suite_summary.csv
```

### 5. Timeseries 主要字段

CSV 中包含：

```text
time_s
T_avg_C
T_stack_C
v_stack_mps
T_set_C
omega_out
omega_target
Tg_cmd_C
vg_cmd_mps
Tavg_pred_C
control_source
control_note
```

资源与执行器诊断包括：

```text
T_stack_available_C
v_stack_available_mps
mdot_stack_available_kgps
mdot_preheater_cmd_kgps
resource_limited
aux_heat_enable
heater_deficit_kW
mdot_aux_flow_kgps
fan_circulation_power_kW
```

扰动诊断包括：

```text
disturbance_Tavg_C
disturbance_Tstack_C
disturbance_vstack_mps
disturbance_est_Tavg_C
disturbance_est_Tstack_C
disturbance_est_vstack_mps
```

feed 诊断包括：

```text
feed_x1 ... feed_x6
slot_omega0
```

### 6. Metrics 主要指标

每个场景按 full、pre_event、event_window、post_event、tail、recovery 等区段输出指标。常用字段为：

```text
duration_s
Tavg_MAE_C
Tavg_RMSE_C
Tavg_mean_C
Tavg_min_C
Tavg_max_C
Tavg_pp_C
ratio_in_ref_band
ratio_in_safe_band
ratio_in_supervision_band
omega_req_tv
Tg_cmd_tv
recovery_to_safe_s
recovery_to_ref_s
overshoot_crossings
aux_flow_mean_kgps
aux_flow_max_kgps
fan_power_mean_kW
fan_energy_kJ
heater_deficit_energy_kJ
```

这些指标用于评估：

1. 稳态精度；
2. 调整时间；
3. 超调和安全带越界；
4. 命令平滑性；
5. 补热、循环和风机经济代价；
6. 预测模型与真实 plant 的一致性。

### 7. 绘图内容

每个场景图包含：

1. 平均炉温与参考/安全区间；
2. 出口含水率与目标含水率；
3. \(T_g\) 命令；
4. \(v_g\) 命令；
5. 入口 feed 组分；
6. 补热、循环流量和风机功率诊断。

资源诊断子图用于区分：

- 温度资源不足导致的补热；
- 自然烟气一次通过质量流量不足导致的循环/补偿需求；
- 高循环速度对应的风机经济代价。

---

## 工程运行说明

建议使用项目根目录作为运行目录：

```bash
cd FlameGuard-main
python3.14 -m runtime.tests.test_preheater_timestep_invariance
python3.14 -m runtime.tests.run_all_cases
```

运行结果写入：

```text
runtime/results/
```

项目结构说明见：

```text
ARCHITECTURE.md
```

历史代码归档说明见：

```text
legacy/LEGACY.md
```

版本整理记录见：

```text
CHANGELOG.md
```
