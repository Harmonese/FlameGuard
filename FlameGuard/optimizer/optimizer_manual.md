# 一、工程问题与优化器定位

在节能减排社会实践创新大赛的整体系统中，预热炉承担“利用焚烧炉烟气余热，对高含水率厨余垃圾进行预热脱水，并向主焚烧炉提供受控含水率物料”的功能。预热炉的控制量主要为烟气入口温度和烟气入口速度，主焚烧炉的稳态燃烧状态由进入焚烧炉的垃圾含水率决定。

本手册聚焦于**预热炉静态运筹学优化器**本身的工程实现。该优化器在完整控制系统中的职责是：

> 在给定未来槽位等效物料性质、目标出口含水率、当前可用烟气资源边界以及动态允许目标带的前提下，求取一组可行、最优的预热炉静态设定值。

优化器的输出用于生成预热炉执行层的主控设定量；动态时延、物料滞留、执行器惯性和焚烧炉动态响应由系统其他模块承担。

优化器采用以下原则：

1. 以等效性质而不是原始 6 维组分作为主输入；
2. 在物理可实现域内跟踪目标出口含水率；
3. 在满足预热炉热工和资源约束的前提下，优先追求目标跟踪，再追求燃烧均匀性与能耗最优；
4. 支持离线扫描制表与在线查表/插值两种运行方式；
5. 输出结果同时包含主控设定量和完整的诊断信息，便于上位控制器、状态估计器和工程人员使用。

---

# 二、建模前期计算与参数定值

## 1. 原始输入数据

六类垃圾编号为：

- $x_1$：菜叶
- $x_2$：西瓜皮
- $x_3$：橙子皮
- $x_4$：肉
- $x_5$：杂项混合
- $x_6$：米饭

并满足：

$$
\sum_{i=1}^6 x_i = 1,\qquad x_i \ge 0
$$

实验测得三类原始性质：

$$
\omega_i = \{0.948,0.948,0.817,0.442,0.773,0.611\}
$$

$$
t_{\mathrm{ref},i} = \{12.1,17.7,15.3,11.5,16.3,15.8\}\ \text{min}
$$

$$
s_i = \{-0.132,-0.251,-0.189,-0.216,-0.210,-0.243\}\ \text{min}/^\circ\text{C}
$$

其中：

- $\omega_i$：第 $i$ 类垃圾的初始湿基含水率；
- $t_{\mathrm{ref},i}$：第 $i$ 类垃圾在参考温度 $175^\circ\text{C}$ 下达到 20% 含水率所需时间；
- $s_i$：温度敏感性斜率，取负值表示温度升高时达到同一脱水水平所需时间缩短。

## 2. 混合垃圾等效性质

对混合垃圾，采用线性等效假设：

1. 各组分均匀混合；
2. 混合性质可近似视作各组分性质的线性组合；
3. 当前工作温区内温度敏感性斜率可近似视为常数。

于是得到：

### 混合初始含水率

$$
\omega_0(x)=0.948x_1+0.948x_2+0.817x_3+0.442x_4+0.773x_5+0.611x_6
$$

### 混合参考干燥时间

$$
t_{\mathrm{ref}}(x)=12.1x_1+17.7x_2+15.3x_3+11.5x_4+16.3x_5+15.8x_6
$$

### 混合温度敏感性斜率

$$
s(x)=-0.132x_1-0.251x_2-0.189x_3-0.216x_4-0.210x_5-0.243x_6
$$

### 参考动力学时间

$$
\tau_{20}(T_m,x)=t_{\mathrm{ref}}(x)+s(x)(T_m-175)
$$

优化器主输入采用等效性质向量：

$$
p=
\begin{bmatrix}
\omega_0(x)\\
t_{\mathrm{ref}}(x)\\
s(x)
\end{bmatrix}
$$

工程实现中，通常先由组分清洗模块将原始 6 维组分翻译为 $\omega_0$、$t_{\mathrm{ref}}$、$s$ 三个量，再将这三个量送入优化器。

## 3. 主焚烧炉稳态代理模型

优化器内部保留主焚烧炉稳态代理模型，用于诊断和稳态约束评估。统一记号如下：

- $w$：进入主焚烧炉的垃圾含水率，单位为 %；
- $\omega=w/100$：相同含水率的湿基小数形式。

采用工程口径下的代理回归式：

### 烟囱出口平均温度

$$
T_{\text{stack}}(w)=-14.412237w+1423.472316
$$

### 烟囱出口平均速度

$$
v_{\text{stack}}(w)=-0.215310w+25.332842
$$

### 燃烧核心区域平均温度

$$
T_{\text{avg}}(w)=-13.109632w+1294.871365
$$

### 燃烧面温度标准差

$$
\sigma(w)=-0.189997w+18.331239
$$

### 燃烧面最低温度

$$
T_{\min}(w)=-13.422320w+1330.692379
$$

### 燃烧面最高温度

$$
T_{\max}(w)=-16.072025w+1589.019616
$$

这些关系在优化器中的用途包括：

1. 生成稳态燃烧诊断量；
2. 在 `strict` 运行策略下作为稳态燃烧硬约束；
3. 为离线表附加主焚烧炉稳态预测输出。

## 4. 稳态最优工作带

根据主焚烧炉稳态代理模型，可计算长期稳态工作带。

### 最低温度约束

$$
T_{\min}(w)\ge 850
$$

得到：

$$
w\le 35.81
$$

### 平均温度下限约束

$$
T_{\text{avg}}(w)\ge 850
$$

得到：

$$
w\le 33.93
$$

### 平均温度上限约束

$$
T_{\text{avg}}(w)\le 1100
$$

得到：

$$
w\ge 14.86
$$

### 最高温度约束

$$
T_{\max}(w)\le 1100
$$

得到：

$$
w\ge 30.43
$$

综合后得到稳态最优工作带：

$$
30.43\% \le w \le 33.93\%
$$

即：

$$
0.30426758 \le \omega \le 0.33934695
$$

该区间在优化器中作为：

- 稳态运行的参考带；
- 离线诊断量中的“稳态带偏离量”计算依据；
- `strict` 模式可行性判定的重要参考。

## 5. 预热炉设备结构与处理能力参数

根据既定设备参数：

- 主筒体内径 $D=1.2\ \text{m}$
- 有效长度 $L=3.2\ \text{m}$
- 工作充填率 $\phi=0.14$
- 内置导烟管数量 $N_t=6$
- 内置导烟管外径 $d_t=0.168\ \text{m}$
- 预热炉主烟道半径 $r_d=0.4\ \text{m}$
- 主焚烧炉烟囱半径 $r_s=0.3\ \text{m}$
- 设计体积密度 $\rho_b=450\ \text{kg/m}^3$
- 连续处理量 $20\ \text{t/d}$

连续进料质量流量为：

$$
\dot m_w = \frac{20000}{24\times 3600}=0.2315\ \text{kg/s}
$$

预热炉主烟道截面积：

$$
A_d=\pi r_d^2=0.503\ \text{m}^2
$$

焚烧炉烟囱截面积：

$$
A_s=\pi r_s^2=0.2827\ \text{m}^2
$$

有效传热面积：

$$
A=\pi DL+6\pi d_tL=22.20\ \text{m}^2
$$

炉内平均滞留量：

$$
M_h=\rho_b\phi\frac{\pi D^2}{4}L=228.0\ \text{kg}
$$

平均停留时间：

$$
\tau_r=\frac{M_h}{\dot m_w}=985.0\ \text{s}=16.42\ \text{min}
$$

优化器中的停留时间约束直接使用该 $\tau_r$。

## 6. 传热模型与烟气物性

### 总体传热系数模型

$$
U(v_g)=U_0+k v_g^n
$$

取值：

$$
U_0=18.97,\qquad k=20.09,\qquad n=0.65
$$

即：

$$
U(v_g)=18.97+20.09v_g^{0.65}
$$

### 烟气物性

取：

$$
c_{pg}=1.05\ \text{kJ/(kg⋅K)},\qquad \rho_{g,\mathrm{ref}}=0.78\ \text{kg/m}^3
$$

烟气密度函数：

$$
\rho_g(T)=0.78\times\frac{450}{T+273.15}
$$

于是：

$$
\rho_{\text{stack}}=\rho_g(T_{\text{stack}})
$$

$$
\rho_{\text{pre}}=\rho_g(T_g)
$$

### 烟气质量流量

烟囱可用质量流量上限：

$$
\dot m_{g,\max}^{\text{stack}}=\rho_g(T_{\text{stack}})A_sv_{\text{stack}}
$$

预热炉使用质量流量：

$$
\dot m_g=\rho_g(T_g)A_dv_g
$$

优化器中的资源约束由这两式给出。

## 7. 垃圾热工参数

环境温度与入炉垃圾温度取：

$$
T_0=T_{amb}=20^\circ\text{C}
$$

比热和潜热参数为：

$$
c_w=4.1844\ \text{kJ/(kg⋅K)}
$$

$$
c_s=1.70\ \text{kJ/(kg⋅K)}
$$

$$
\lambda=2257.9\ \text{kJ/kg}
$$

混合垃圾等效比热：

$$
c_{eq}(x)=(1-\omega_0(x))c_s+\omega_0(x)c_w
$$

---

# 三、优化器接口定义

## 1. 输入量

优化器的输入记为：

$$
\chi=
\Big(
\omega_0,\ t_{\mathrm{ref}},\ s,\ \omega_{\mathrm{tar}},\ T_{\text{stack}}^{\max},\ v_{\text{stack}}^{\max},\ [\omega_{\min}^{\mathrm{dyn}},\omega_{\max}^{\mathrm{dyn}}],\ \text{burn\_policy}
\Big)
$$

逐项解释如下。

### (1) 等效物料性质

$$
\omega_0,\qquad t_{\mathrm{ref}},\qquad s
$$

三者共同描述未来槽位物料的入口含水水平、参考干燥速度和温度敏感性。

### (2) 目标出口含水率

$$
\omega_{\mathrm{tar}}
$$

由上游模块提供，单位为湿基小数。该值代表当前求解槽位的目标出口含水率。

### (3) 资源边界

$$
T_{\text{stack}}^{\max},\qquad v_{\text{stack}}^{\max}
$$

分别表示当前可用烟气温度上限和烟囱平均速度。优化器将这两个量折算成烟气质量流量上限和温度上限约束。

### (4) 动态允许目标带

$$
[\omega_{\min}^{\mathrm{dyn}},\omega_{\max}^{\mathrm{dyn}}]
$$

表示当前时刻允许优化器追踪的目标带。优化器会将该目标带与物理可实现带求交，得到本次求解实际采用的目标带。

### (5) 运行策略 `burn_policy`

- `advisory`：主焚烧炉稳态代理输出作为诊断量，不加入硬约束；
- `strict`：主焚烧炉稳态代理的燃烧约束加入优化问题，作为硬约束处理。

## 2. 输出量

优化器输出记为：

$$
\psi=
\Big(
T_g^*,\ v_g^*,\ T_m^*,\ \omega^*,\ w^*,\ \Pi
\Big)
$$

其中：

- $T_g^*$：预热炉入口烟气温度最优设定值；
- $v_g^*$：预热炉入口烟气速度最优设定值；
- $T_m^*$：最优等效垃圾平均温度；
- $\omega^*$：最优出口含水率（湿基小数）；
- $w^*=100\omega^*$：最优出口含水率（百分数）；
- $\Pi$：诊断量集合。

诊断量 $\Pi$ 包括：

- 目标跟踪偏差 $d_\omega^-$、$d_\omega^+$；
- 参考动力学时间 $\tau_{20}$、目标动力学时间 $\tau_{\text{target}}$、平均停留时间 $\tau_r$；
- 供热功率 $\dot Q_{sup}$、所需热负荷 $\dot Q_{req}$、预热能耗 $P$；
- 烟囱可用质量流量上限和预热炉实际质量流量；
- 主焚烧炉稳态代理输出 $T_{\text{avg}}(w^*)$、$T_{\min}(w^*)$、$T_{\max}(w^*)$、$\sigma(w^*)$；
- 与稳态最优工作带的偏离量。

---

# 四、优化器内部变量与目标函数

## 1. 决策变量

优化器的决策变量为：

$$
z=
\begin{bmatrix}
T_g\\
v_g\\
T_m\\
\omega\\
d_\omega^-\\
d_\omega^+
\end{bmatrix}
$$

其中：

- $T_g$：预热炉入口烟气温度；
- $v_g$：预热炉入口烟气速度；
- $T_m$：垃圾层等效平均温度；
- $\omega$：出口含水率（湿基小数）；
- $d_\omega^-$、$d_\omega^+$：目标跟踪偏差变量。

## 2. 目标跟踪方程

$$
\omega+d_\omega^- - d_\omega^+=\omega_{\mathrm{tar,proj}}
$$

其中 $\omega_{\mathrm{tar,proj}}$ 是将请求目标 $\omega_{\mathrm{tar}}$ 投影到“动态允许带与物理可实现带交集”后的结果。

## 3. 词典序目标

优化器采用三阶段词典序优化。

### 第一阶段：目标含水率跟踪

$$
J_1=d_\omega^-+d_\omega^+
$$

### 第二阶段：燃烧均匀性

$$
J_2=\sigma(w)=\sigma(100\omega)
$$

### 第三阶段：预热能耗

$$
J_3=P(T_g,v_g)
$$

实现上，第二阶段在第一阶段最优值的容差内继续求解，第三阶段在前两阶段最优值的容差内继续求解，从而得到词典序结果。

---

# 五、优化器约束体系

## 1. 动态允许目标带与物理可实现带

动态允许目标带由上游给定：

$$
\omega\in[\omega_{\min}^{\mathrm{dyn}},\omega_{\max}^{\mathrm{dyn}}]
$$

物理可实现带定义为：

$$
\omega\in[0.20,\ \min(\omega_0,0.98)]
$$

本次求解实际采用的目标带为两者交集：

$$
\omega\in[\omega_{\min}^{\mathrm{app}},\omega_{\max}^{\mathrm{app}}]
$$

并将请求目标投影到该区间：

$$
\omega_{\mathrm{tar,proj}}=
\Pi_{[\omega_{\min}^{\mathrm{app}},\omega_{\max}^{\mathrm{app}}]}(\omega_{\mathrm{tar}})
$$

## 2. 预热炉动力学时间约束

给定等效性质 $p=(\omega_0,t_{\mathrm{ref}},s)$，有：

$$
\tau_{20}(T_m,p)=t_{\mathrm{ref}}+s(T_m-175)
$$

目标含水率对应的动力学时间采用线性缩放：

$$
\tau_{\text{target}}(T_m,p,\omega)=\tau_{20}(T_m,p)\cdot \frac{\omega_0-\omega}{\omega_0-0.20}
$$

约束为：

$$
\tau_{\text{target}}(T_m,p,\omega)\le \tau_r
$$

## 3. 所需热负荷

每千克初始湿垃圾蒸发水量：

$$
m_{evap}(\omega_0,\omega)=\frac{\omega_0-\omega}{1-\omega}
$$

单位时间所需热负荷：

$$
\dot Q_{req}=\dot m_w\left(c_{eq}(p)(T_m-T_0)+\lambda m_{evap}(\omega_0,\omega)\right)
$$

其中：

$$
c_{eq}(p)=(1-\omega_0)c_s+\omega_0c_w
$$

## 4. 供热能力约束

预热炉供热能力：

$$
\dot Q_{sup}=(U_0+kv_g^{0.65})A(T_g-T_m)
$$

换算到 kW 为：

$$
\dot Q_{sup}=\frac{(18.97+20.09v_g^{0.65})\cdot 22.20\cdot (T_g-T_m)}{1000}
$$

供热能力约束：

$$
\dot Q_{sup}\ge \dot Q_{req}
$$

## 5. 烟气资源约束

### 温度上限

$$
T_g\le T_{\text{stack}}^{\max}
$$

### 质量流量上限

$$
\rho_g(T_g)A_dv_g\le \rho_g(T_{\text{stack}}^{\max})A_sv_{\text{stack}}^{\max}
$$

## 6. 设备与传热约束

$$
3\le v_g\le 12
$$

$$
T_g\ge 100
$$

$$
T_m\le 250
$$

$$
T_g-T_m\ge 0
$$

$$
\omega\le \omega_0
$$

## 7. 主焚烧炉稳态代理约束（`strict` 模式）

当 `burn_policy=strict` 时，增加：

$$
T_{\min}(100\omega)\ge 850
$$

$$
850\le T_{\text{avg}}(100\omega)\le 1100
$$

$$
T_{\max}(100\omega)\le 1100
$$

当 `burn_policy=advisory` 时，这些量作为诊断信息输出，不作为硬约束参与求解。

---

# 六、优化器运行逻辑

优化器对单个未来槽位的求解流程如下：

## 步骤 1：输入校验

检查：

- $\omega_0\in(0,1)$；
- $t_{\mathrm{ref}}>0$；
- 动态允许目标带满足 $\omega_{\min}^{\mathrm{dyn}}<\omega_{\max}^{\mathrm{dyn}}$；
- 资源边界满足 $T_{\text{stack}}^{\max}\ge 100^\circ\text{C}$、$v_{\text{stack}}^{\max}>0$。

## 步骤 2：目标带求交与目标投影

计算动态允许目标带与物理可实现带的交集，得到实际应用带：

$$
[\omega_{\min}^{\mathrm{app}},\omega_{\max}^{\mathrm{app}}]
$$

并对请求目标做投影：

$$
\omega_{\mathrm{tar,proj}}
$$

## 步骤 3：生成初值集合

围绕温度、速度、等效平均温度和出口含水率生成一组初值种子，用于提高 SLSQP 数值求解的稳定性。

## 步骤 4：第一阶段求解

在全部约束下求解：

$$
\min J_1
$$

得到阶段一最优值 $J_1^*$。

## 步骤 5：第二阶段求解

在保留阶段一最优容差约束的条件下求解：

$$
\min J_2
$$

得到阶段二最优值 $J_2^*$。

## 步骤 6：第三阶段求解

在保留前两阶段最优容差约束的条件下求解：

$$
\min J_3
$$

得到最终最优解：

$$
(T_g^*,v_g^*,T_m^*,\omega^*)
$$

## 步骤 7：诊断量计算

根据最优解计算：

- $\tau_{20}$、$\tau_{\text{target}}$、$\tau_r$；
- $\dot Q_{sup}$、$\dot Q_{req}$、$P$；
- 烟气质量流量占用；
- $T_{\text{avg}}$、$T_{\min}$、$T_{\max}$、$\sigma$；
- 稳态工作带偏离量；
- 严格燃烧可行性判断。

---

# 七、制表方案

## 1. 制表目的

离线制表用于将优化器从“在线求解器”转化为“在线查表/插值器”。对于大量重复工况，先离线扫描并存储


## 2. 推荐制表输入轴

推荐的离线制表输入轴为：

$$
(\omega_0,\ t_{\mathrm{ref}},\ s,\ \omega_{\mathrm{tar}},\ T_{\text{stack}}^{\max},\ v_{\text{stack}}^{\max},\ \omega_{\min}^{\mathrm{dyn}},\omega_{\max}^{\mathrm{dyn}},\ \text{burn\_policy})
$$

其中：

- 前三项表示未来槽位等效物料性质；
- 第四项表示请求目标；
- 第五、第六项表示当前烟气资源边界；
- 第七、第八项表示动态允许目标带；
- 最后一项表示运行策略。

## 3. 制表输出字段

推荐存储字段包括：

### 核心设定值

- `Tg_star_C`
- `vg_star_mps`
- `Tm_star_C`
- `omega_opt`
- `w_opt_percent`

### 目标带与目标投影信息

- `omega_tar_requested`
- `omega_tar_projected`
- `omega_band_applied_min`
- `omega_band_applied_max`

### 约束与诊断信息

- `d_omega_minus`
- `d_omega_plus`
- `tau20_min`
- `tau_target_min`
- `tau_r_min`
- `Qsup_kW`
- `Qreq_kW`
- `power_kW`
- `mdot_stack_cap_kgps`
- `mdot_preheater_kgps`
- `Tavg_burn_C`
- `Tmin_burn_C`
- `Tmax_burn_C`
- `sigma_burn_C`
- `steady_band_violation_percent`
- `feasible_under_strict_burn`
- `max_constraint_violation`
- `success`
- `message`

## 4. 表的使用方式

在线阶段可采用以下方式使用控制表：

1. 上游模块提供未来槽位等效物料性质 $(\omega_0,t_{\mathrm{ref}},s)$；
2. 上游模块提供目标出口含水率 $\omega_{\mathrm{tar}}$ 和动态允许目标带；
3. 实测获得当前烟气资源边界 $(T_{\text{stack}}^{\max},v_{\text{stack}}^{\max})$；
4. 使用这些量作为表索引做多维插值；
5. 读取 $T_g^*$、$v_g^*$ 并送入执行器；
6. 将诊断量送入上位机、报警模块或离线评估模块。

---

# 八、模块输入、输出与实现建议

## 1. 模块输入

建议定义如下数据结构：

- `EquivalentProperties(omega0, tref_min, slope_min_per_c)`
- `ResourceBoundary(T_stack_cap_C, v_stack_cap_mps)`
- `DynamicTargetBand(omega_min, omega_max)`
- `OptimizerRequest(props, omega_tar, resource, dyn_band, slot_id, burn_policy)`

## 2. 模块输出

建议输出 `OptimizerResult`，至少包含：

- 求解是否成功；
- 核心设定值 $T_g^*,v_g^*,T_m^*,\omega^*$；
- 目标带投影结果；
- 主要约束核验量；
- 燃烧代理输出；
- 数值求解误差。

## 3. 数值求解建议

- 采用 `scipy.optimize.minimize(method="SLSQP")`；
- 使用多组初值提高稳定性；
- 采用三阶段词典序优化；
- 在结果中输出最大约束违背量，便于诊断；
- 对失败样本保留完整输入和报错信息，便于补表和修模。

---

# 九、完整静态优化模型

优化器的静态模型可归纳为：

## 输入

$$
\chi=
\Big(
\omega_0,\ t_{\mathrm{ref}},\ s,\ \omega_{\mathrm{tar}},\ T_{\text{stack}}^{\max},\ v_{\text{stack}}^{\max},\ [\omega_{\min}^{\mathrm{dyn}},\omega_{\max}^{\mathrm{dyn}}],\ \text{burn\_policy}
\Big)
$$

## 决策变量

$$
z=
\begin{bmatrix}
T_g\\
v_g\\
T_m\\
\omega\\
d_\omega^-\\
d_\omega^+
\end{bmatrix}
$$

## 目标函数

第一阶段：

$$
\min J_1=d_\omega^-+d_\omega^+
$$

第二阶段：

$$
\min J_2=\sigma(100\omega)
$$

第三阶段：

$$
\min J_3=P(T_g,v_g)
$$

## 约束条件

$$
\omega+d_\omega^- - d_\omega^+=\omega_{\mathrm{tar,proj}}
$$

$$
\tau_{\text{target}}(T_m,p,\omega)\le \tau_r
$$

$$
\dot Q_{sup}(T_g,v_g,T_m)\ge \dot Q_{req}(p,T_m,\omega)
$$

$$
\rho_g(T_g)A_dv_g\le \rho_g(T_{\text{stack}}^{\max})A_sv_{\text{stack}}^{\max}
$$

$$
T_g\le T_{\text{stack}}^{\max}
$$

$$
3\le v_g\le 12
$$

$$
T_g\ge 100
$$

$$
T_m\le 250
$$

$$
T_g-T_m\ge 0
$$

$$
\omega\le \omega_0
$$

并在 `strict` 模式下增加：

$$
T_{\min}(100\omega)\ge 850
$$

$$
850\le T_{\text{avg}}(100\omega)\le 1100
$$

$$
T_{\max}(100\omega)\le 1100
$$

## 输出

$$
\psi=
\Big(
T_g^*,\ v_g^*,\ T_m^*,\ \omega^*,\ w^*,\ \Pi
\Big)
$$

该输出既可以直接用于在线控制执行，也可以作为离线控制表的记录行。
