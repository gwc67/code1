# PRD：`analyze_pid_performance` 升级为 PID 建模 + 调参工具

> 来源：与用户的设计访谈（grilling）完整收敛结果
> 范围：本地 MATLAB 项目 `code1`，单一函数 `analyze_pid_performance.m`
> 关联外部代码（只读参考，不在本 PRD 改动范围内）：
> - STM32 飞控 C 代码 `PID_ctrl.c`（PID 算法）
> - STM32 飞控 C 代码 `fc_ctrl.c`（10ms 任务 + 坐标旋转）

---

## Problem Statement

当前 `analyze_pid_performance.m` 是一个**启发式诊断脚本**：它硬编码了 `PID_ctrl.c` 中的默认参数，基于几个阈值（稳态误差、超调、D 项贡献）给出 `if-else` 形式的模糊建议（"建议增大 Ki"），**无法给出具体可烧录的参数值**。

用户（无人机飞控开发者）真正的痛点是：

1. 每次调参都靠"试飞 → 看日志 → 拍脑袋改 Kp/Ki/Kd → 再试飞"，**没有定量依据**。
2. 飞控 C 代码里有**自适应增益调度、积分分离、抗饱和、微分滤波、前馈**等多种机制，人工很难判断"当前参数到底哪里不好"。
3. 不同飞行日志（不同任务/载荷/目标轨迹）可能需要**不同的 PID 参数**，没有系统化方法针对具体数据给出建议。
4. 现有函数**只能看当前参数的表现**，不能回答"如果我把 Kp 改成 0.22，预期会怎样？"

**用户需要的是：一个能从真实飞行日志中辨识出被控对象模型、并基于控制理论（而非 if-else）推荐具体 PID 参数值的工具。**

---

## Solution

把 `analyze_pid_performance` 升级为一个**混合式 PID 分析工具**，包含两个核心模块：

- **Part A（正向复现）**：在 MATLAB 中完整复刻 C 代码的 PID 算法（含 XY 坐标旋转、自适应、积分分离、抗饱和、微分滤波、修正后的前馈），把 logged `cmd_vel` 喂进辨识出的被控对象模型，验证模型能复现 logged `radar_pos`，从而建立用户对模型精度的信任。
- **Part B（被控对象辨识 + 调参）**：对每轴独立做 SISO 物理模型辨识（`G(s) = K·e^(-τs) / (s(Ts+1))`），然后用 **IMC 解析公式** 给初始猜测，再用 **ITAE 优化**（`fmincon`）在约束下精修，输出**可直接烧录到 STM32 的 `Kp_base, Ki_base, Kd_base`**。

**输出三件套**：控制台诊断报告 + 6 张分析图 + 可程序化使用的 `result` struct，并自动保存 `.mat` 文件到 CSV 同目录。

---

## User Stories

### 输入与调用方式

1. As a 飞控开发者, I want to 只需传入 CSV 文件路径就能跑分析, so that 我可以零配置地开始诊断（函数内部使用 C 代码默认参数作为 fallback）
2. As a 飞控开发者, I want to 能用 struct 部分覆盖 PID 参数, so that 我可以快速验证"如果我用另一组参数"的假设而不需要改源码
3. As a 飞控开发者, I want to 能指定 `tuningMode` 为 `'balanced'`/`'aggressive'`/`'conservative'`, so that 工具能根据我的任务场景（追目标 vs 精密悬停）自动调整推荐激进程度
4. As a 高级用户, I want to 能为每轴指定辨识初值 `plant_init = [K0, T0, tau0]`, so that 当自动估计不收敛时我可以手动引导优化
5. As a 用户, I want to 不用在 CSV 里提供时间戳列, so that 我的现有日志格式不需要改变（函数固定假设 10Hz / dt=0.1s）

### 数据读取与质量门控

6. As a 用户, I want to 函数自动识别 CSV 列含义（U1-U3 雷达位置、U4-U6 目标位置、U7-U9 速度指令）, so that 我不需要手动指定列映射
7. As a 用户, I want to 函数自动检测数据质量（cmd_vel 方差、目标变化次数）, so that 如果日志里无人机几乎没动，函数会警告我"数据不足以辨识"而不是给出误导性的参数建议
8. As a 用户, I want to 数据不足时函数优雅中止辨识部分, so that 我可以收到清晰的原因说明并知道该补充什么样的飞行数据

### Part A：正向复现与模型验证

9. As a 飞控开发者, I want to MATLAB 版 PID 仿真结果能匹配 C 代码行为, so that 我相信后续的调参建议是基于真实算法得出的
10. As a 飞控开发者, I want to MATLAB 版完整复刻 C 代码的 XY 坐标旋转逻辑, so that 2D 曲线飞行时闭环验证也准确（不只是单轴直线运动）
11. As a 飞控开发者, I want to MATLAB 版修正 C 代码中 feed-forward 的已知 bug（错误地混入上一拍 setpoint）, so that 我的推荐参数基于"修正后"的算法，避免把 bug 的影响误算进模型
12. As a 飞控开发者, I want to 函数在控制台打印警告提示 feed-forward 已被修正, so that 我知道 C 代码还有这个 bug 待修
13. As a 飞控开发者, I want to 函数复刻 C 代码的自适应增益调度（三区域 Kp/Ki/Kd 切换）, so that 闭环验证能反映真实飞行中的增益变化
14. As a 飞控开发者, I want to 函数复刻 C 代码的积分分离（I_Band）+ 抗饱和（back-calculation）, so that 积分项行为与真实代码一致
15. As a 飞控开发者, I want to 函数复刻 C 代码的微分低通滤波（d_filter_alpha）, so that 微分项的噪声抑制行为被正确反映
16. As a 用户, I want to 看到"开环验证"图（logged cmd_vel → 模型 → 预测 radar_pos vs 实际 radar_pos）, so that 我能直观判断被控对象模型 G(s) 是否准确
17. As a 用户, I want to 看到 NRMSE（归一化均方根误差）数值, so that 我有定量的模型质量指标（建议 < 15%）
18. As a 用户, I want to 看到"闭环验证"图（logged PID 参数 + 模型 vs 实际 logged 轨迹）, so that 我能验证整体系统（PID + 被控对象）的仿真精度

### Part B：被控对象辨识

19. As a 控制理论使用者, I want to 被控对象模型采用物理可解释的结构 `G(s) = K·e^(-τs) / (s(Ts+1))`, so that 我能从辨识结果里读出"等效速度增益 / 内环时间常数 / 纯延迟"等物理量
20. As a 飞控开发者, I want to 每轴独立辨识（SISO）, so that 算法简单稳定，不被 XY 弱耦合干扰
21. As a 用户, I want to 辨识初值由函数自动从数据估计, so that 大多数情况下我不需要手工提供初值
22. As a 高级用户, I want to 能通过 `pidParams.X.plant_init` 覆盖自动初值, so that 自动估计失败时我能救场
23. As a 用户, I want to 看到辨识出的 G(s) 表达式（每轴）打印在控制台, so that 我能复制到其他地方使用
24. As a 用户, I want to 看到 Bode 图, so that 我能直观看到被控对象的频域特性（增益裕度、相位裕度、带宽）

### Part B：调参推荐

25. As a 飞控开发者, I want to 推荐基于 IMC（内模控制）解析公式, so that 推荐有清晰的理论基础，不是黑盒
26. As a 飞控开发者, I want to IMC 的初始猜测被 ITAE 优化在约束下精修, so that 推荐的参数在满足超调/控制量/稳定裕度约束的前提下是接近最优的
27. As a 飞控开发者, I want to `balanced` 模式下 λ ≈ 2τ, `conservative` 下 λ ≈ 3τ, `aggressive` 下 λ ≈ τ, so that 三种档位有清晰的物理区分
28. As a 飞控开发者, I want to 默认档位适合精密悬停（conservative：超调 ≤ 5%）, so that 推荐参数对安全性友好
29. As a 飞控开发者, I want to 约束包含"控制量 ≤ 80% 输出限幅", so that 推荐的参数不会让执行器频繁饱和
30. As a 飞控开发者, I want to 约束包含"相位裕度 ≥ 45°", so that 即使被控对象模型有误差，闭环仍然稳定
31. As a 飞控开发者, I want to 推荐范围**仅限 Kp_base, Ki_base, Kd_base**, so that 我不会被过多的建议参数搞糊涂，而自适应比率、I_Band 等高级参数保留为当前值
32. As a 飞控开发者, I want to 看到"当前参数 vs 建议参数"的对比表, so that 我知道每个参数要改多少
33. As a 飞控开发者, I want to 看到"预期性能改善"对比（超调、settling time、稳态误差、控制量峰值）, so that 我能判断"换这组参数值不值得"

### 诊断与高级洞察

34. As a 飞控开发者, I want to 函数诊断积分饱和风险（观测 logged integral 项是否频繁触及 integral_max）, so that 我能在必要时调整 integral_max 或 I_Band
35. As a 飞控开发者, I want to 函数诊断 D 项的实际贡献比例, so that 我知道微分项是不是在"白干活"（传感器噪声太大被滤波掉了）
36. As a 飞控开发者, I want to 函数诊断自适应增益调度是否真的在起作用, so that 我知道 C 代码里的三区域切换逻辑是不是有意义

### 输出形式

37. As a 用户, I want to 控制台报告格式清晰、分节、带 emoji 标记（✅ ⚠️ ❌）, so that 我能 30 秒内抓住关键结论
38. As a 用户, I want to 图表自动组织成一张大 figure（6 张 subplot）, so that 我能一眼看清整体分析
39. As a 高级用户, I want to 函数返回 `result` struct（包含推荐参数、辨识模型、性能指标）, so that 我能在脚本里批量处理多个 CSV 并对比结果
40. As a 用户, I want to 自动保存 `result_<csv_name>_<timestamp>.mat` 到 CSV 同目录, so that 我事后能重新打开分析结果，不丢失
41. As a 用户, I want to 函数运行结束后控制台打印 `.mat` 文件的保存路径, so that 我知道去哪里找

### 文件与版本管理

42. As a 用户, I want to 现有的 `analyze_pid_performance.m` 被保留为 `analyze_pid_performance_v0.m` 备份, so that 我可以对比新旧版本、或在新版本出问题时回退
43. As a 用户, I want to 新实现写入 `analyze_pid_performance.m`, so that 我现有的调用脚本不用改路径

---

## Implementation Decisions

### I1. 函数签名

```
result = analyze_pid_performance(csvPath, pidParams, tuningMode)
```

- `csvPath`：必填，字符串，CSV 文件路径
- `pidParams`：可选 struct，嵌套按轴（`pidParams.X`, `pidParams.Y`, `pidParams.Z`），每轴可覆盖：
  - PID 基础增益：`Kp_base`, `Ki_base`, `Kd_base`
  - 自适应参数：`error_threshold_high/low`, `Kp/Ki/Kd_high_ratio`
  - 积分参数：`integral_max`, `I_Band`
  - 微分滤波：`d_filter_alpha`
  - 前馈：`kv`, `ka`
  - 输出限幅：`output_max`, `output_min`
  - 辨识初值：`plant_init = [K0, T0, tau0]`
  - 任何未指定字段从 C 代码 `PID_Init` 默认值读取
- `tuningMode`：可选字符串，`'balanced'`（默认）/ `'aggressive'` / `'conservative'`
- 返回值：`result` struct（详见 O3）

### I2. 数据规格

- CSV 列映射（固定，不自动检测）：
  - U1-U3 → RADAR_POS_X/Y/Z（测量值）
  - U4-U6 → TARGET_POS_X/Y/Z（设定值）
  - U7-U9 → CMD_SPEED_X/Y/Z（PID 输出 / 被控对象输入）
- 跳过首行（header）
- 固定采样率：10Hz，dt = 0.1s
- 单位：沿用 C 代码内部单位（radar_pos 为 x100 整数单位，cmd_vel 为 C 代码的 float 输出）

### I3. Part A — C PID 复刻

- 在 MATLAB 中实现一个"PID 模拟器"，状态变量与 C 代码 `struct AdaptivePID_t` 完全对齐
- 每轴独立实例，每步执行：
  1. 计算误差（注意 XY 旋转）
  2. 三区域自适应增益调度
  3. 积分分离 + 限幅
  4. 微分低通滤波（基于测量值）
  5. PID 输出 = Kp·e + Ki·∫e - Kd·d_filtered
  6. **修正后的前馈**：`target_vel = (measurement[k] - measurement[k-1])/dt`（C 代码错误地用了 `measurement[k] - setpoint[k-1]`）
  7. 输出限幅 + 抗饱和（back-calculation 重置 integral）
- **XY 坐标旋转**完整复刻：
  - 检测 `target_pos` 变化 → 重设 start_point、计算 theta_1、reset PID
  - 把 radar_pos 投影到"起点→目标"方向（`length.x`）和垂直方向（`length.y`）
  - X PID 跟踪 s_setpoint_modulus，Y PID 跟踪 0
  - 输出反变换回雷达系

### I4. Part B — 被控对象辨识

- 模型结构：`G(s) = K·e^(-τs) / (s(Ts+1))`
  - 物理含义：K = 速度增益（cmd_vel → 稳态速度），T = 内环+阻力时间常数，τ = 传感器+处理纯延迟
- 每轴独立 SISO 辨识，输入 = logged cmd_vel，输出 = logged radar_pos
- 使用 MATLAB `lsqnonlin`（Optimization Toolbox）最小化仿真误差
- 初始猜测自动估计：
  - K：从稳态斜率 `Δpos/Δt / mean(cmd_vel)`
  - T：从 63% 响应时间
  - τ：从响应起始延迟
- 用户可通过 `pidParams.<axis>.plant_init = [K0, T0, tau0]` 覆盖
- 输出 `idtf` 或 `tf` 对象便于后续频域分析

### I5. Part B — 调参算法

- **第一步：IMC 解析公式**
  - Kp_init = T / (K·(λ+τ))
  - Ki_init = 1 / (K·(λ+τ))
  - Kd_init = T·τ / (K·(λ+τ))
  - λ 由 tuningMode 决定：`conservative` → 3τ，`balanced` → 2τ，`aggressive` → τ
- **第二步：ITAE 优化精修**
  - 目标函数：ITAE = ∫ t·|e(t)| dt（在辨识出的 G(s) 上闭环仿真）
  - 优化器：`fmincon`
  - 约束：
    - 最大超调 ≤ 5%（precision hover 默认，可调）
    - 控制量峰值 ≤ 80% × output_max
    - 相位裕度 ≥ 45°（通过 `margin()` 校验）
- **推荐范围**：仅 Kp_base, Ki_base, Kd_base

### I6. 数据质量门控

- 检查 cmd_vel 每轴方差 < 阈值 → 警告"激励不足"
- 检查 target_pos 变化次数 < 2 → 警告"缺少阶跃，难以辨识动态"
- 警告打印后中止辨识部分，仅输出原始数据统计

### I7. 输出形式

**控制台**（按轴分节）：
- 被控对象辨识结果（G(s) 表达式 + 物理含义 + NRMSE）
- 当前 vs 建议参数对比表
- 预期性能改善对比表
- 诊断项（积分饱和、D 项贡献、数据充分性）

**图表**（单 figure，6 subplot）：
1. 数据概览：三轴 target vs radar_pos
2. 被控对象开环验证：模型预测 vs 实际（每轴叠加）
3. Bode 图：辨识出的 G(s)
4. 当前参数闭环阶跃响应：仿真 vs 实际
5. 建议参数闭环阶跃响应：预测
6. 控制量对比：当前 vs 建议（含限幅参考线）

**返回值 struct**：
- `result.recommended.X/Y/Z.Kp_base, Ki_base, Kd_base`
- `result.identified_plant.X/Y/Z`（tf 对象）
- `result.identified_plant.X/Y/Z_params`（`struct('K', 'T', 'tau')`）
- `result.metrics.X/Y/Z`（`struct('overshoot', 'settling_time', 'steady_state_error', 'peak_control')`）
- `result.model_fit.NRMSE`（`struct('X', 'Y', 'Z')`）

**持久化**：
- 保存 `result_<csv_name>_<yyyyMMdd_HHmmss>.mat` 到 CSV 同目录
- 控制台打印保存路径

### I8. 文件操作

- 实现前先重命名：`analyze_pid_performance.m` → `analyze_pid_performance_v0.m`
- 新实现写入 `analyze_pid_performance.m`

### I9. C 代码 bug 处理

- C 代码中 feed-forward 项 `target_vel_f = (measurement_f - pre_target_position_f)/dt` 中，`pre_target_position_f` 在末尾被赋值为 `setpoint_f`，导致该项混入了测量值与上一拍设定值
- 用户已确认为 bug，MATLAB 版修正为 `target_vel = (measurement[k] - measurement[k-1])/dt`
- 函数在控制台打印一次警告提示此修正

### I10. 依赖

- 需要 MATLAB R2025a + Optimization Toolbox + Control System Toolbox
- 不需要 System Identification Toolbox（使用自实现 `lsqnonlin` + `tf`）
- 所有使用的外部函数：`readmatrix`, `detectImportOptions`, `lsqnonlin`, `fmincon`, `tf`, `margin`, `step`, `ltlsim`

---

## Testing Decisions

### T1. 测试哲学

只测试**外部行为**，不测试内部实现细节。具体而言：
- 给定已知输入数据 → 检查输出 struct 字段存在、数值合理
- 给定合成数据（已知被控对象 + 已知 PID 参数仿真得到）→ 检查辨识出的 (K, T, τ) 与真值偏差 < 20%
- 给定合成数据 → 检查推荐的 PID 参数在闭环仿真中确实满足约束（超调 ≤ 5%、控制量不饱和、相位裕度 ≥ 45°）

### T2. 测试层级

- **单元测试**（建议新建 `test_analyze_pid_performance.m`，不在本 PRD 实现范围内但预留接口）：
  - 测试默认参数 struct 填充是否正确（对照 C 代码 `PID_Init` 硬编码值）
  - 测试数据质量门控在合成"无激励"数据上能正确警告
  - 测试 PID 模拟器在已知输入下与 C 代码行为一致（合成一个简单阶跃响应）
- **端到端测试**：
  - 用现有 `3_kd_0_5.csv` 和 `speed_15.csv` 跑一遍，确认函数不报错、返回完整 struct、保存 .mat 文件

### T3. 验收标准

- 在 `3_kd_0_5.csv` 上运行，NRMSE < 20%（被控对象模型合理）
- 推荐的参数在仿真中确实改善性能（超调、settling time 至少一项改善，无恶化项）
- 控制台报告清晰、分节正确、无 NaN / Inf

---

## Out of Scope

以下**不在本 PRD 范围内**，留待后续迭代：

- **Yaw 轴分析**：当前 CSV 只有 U1-U9（XYZ），不含 yaw 数据；Yaw PID 单独处理
- **摄像头 PID（`cam_xy_pst`）**：与雷达定位是独立回路，本工具只针对 `loc_xyz_pst`
- **多变量耦合辨识（MIMO）**：V1 只做 SISO；XY 耦合留待 V2
- **自动烧录参数到 STM32**：本工具只输出建议，不直接修改 C 代码或烧录固件
- **在线/实时调参**：本工具是离线分析工具
- **其他模型结构**（如 ARX、状态空间黑盒）：本 PRD 锁定物理模型 `K·e^(-τs)/(s(Ts+1))`
- **推荐高级参数**（自适应比率、I_Band、d_filter_alpha、kv/ka）：V1 只推荐基础增益
- **CSV 时间戳列支持**：V1 固定 10Hz
- **批处理多个 CSV**：V1 单文件；批量处理可作为后续脚本层功能
- **C 代码 feed-forward bug 的修复**：本 PRD 只在 MATLAB 端修正并打印警告，不修改 STM32 C 代码
- **`gh` CLI 发布 Issue**：当前环境无 `gh`，PRD 以本地 markdown 形式保存

---

## Further Notes

### 关于 C 代码的几点观察（不改动，仅记录）

1. **X 轴自适应过渡区不可达**：`error_threshold_high_f == error_threshold_low_f == 25`，导致过渡分支 `low < abs_error <= high` 永假，X 轴在高/低两区直接跳变
2. **`float_to_int32_l` 函数未被调用**：`PID_Update_l` 返回类型是 `int32_t`，但内部 `return out_put_f` 直接返回 float，存在隐式 float→int32 截断
3. **`Yaw_PID_Update` 的测量值构造奇特**：调用 `PID_Update_l(pid, setpoint, setpoint + angleDifference)`，实际是让标准 error 计算产出 wrap-around 后的误差，建议加注释说明

### 关于被控对象模型的物理直觉

- 10Hz 采样下，奈奎斯特频率 = 5Hz，实际可辨识带宽 ≈ 1-2Hz
- 无人机内环姿态响应（~50-100ms）会表现为 1-2 拍的纯延迟 τ（100-200ms）
- 时间常数 T 通常在 100-500ms 范围
- 速度增益 K 接近 1（如果 cmd_vel 单位标定正确）

### 风险与缓解

| 风险 | 缓解 |
|---|---|
| 数据激励不足导致辨识失败 | 数据质量门控（I6）提前拦截 |
| `lsqnonlin` 陷入局部最优 | 自动初值估计 + 用户可覆盖 |
| 推荐参数在实飞中不稳定 | 约束相位裕度 ≥ 45°，留稳定裕度 |
| XY 旋转复刻 bug 导致 Part A 验证失败 | 与 C 代码逐行对照，关键断言打印 |
| `.mat` 文件覆盖旧结果 | 文件名带时间戳 |

### 后续 V2 候选

- 支持 Yaw 轴（需要日志包含 yaw 数据）
- 支持 MIMO 辨识（XY 耦合）
- 支持自定义模型结构（ARX / 状态空间）
- 推荐高级参数（自适应比率、I_Band）
- 自动对比多个 CSV 的调参趋势
- 与 STM32 工具链集成，一键烧录推荐参数
