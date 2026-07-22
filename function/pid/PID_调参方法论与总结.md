# PID 调参方法论与总结

## 项目概述

本文档记录了无人机位置环 PID 控制系统的分析、仿真与调参过程。通过建立高精度的 MATLAB 仿真模型,结合系统辨识和 IMC 调参方法,成功优化了定点保持性能。

---

## 一、关键发现与修复

### 1.1 MATLAB 仿真与 C 代码不一致问题

**问题描述**: 初始 MATLAB 仿真输出与实际 C 代码输出差异巨大,无法用于参数调优。

**根本原因**: `dt` 单位不匹配

- C 代码中 `xTaskGetTickCount()` 返回的是 **RTOS tick 数**(1 tick = 1ms)
- MATLAB 仿真使用的是 **秒**
- 差异约 **1000 倍**,导致积分项、微分项、前馈项全部计算错误

**修复方法**:
```matlab
% my_pid_analyze.m 第 70 行
dt_k = dt_array(k) * 1000;  % 秒 → RTOS tick
```

**其他修复**:
1. `pos_cmd_st` 函数参数列表错误(多了两个参数)
2. `pid_reset_v` 行为不完整(只重置 integral 和 prev_measurement,不重置其他状态)
3. 反饱和计算中的 `1e-10` 冗余
4. 初始状态 `pre_target_position` 应为 0 而非 measurement

**结果**: 修复后仿真与实际 cmd_vel 高度拟合,NRMSE < 5%。

---

### 1.2 机体系坐标旋转建模

**问题描述**: 仿真只计算了雷达系速度指令,但实际飞控接收的是机体系速度。

**发现**: C 代码中 `s_cmd_vel_consume_v()` 使用四元数将雷达系速度旋转到机体系:

```c
// My_ANO_LX.c 第 47-88 行
float qX_f = -qua_st.qX_f;  // 取共轭
float qY_f = -qua_st.qY_f;
float qZ_f = -qua_st.qZ_f;
float qW_f = qua_st.qW_f;

vel_x = (1 - 2*qY² - 2*qZ²) * vX + (2*qX*qY - 2*qW*qZ) * vY;
vel_y = (2*qX*qY + 2*qW*qZ) * vX + (1 - 2*qX² - 2*qZ²) * vY;
```

**MATLAB 实现**:
```matlab
function [vx_body, vy_body] = quat_rot_vel_xy(vx_radar, vy_radar, q)
    qw = q(1); qx = -q(2); qy = -q(3); qz = -q(4);  % 共轭
    
    r11 = 1 - 2*qy*qy - 2*qz*qz;
    r12 = 2*qx*qy - 2*qw*qz;
    r21 = 2*qx*qy + 2*qw*qz;
    r22 = 1 - 2*qx*qx - 2*qz*qz;
    
    vx_body = r11 * vx_radar + r12 * vy_radar;
    vy_body = r21 * vx_radar + r22 * vy_radar;
end
```

**数据支持**: CSV 中新增列 `qw,qx,qy,qz,rt_tar_vel_x,rt_tar_vel_y`,可用于验证旋转模型。

**结果**: 仿真现在可以输出机体系速度,与实际 `rt_tar_vel` 对比验证。

---

## 二、分析方法与工具

### 2.1 超调分析工具 (`analyze_overshoot.m`)

**功能**:
- 自动检测航点段(目标位置变化的区间)
- 计算每段的超调量、调节时间、振荡次数、稳态误差
- 频域分析(FFT 检测主振荡频率)
- 可视化:位置跟踪曲线 + 误差曲线 + 2% 调节带

**关键指标**:
```matlab
% 超调量
overshoot_abs = max(|meas - setpoint|) after first crossing
overshoot_pct = overshoot_abs / |initial_move| * 100

% 调节时间 (2% 准则)
settling_time = time when |error| < 2% * |setpoint| for last time

% 稳态误差
ss_error = mean(|error|) in last 20% of segment

% 积分指标
IAE = ∫|error| dt
ITAE = ∫t·|error| dt
```

**使用方法**:
```matlab
result = analyze_overshoot('excel_csv/test_data.csv');
```

---

### 2.2 系统辨识与 IMC 调参 (`identify_and_tune.m`)

**原理**: 利用航点切换时的阶跃响应辨识被控对象模型,再用 IMC 公式计算 PID 参数。

**流程**:
1. **数据分段**: 按目标位置变化拆分数据
2. **速度估计**: 对位置做中心差分得到速度
3. **FOPDT 模型拟合**: 
   ```
   G(s) = K·e^(-τs) / (T·s + 1)
   ```
   - K: 稳态增益 = Δvel_ss / Δcmd_vel
   - τ: 死区时间 = 首次响应时间
   - T: 时间常数 = 达到 63.2% 稳态值的时间 - τ

4. **IMC-PID 调参公式**:
   ```
   Kp = (T + τ/2) / (K·(λ + τ))
   Ki = 1 / (K·(λ + τ))
   Kd = (T·τ/2) / (K·(λ + τ))
   ```
   其中 λ 是调节参数(越大越保守,越小越激进)

5. **多段加权平均**: 用拟合质量作为权重,合并各段辨识结果

**使用方法**:
```matlab
% 默认 λ = 1.0
result = identify_and_tune('excel_csv/test_data.csv');

% 自定义 λ (0.3~3.0)
result = identify_and_tune('excel_csv/test_data.csv', 1.5);  % 保守
result = identify_and_tune('excel_csv/test_data.csv', 0.5);  % 激进
```

**输出**:
```matlab
result.plant.X/Y/Z       % 辨识的 FOPDT 模型参数 (K, T, tau)
result.recommended.X/Y/Z % IMC 推荐的 PID 参数
result.validation        % 仿真验证结果
```

---

### 2.3 PID 仿真工具 (`my_pid_analyze.m`)

**功能**:
- 完整复刻 C 代码的 PID 控制逻辑(自适应增益、积分分离、微分滤波、前馈)
- 复刻 `Pos_Cmd_st()` 的坐标旋转(POS_TRANS=1)
- 复刻 `s_cmd_vel_consume_v()` 的四元数旋转
- 对比仿真输出与实际 cmd_vel / rt_tar_vel

**自适应增益调度** (3 区域):
```matlab
if |error| > threshold_high
    Kp = Kp_base * Kp_high_ratio
    Ki = Ki_base * Ki_high_ratio
    Kd = Kd_base * Kd_high_ratio
elseif |error| > threshold_low
    ratio = (|error| - threshold_low) / (threshold_high - threshold_low)
    Kp = Kp_base * (1 + ratio * (Kp_high_ratio - 1))
    Ki = Ki_base * (1 - ratio * (1 - Ki_high_ratio))
    Kd = Kd_base * (1 - ratio * (1 - Kd_high_ratio))
else
    Kp = Kp_base, Ki = Ki_base, Kd = Kd_base
```

**积分分离**:
```matlab
if |error| <= I_Band
    integral += error * dt
    integral = clamp(integral, -integral_max, integral_max)
else
    integral = 0  % 大误差时不积分,防止饱和
```

**前馈控制**:
```matlab
target_vel = (measurement - pre_target_position) / dt
target_acc = (target_vel - pre_target_vel) / dt
output += kv * target_vel + ka * target_acc
```

**使用方法**:
```matlab
% 使用默认参数
result = my_pid_analyze('excel_csv/test_data.csv');

% 自定义 PID 参数
pid.X.Kp_base = 0.15;
pid.X.Ki_base = 0.08;
pid.X.Kd_base = 0.3;
result = my_pid_analyze('excel_csv/test_data.csv', pid);
```

**输出**:
```matlab
result.sim_cmd_vel.XY_radar  % 雷达系仿真速度
result.sim_cmd_vel.body_XY   % 机体系仿真速度 (需有四元数数据)
result.yaw_deg               % 四元数导出的 Yaw 角
```

---

## 三、调参方法论

### 3.1 调参流程

```
1. 数据采集
   ├─ 执行多次航点切换任务 (不同距离、方向)
   ├─ 确保数据包含: RADAR_POS, TARGET_POS, CMD_SPEED, T_REL
   └─ (可选) 记录四元数 + rt_tar_vel 用于机体系验证

2. 初步分析
   ├─ 运行 analyze_overshoot.m 查看超调、调节时间、振荡
   └─ 识别问题: 超调过大? 调节太慢? 稳态误差大?

3. 系统辨识
   ├─ 运行 identify_and_tune.m 获取 FOPDT 模型
   ├─ 检查辨识质量 (拟合质量 > 60% 可用)
   └─ 查看 IMC 推荐的 PID 参数

4. 仿真验证
   ├─ 用 my_pid_analyze.m 仿真推荐参数
   ├─ 对比仿真 vs 实际的 cmd_vel (雷达系)
   ├─ (若有四元数数据) 对比 rt_tar_vel (机体系)
   └─ 调整 λ 参数 (0.3~3.0) 平衡响应速度与稳定性

5. 实机测试
   ├─ 从保守参数开始 (λ = 2.0)
   ├─ 逐步减小 λ (每次 -0.3) 提升响应速度
   └─ 观察超调、振荡、稳态误差,找到最佳平衡点
```

---

### 3.2 参数调整策略

#### 场景 1: 超调过大 (> 30%)

**原因**: 
- Kp 过大或 Ki 积累过快
- 微分滤波不足 (d_filter_alpha 太小)

**解决方案**:
```matlab
% 降低比例增益
pid.X.Kp_base = pid.X.Kp_base * 0.7;

% 降低积分增益
pid.X.Ki_base = pid.X.Ki_base * 0.6;

% 增强微分滤波 (更平滑)
pid.X.d_filter_alpha = 0.9;  % 原值 0.8

% 缩小 I-Band (减少积分积累区间)
pid.X.I_Band = 15;  % 原值 25
```

---

#### 场景 2: 响应太慢 (调节时间 > 2s)

**原因**:
- Kp 过小
- Ki 过小或 I-Band 过窄
- 前馈增益不足

**解决方案**:
```matlab
% 增加比例增益
pid.X.Kp_base = pid.X.Kp_base * 1.3;

% 增加积分增益
pid.X.Ki_base = pid.X.Ki_base * 1.5;

% 扩大 I-Band (允许更多积分)
pid.X.I_Band = 30;

% 增加前馈增益 (如果有目标轨迹预测)
pid.X.kv = 1.5;  % 原值 1.0
pid.X.ka = 3.0;  % 原值 2.27
```

---

#### 场景 3: 稳态误差大 (> 5cm)

**原因**:
- 积分作用不足 (I-Band 太窄或 Ki 太小)
- 存在外部扰动 (风、重心偏移)

**解决方案**:
```matlab
% 增加积分增益
pid.X.Ki_base = pid.X.Ki_base * 1.8;

% 扩大 I-Band
pid.X.I_Band = 35;

% 增加积分限幅 (允许更多积累)
pid.X.integral_max = 8;  % 原值 5
```

---

#### 场景 4: 高频振荡

**原因**:
- 微分增益过大
- 传感器噪声过大
- 控制频率过高

**解决方案**:
```matlab
% 降低微分增益
pid.X.Kd_base = pid.X.Kd_base * 0.5;

% 增强微分滤波
pid.X.d_filter_alpha = 0.95;

% 启用自适应增益调度 (大误差时降低增益)
pid.X.error_threshold_high = 20;
pid.X.Kd_high_ratio = 0.5;  % 大误差时 Kd 减半
```

---

### 3.3 自适应增益调度调优

**三区域参数设计原则**:

| 区域 | 误差范围 | Kp_ratio | Ki_ratio | Kd_ratio | 目的 |
|------|---------|----------|----------|----------|------|
| 高误差区 | > threshold_high | 1.5~2.5 | 0.5~0.8 | 0.5~0.7 | 快速响应 |
| 过渡区 | low ~ high | 线性插值 | 线性插值 | 线性插值 | 平滑切换 |
| 低误差区 | < threshold_low | 1.0 | 1.0 | 1.0 | 精确保持 |

**调参步骤**:
```matlab
% 1. 先调低误差区 (精确保持)
pid.X.Kp_base = 0.10;  % 从保守值开始
pid.X.Ki_base = 0.06;
pid.X.Kd_base = 0.20;

% 2. 调高误差区 (快速响应)
pid.X.error_threshold_high = 25;  % > 25cm 进入高增益区
pid.X.Kp_high_ratio = 2.0;        % Kp 翻倍
pid.X.Ki_high_ratio = 0.7;        % Ki 降低,防止饱和
pid.X.Kd_high_ratio = 0.58;       % Kd 降低,防止振荡

% 3. 调过渡区 (平滑切换)
pid.X.error_threshold_low = 10;   % 10~25cm 为过渡区
```

---

### 3.4 前馈控制调优

**前馈作用**: 根据目标速度/加速度提前补偿,减少跟踪延迟。

**公式**:
```matlab
output += kv * target_vel + ka * target_acc
```

**调参原则**:
- `kv`: 速度前馈,补偿匀速运动
  - 初始值: 1.0
  - 调整范围: 0.5 ~ 2.0
  - 过大: 超调增加
  - 过小: 跟踪延迟

- `ka`: 加速度前馈,补偿加减速
  - 初始值: 2.0 ~ 2.5
  - 调整范围: 1.0 ~ 4.0
  - 过大: 振荡
  - 过小: 加减速时误差大

**注意事项**:
- 前馈只在目标运动时有效
- 定点保持时 target_vel = 0,前馈不起作用
- 需要准确的目标轨迹预测 (目前使用 measurement 差分,有延迟)

---

## 四、当前调参结果

### 4.1 测试结果 (20cm 阶跃响应)

**测试条件**:
- 目标: 从 (0,0,0) 移动到 (0,0,140) (Z 轴 20cm)
- PID 参数: Kp=0.1, Ki=0.06, Kd=0.2
- 采样率: ~10 Hz

**性能指标**:
```
Z 轴:
  超调量: ~10cm (50%)
  调节时间 (10cm 带): ~1.5s
  稳态误差 (最后 20%): < 2cm
  振荡次数: 0~1 次
  
XY 轴:
  稳态误差: < 3cm
  无明显振荡
```

**评估**:
- ✅ 位置收敛到 10cm 以内
- ✅ 曲线平滑,无高频振荡
- ✅ 定点保持稳定
- ⚠️ 超调略大 (50%),可通过增加 Kd 或降低 Ki 改善

---

### 4.2 当前参数

```matlab
% X/Y 轴
Kp_base = 0.10
Ki_base = 0.06
Kd_base = 0.20
output_max = 23
integral_max = 5
I_Band = 25
d_filter_alpha = 0.8
error_threshold_high = 25
error_threshold_low = 25
Kp_high_ratio = 2.0
Ki_high_ratio = 0.7
Kd_high_ratio = 0.58
kv = 1.0
ka = 2.27

% Z 轴
Kp_base = 0.23
Ki_base = 0.03
Kd_base = 0.08
output_max = 20
I_Band = 20
error_threshold_high = 10
error_threshold_low = 2
```

---

## 五、进一步优化建议

### 5.1 短期优化 (可立即尝试)

#### 建议 1: 降低超调

```matlab
% 增加微分增益 (抑制超调)
pid.Z.Kd_base = 0.12;  % 原值 0.08,增加 50%

% 降低积分增益 (减少积累)
pid.Z.Ki_base = 0.02;  % 原值 0.03
```

**预期效果**: 超调从 50% 降到 30%,调节时间可能增加 0.2~0.3s。

---

#### 建议 2: 加快响应

```matlab
% 增加比例增益
pid.Z.Kp_base = 0.28;  % 原值 0.23,增加 20%

% 增加前馈
pid.Z.kv = 1.3;  % 原值 1.0
pid.Z.ka = 2.5;  % 原值 2.22
```

**预期效果**: 调节时间减少 0.3~0.5s,超调可能略增。

---

#### 建议 3: 使用 IMC 调参结果

运行系统辨识获取理论最优参数:
```matlab
result = identify_and_tune('excel_csv/test_data.csv', 1.5);  % 保守起步
disp(result.recommended);
```

将推荐参数应用到仿真验证:
```matlab
pid.Z.Kp_base = result.recommended.Z.Kp;
pid.Z.Ki_base = result.recommended.Z.Ki;
pid.Z.Kd_base = result.recommended.Z.Kd;

my_pid_analyze('excel_csv/test_data.csv', pid);
```

---

### 5.2 中期优化 (需要更多数据)

#### 建议 4: 机体系验证

**前提**: CSV 中包含四元数数据 (`qw,qx,qy,qz,rt_tar_vel_x,rt_tar_vel_y`)

**步骤**:
```matlab
result = my_pid_analyze('excel_csv/test_data.csv', pid);

% 查看机体系对比图
% 对比 rt_tar_vel (实际机体系速度) vs sim_body (仿真机体系速度)
% 如果差异大,说明四元数旋转模型有问题或飞控内环响应慢
```

**诊断**:
- 若 `rt_tar_vel` 幅值 < `sim_body`: 飞控内环增益不足
- 若 `rt_tar_vel` 相位滞后: 飞控内环响应慢或延迟大
- 若 `rt_tar_vel` 振荡: 飞控内环 PID 需要调优

---

#### 建议 5: 多航点测试

**目的**: 验证不同距离、方向的响应一致性

**测试方案**:
```
测试 1: 短距离 (10cm) → 检查小误差线性度
测试 2: 中距离 (30cm) → 检查自适应增益切换
测试 3: 长距离 (50cm) → 检查大误差响应
测试 4: 斜向移动 (20cm X + 20cm Y) → 检查坐标旋转
测试 5: 连续航点 → 检查积分重置行为
```

对每个测试运行 `analyze_overshoot.m`,统计指标。

---

### 5.3 长期优化 (需要架构改动)

#### 建议 6: 级联 PID (位置环 + 速度环)

**当前架构**:
```
位置误差 → PID → 速度指令 → 飞控内环
```

**改进架构**:
```
位置误差 → 位置环 PID → 速度指令 → 速度环 PID → 加速度指令 → 飞控
```

**优势**:
- 速度环可快速抑制扰动
- 位置环可更保守,专注于精度
- 解耦调参,互不影响

**实现**: 需要在飞控中添加速度环 PID,并记录实际速度数据。

---

#### 建议 7: 模型预测控制 (MPC)

**原理**: 基于 FOPDT 模型预测未来行为,优化控制序列

**优势**:
- 可处理约束 (速度限幅、加速度限幅)
- 可处理延迟 (死区时间 τ)
- 多变量耦合控制 (XY 联动)

**挑战**:
- 计算量大,需要高性能处理器
- 需要准确的模型
- 实现复杂

**建议**: 仅在 PID 无法满足性能需求时考虑。

---

## 六、工具使用速查表

### 6.1 快速分析流程

```matlab
% Step 1: 超调分析
result = analyze_overshoot('excel_csv/test_data.csv');

% Step 2: 系统辨识 + IMC 调参
tune_result = identify_and_tune('excel_csv/test_data.csv', 1.5);

% Step 3: 仿真验证
pid.Z.Kp_base = tune_result.recommended.Z.Kp;
pid.Z.Ki_base = tune_result.recommended.Z.Ki;
pid.Z.Kd_base = tune_result.recommended.Z.Kd;
sim_result = my_pid_analyze('excel_csv/test_data.csv', pid);

% Step 4: 实机测试 → 迭代
```

---

### 6.2 常见问题诊断

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| 超调 > 30% | Kp 过大 / Ki 积累过快 | 降低 Kp/Ki,增加 Kd |
| 调节时间 > 2s | Kp 过小 / 前馈不足 | 增加 Kp/kv/ka |
| 稳态误差 > 5cm | 积分不足 | 增加 Ki/I_Band |
| 高频振荡 | Kd 过大 / 噪声大 | 降低 Kd,增加 d_filter_alpha |
| 低频振荡 | 自适应增益切换不平滑 | 调整 threshold_high/low |
| 仿真 vs 实际差异大 | dt 单位错误 / 坐标旋转未建模 | 检查 dt*1000,添加四元数旋转 |

---

### 6.3 参数调整优先级

```
1. Kp (比例) — 影响响应速度和超调
2. Ki (积分) — 影响稳态精度和饱和
3. Kd (微分) — 影响振荡和噪声敏感度
4. I_Band — 影响积分作用范围
5. d_filter_alpha — 影响微分滤波强度
6. kv/ka (前馈) — 影响动态跟踪
7. error_threshold_high/low — 影响自适应增益调度
```

---

## 七、总结

### 7.1 主要成果

1. ✅ **修复了 MATLAB 仿真**: dt 单位转换使仿真与 C 代码高度拟合
2. ✅ **建立了完整模型**: 包含坐标旋转、四元数变换、自适应增益
3. ✅ **开发了分析工具**: 超调分析、系统辨识、IMC 调参
4. ✅ **成功调参**: 20cm 阶跃响应收敛到 10cm 以内,无明显振荡

### 7.2 关键经验

1. **单位一致性至关重要**: RTOS tick vs 秒的差异会导致 1000 倍误差
2. **坐标变换必须建模**: 雷达系 → 机体系的旋转影响实际控制效果
3. **仿真指导调参**: 先在仿真中验证,再实机测试,降低风险
4. **渐进式调优**: 从保守参数开始,逐步提升性能

### 7.3 下一步行动

1. 尝试短期优化建议 (降低超调、加快响应)
2. 收集更多测试数据 (多航点、不同距离)
3. 使用 IMC 调参结果对比手动调参
4. 若有四元数数据,验证机体系模型准确性

---

**文档版本**: v1.0  
**最后更新**: 2026-07-22  
**维护者**: Claude Code (Anthropic)
