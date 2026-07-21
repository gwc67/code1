# function/pid/ — PID 性能分析模块

## 文件清单

| 文件 | 用途 |
|---|---|
| `analyze_pid_performance.m` | 主分析函数（1047 行，14 个内部函数） |
| `analyze_pid_performance_v0.m` | 旧版启发式备份 |
| `CLAUDE.md` | 本文件 |

## 主函数签名

```matlab
result = analyze_pid_performance(csvPath, pidParams, tuningMode)
```

- `csvPath`: CSV 路径（支持 raw 20 列 和 clean 14 列自动检测）
- `pidParams`: 可选 struct，部分覆盖 PID 参数
- `tuningMode`: `'balanced'`(默认) / `'aggressive'` / `'conservative'`

## 10 个切片（S1-S10）

| 切片 | 函数 | 功能 |
|---|---|---|
| S1 | `printConsoleReport`, `plotAnalysis`, `saveResult` | 输出报告 + 3x3 图表 + .mat |
| S2 | `getDefaultPidParams`, `mergePidParams` | C 代码默认值 + 用户覆盖合并 |
| S3 | `loadCsvData` → `loadCleanFormat` / `loadRawFormat` | 自动识别 CSV 格式 |
| S4 | `checkDataQuality` | 激励充足性门控 |
| S5 | `simulatePlant` | 被控对象正向仿真 G(s)=Ke^(-τs)/(s(Ts+1)) |
| S6 | `identifyPlant`, `estimateInitialGuess`, `plantResidual` | lsqnonlin 被控对象辨识 |
| S7 | `imcPid` | IMC 解析公式 |
| S8 | `tunePid`, `itaeCost`, `pidConstraints`, `computeMetrics` | IMC+ITAE 组合调参 |
| S9 | `pidSimulate` | **复刻 C 代码 `PID_Update_l`** |
| S10 | `xyRotateAndPid` | **复刻 fc_ctrl.c `POS_TRANS=1` 坐标旋转** |

## 分析流程（6 步）

```
[1/6] 读取数据 → loadCsvData
[2/6] 数据质量检查 → checkDataQuality
[3/6] Part A: PID 仿真对比（pidSimulate vs 实际 cmd_vel）
[4/6] 稳态分析（analyzeSteadyState）
[5/6] Part B: 被控对象辨识 + IMC/ITAE 调参（仅激励充足时）
[6/6] 输出报告 + 3x3 图表 + .mat 保存
```

## 输出结构

```
result.pid_validation.X/Y/Z  — cmd_vel_sim, nrmse, correlation
result.steady_state.X/Y/Z    — mean_error, max_error, Kp_eff, small_overshoot
result.recommended.X/Y/Z     — Kp, Ki, Kd (Part B 辨识成功时)
result.identified_plant       — tf 对象 + K/T/tau 参数
result.metrics.X/Y/Z          — overshoot, settling_time, phase_margin
result.model_fit.NRMSE        — 辨识模型 NRMSE
```

## C 代码对应关系

### 源文件

- **PID_ctrl.c**: `PID_Update_l()` — 核心 PID 计算
- **fc_ctrl.c**: `Pos_Cmd_st()` — 上层调度 + POS_TRANS=1 坐标旋转

### pidSimulate 与 PID_Update_l 逐行对比

| 模块 | C 代码行号 | MATLAB 行号 | 一致? |
|---|---|---|---|
| 误差计算 | 209 | 703 | ✅ |
| dt 来源 | `xTaskGetTickCount()` 差值(213-216) | 固定 `dt` 参数 | ✅ 等效 |
| 三区域自适应增益 | 223-244 | 707-721 | ✅ |
| 积分分离 + 限幅 | 247-255 | 724-729 | ✅ |
| 微分低通滤波 | 257-259 | 732-734 | ✅ |
| PID 输出 | 261 | 737 | ✅ |
| 前馈 kv*vel + ka*acc | 263-265 | 740-742 | ⚠️ 见下方 |
| 输出限幅 + 抗饱和 | 267-279 | 745-751 | ✅ |
| 状态更新 | 281-284 | 754-757 | 🔴 见下方 |

### 🔴 已知差异：`pre_target_position` 存储值

```
C 代码 (第283行):   pre_target_position_f = measurement_f;   ← 存测量值
MATLAB (第756行):   state.pre_target_position = setpoint;     ← 存设定值
```

效果：前馈 target_vel 计算不同：
- C: `target_vel = (measurement[k] - measurement[k-1]) / dt` → 实际速度估计
- MATLAB: `target_vel = (measurement[k] - setpoint[k-1]) / dt` → 混合量

**如需忠实复刻 C 代码，第 756 行应改为 `state.pre_target_position = measurement;`**

### xyRotateAndPid 与 Pos_Cmd_st (POS_TRANS=1) 对比

| C 代码 | MATLAB | 一致? |
|---|---|---|
| 目标变化检测 (第199行) | `any(rot_state.last_target ~= target_pos_xy)` | ✅ |
| 起点重设 (第201-202行) | `rot_state.start_point = rot_state.last_target` | ✅ |
| theta_1 = atan2(delta_y, delta_x) | `atan2(delta(2), delta(1))` | ✅ |
| setpoint_modulus = 距离 | `norm(delta)` | ✅ |
| PID reset (第219-220行) | `rot_state.state_X = []` | ✅ |
| theta_2 = atan2(pos_y, pos_x) | `atan2(un_trans_pos(2), un_trans_pos(1))` | ✅ |
| length_x = pos_mod * cos(θ₁-θ₂) | `pos_modulus * cos(theta_1 - theta_2)` | ✅ |
| length_y = pos_mod * sin(θ₁-θ₂) | `pos_modulus * sin(theta_1 - theta_2)` | ✅ |
| X PID(setpoint_modulus, length_x) | `pidSimulate(pid_X, setpoint_modulus, length_x, ...)` | ✅ |
| Y PID(0, length_y) | `pidSimulate(pid_Y, 0, length_y, ...)` | ✅ |
| 反变换 (第249-250行) | 第 833-834 行 | ✅ |

## 默认 PID 参数（来自 PID_Init）

| 参数 | X/Y 轴 | Z 轴 |
|---|---|---|
| Kp_base | 0.18 | 0.23 |
| Ki_base | 0.2 | 0.03 |
| Kd_base | 0.5 | 0.08 |
| output_max/min | ±23 | ±20 |
| integral_max | 5 | 5 |
| I_Band | 25 | 20 |
| error_threshold_high | 25 | 10 |
| error_threshold_low | 25 | 2 |
| Kp_high_ratio | 2.0 | 2.0 |
| Ki_high_ratio | 0.7 | 0.7 |
| Kd_high_ratio | 0.58 | 0.7 |
| d_filter_alpha | 0.8 | 0.8 |
| kv | 1.0 | 1.0 |
| ka | 2.27 | 2.22 |

## 调用示例

```matlab
addpath('function/pid');

% 基础调用
result = analyze_pid_performance('data/flight_log.csv');

% 自定义参数
pid.X.Kp_base = 0.25;
pid.X.plant_init = [2.0, 0.3, 0.05];  % [K0, T0, tau0]
result = analyze_pid_performance('data/flight_log.csv', pid, 'aggressive');
```
