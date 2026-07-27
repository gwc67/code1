# 轨迹跟踪模块 — 3D 可视化

## 文件

- `plot_radar_3d.m` — 静态 3D 轨迹图 + 动态悬停箭头
- `play_radar_3d_2.m` — 3D 动画播放器（无人机模型 + 速度箭头 + 播放控制）

---

## plot_radar_3d.m

### 功能

绘制雷达轨迹（蓝线）和目标点（红色散点），鼠标悬停在雷达轨迹上时显示速度箭头。

### 依赖的 CSV 列

| 列名 | 必需/可选 | 用途 |
|------|-----------|------|
| RADAR_POS_X/Y/Z | **必需** | 轨迹线 |
| TARGET_POS_X/Y/Z | **必需** | 目标点 |
| CMD_SPEED_X/Y/Z | 可选 | 蓝色 CMD 速度箭头 |
| rt_tar_vel_x/y | 可选 | 红色 RT_TAR 速度箭头 |
| fc_sen_vel_x/y | 可选 | 品红 FC_SEN 传感器速度箭头 |
| qw/qx/qy/qz | 可选 | 四元数 → YAW → 青绿机头箭头 |

### 可选参数（`'参数名', 值` 对）

```
'LineWidth'    雷达轨迹线宽 (默认 2.5)
'MarkerSize'   目标点大小 (默认 80)
'RadarColor'   轨迹颜色 (默认 'b')
'TargetColor'  目标颜色 (默认 [0.8 0 0])
'ShowGrid'     网格开关 (默认 true)
'EqualAxis'    等比例轴 (默认 true)
'Title'        标题
'ShowYaw'      YAW 箭头开关 (默认 true)
'YawInterval'  箭头间隔 (默认 auto)
'YawArrowScale' YAW 箭头长度比例 (默认 0.02)
```

### 坐标系约定

| 箭头 | 坐标系 | 逻辑 |
|------|--------|------|
| CMD（蓝） | 世界系 | 直接使用 `atan2(cmd_y, cmd_x)` |
| RT_TAR（红） | 机头系 → 旋转 → 世界系 | `rt_w = R(yaw) * [rt_bx; rt_by]` 显示 |
| FC_SEN（品红） | 机头系 → 旋转 → 世界系 | `fc_w = R(yaw) * [fc_bx; fc_by]` 显示 |
| YAW（青绿） | 世界系 | `atan2(2*(qw*qz + qx*qy), 1 - 2*(qy²+qz²))` |

### 悬停机制（WindowButtonMotionFcn）

1. `onRadarHover(fig)` 在鼠标移动时触发
2. 获取 `ax.CurrentPoint`（视线射线）
3. 向量化计算所有轨迹点到视线的垂直距离
4. 最近点距离 < `hoverThreshold`（`span_xy * 0.03`）→ 显示箭头
5. 切换到不同点才更新（`lastIdx` 缓存），减少冗余 set()
6. 离开轨迹 → 隐藏箭头

### 箭头缩放

- 速度箭头长度 = `sqrt(vx² + vy²) * span_xy * 0.005`
- YAW 固定长度 = `span_xy * 0.02`
- 标注偏移 = `span_xy * 0.06`

### 速度标注

单行 TeX 彩色文本，格式：
```
\color[rgb]{0,0,1}CMD:+1.23  |  \color[rgb]{1,0,0}RT:-0.45  |  \color[rgb]{1,0,0.6}FCS:+0.67
```

符号取自 X 分量（纵向），物理含义为前进/后退方向。

---

## play_radar_3d_2.m

### 功能

完整的 3D 轨迹动画播放器，带无人机模型和速度箭头。

### 核心实现

- **轨迹**：`animatedline` 高性能累加绘制，支持拖尾（TrailLength）
- **无人机**：`hgtransform` 驱动 patch 模型（机身 + 机头三角 + RGB 三轴），四元数旋转矩阵驱动姿态
- **速度箭头**：`line`（箭身）+ `patch`（锥体头部），`hgtransform` 统一管理
- **UI 控制**：播放/暂停、单步前进后退、进度条、速度倍率

### 关键模式：fig.UserData 共享状态

MATLAB 回调中 struct 传值为副本，**必须在每次修改后写回**：
```matlab
ud = fig.UserData;
ud.currentIdx = ud.currentIdx + 1;
fig.UserData = ud;            % ← 关键：写回
```

`fig.UserData` 中缓存的字段：
```
isPlaying, currentIdx, timer, playSpeed
rx/ry/rz, yaw_rad, arrowBaseLen, span
hasCmd, hasRt, hasFc, hasQuat
cmd_x/y, rt_x/y, fc_x/y, qw/qx/qy/qz
h_cmd, h_rt, h_fc, h_yaw  (结构体: .shaft .head)
txt_info, trail, t_uav, t_vel
lbl_frame, N, fps
```

### 坐标系（与 plot_radar_3d 一致）

| 箭头 | 颜色 | 变换 |
|------|------|------|
| CMD | [0.2 0.4 1] 蓝 | 世界系，直接 |
| RT_TAR | [1 0.2 0.2] 红 | 机头系 → yaw 旋转 → 世界系 |
| FC_SEN | [1 0 0.6] 品红 | 机头系 → yaw 旋转 → 世界系 |
| YAW | [0 0.8 0.5] 青绿 | `[cos(yaw), sin(yaw), 0]` |

### 3D 箭头构造（`buildArrow3D` + `updateArrow3D`）

- `updateArrow3D(h, origin, direction, baseLen, headFrac)`
- 箭身：从 `origin` 到 `tip - dir * headLen` 的 line
- 锥体头部：4 个底面点 + 1 个尖端点的 patch
- 方向归一化，总长度 = `baseLen * norm(direction)`
- 速度为零时自动隐藏

### 回调函数

| 函数 | 触发 | 行为 |
|------|------|------|
| `onPlayPause` | Play 按钮 | 切换 isPlaying，启动/停止 timer |
| `timerCallback` | Timer 周期 | currentIdx++，updateFrame，更新滑动条 |
| `onSliderChange` | 进度条 | 跳转指定帧 |
| `onStep` | ◀◀ / ▶▶ | currentIdx ± 1 |
| `onSpeedChange` | 速度下拉 | 修改 timer.Period |
