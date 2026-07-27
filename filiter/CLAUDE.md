# 滤波器模块 — my_flitter.m

## 概述

`my_flitter.m` 处理雷达原始 CSV 日志：去重（按 Tick 合并重复行）、重构列顺序、输出标准 22 列格式。

## 入口分发

```
my_flitter(csvPath, options)
├── isfolder(csvPath) → batchProcess(dirPath, options)   // 批量
└── isfile(csvPath)   → processOneFile(csvPath, options)  // 单个
```

## 核心函数

### `processOneFile(csvPath, options)` — 单文件处理

1. **options 合并**：用 `options` 覆盖 `opts` 的 `output_dir` / `output_name`，无关字段（`X`、`Y` 等 PID 参数）不拷入
2. **loadRawCsv** → 读取原始 CSV（20 列，跳过第 1 行数字头，第 2 行解析列名，第 3 行起数据）
3. **deduplicateByTick** → 按第 20 列（Tick）分组去重
4. **构造 22 列输出矩阵**：

   | 列范围 | 名称 | 来源 |
   |--------|------|------|
   | 1 | T_REL | tick - tick(1) |
   | 2 | T_ABS | tick（原始绝对时间） |
   | 3-5 | RADAR_POS_X/Y/Z | 原始列 1-3 |
   | 6-8 | TARGET_POS_X/Y/Z | 原始列 4-6 |
   | 9-11 | CMD_SPEED_X/Y/Z | 原始列 7-9 |
   | 12-14 | ERROR_X/Y/Z | target - radar |
   | 15-18 | qx/qy/qz/qw | 原始列 10-13 |
   | 19-20 | fc_sen_vel_x/y | 原始列 14-15 |
   | 21-22 | rt_tar_vel_x/y | 原始列 17-18 |

5. **saveOutput** → 写入 CSV

### `batchProcess(dirPath, options)` — 批量处理

- 遍历文件夹下所有 `.csv`
- `isRawFormat()` 检测：读第 2 行是否包含 `U1:` → 是则原始格式，否则跳过
- 输出路径：`dirPath/<文件名>/<文件名>.csv`（各自子文件夹，同名文件）
- 返回结果结构体数组：`file, output_path, data, row_raw_num, row_de_num, output_re_num`

### `isRawFormat(csvPath)`
打开文件，跳过第 1 行，检查第 2 行是否包含 `U1:`。

### `loadRawCsv(csvPath)`
- 跳过第 1 行（数字 header）
- 第 2 行为列名 → `strsplit` 解析
- `detectImportOptions` + `readmatrix` 从第 3 行开始读
- 清除全 NaN 行

### `deduplicateByTick(data_raw, tickCol)`
- 按 Tick 稳定分组
- 每组若有 `CMD_SPEED > 0.5` 的行，取第一个速度行；否则取最后一行
- 排序保留原始顺序

### `saveOutput(mat, col_name, csv_path_raw, opts)`
- 输出目录：`opts.output_dir`（若空则取源文件目录）
- 输出文件名：`opts.output_name` + `.csv`（若空则为 `原文件名_filtered.csv`）
- 自动创建输出文件夹
- `writetable` 写入

## 重要约定

- **原始 CSV 格式**：第 1 行数字、第 2 行含 `U1:` 列名、第 3 行起数据、20 列
- **过滤后 CSV 格式**：第 1 行列名（无 `U1:` 前缀）、22 列
- **options 参数**：仅 `output_dir`（字符串）和 `output_name`（字符串）有效；`X`/`Y` 等 PID 参数会被忽略
- **矩阵索引硬编码**：依赖原始 20 列固定排列（雷达 xyz 1-3, 目标 4-6, CMD 7-9, 四元数 10-13, fc_sen 14-15, 跳 16, rt_tar 17-18, 跳 19, Tick 20）
