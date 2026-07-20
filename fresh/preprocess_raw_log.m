function result = preprocess_raw_log(rawCsvPath, options)
% PREPROCESS_RAW_LOG  将飞控原始日志 CSV 转换为干净的分析数据
%
% 用法:
%   result = preprocess_raw_log(rawCsvPath)
%   result = preprocess_raw_log(rawCsvPath, options)
%
% 输入:
%   rawCsvPath  - 原始 CSV 文件路径
%   options     - 可选 struct:
%       options.timeRange     - [t_start, t_end] 秒 (相对时间)，默认全段
%       options.interactive   - true/false，是否弹出交互图让用户框选时间
%       options.output_dir    - 输出目录，默认与 raw 同目录
%       options.include_radar_speed - true/false, 是否包含 U10-U12 雷达速度
%       options.include_yaw   - true/false, 是否包含 YAW 相关列
%       options.invalid_values - 哨兵值列表，默认 [] (不替换，保留原始数据)
%                                设为 [-32768, -1] 会替换为 NaN
%       options.uniform_dt     - 均匀时间间隔 (s)，默认 0.01 (10ms)
%                                设为 [] 表示不补齐，保留原始时间点
%
% 输出:
%   result - struct:
%       result.clean_csv_path    - clean CSV 文件路径 (全采样率)
%       result.radar_sync_path   - 雷达同步 CSV 路径 (仅 radar_pos 变化行)
%       result.metadata_path     - 元数据 .mat 路径
%       result.metadata          - 元数据 struct (采样率、时间范围、列说明等)
%       result.full_data         - 全量数据矩阵 (Nx13+)
%       result.radar_sync_data   - 雷达同步数据 (Mx13+)
%       result.column_names      - 列名 cell array
%
% 原始 CSV 格式:
%   第 1 行: 数字 header (1,2,3,...,20,)
%   第 2 行: 列名 header (U1:RADAR_POS_X, ...)
%   第 3 行起: 数据
%   U20 列: Tick (秒, MCU 启动后绝对时间)

    %% 0. 输入校验与默认选项
    if nargin < 1 || isempty(rawCsvPath)
        error('rawCsvPath 不能为空');
    end
    % 强制转为 char，避免 string 对象在后续 fileparts/fullfile/fopen 中出问题
    rawCsvPath = char(rawCsvPath);
    if nargin < 2 || isempty(options)
        options = struct();
    end

    opts = fillDefaultOptions(options);

    %% 1. 加载原始 CSV
    fprintf('\n[1/6] 加载原始 CSV: %s\n', rawCsvPath);
    [rawData, rawColNames] = loadRawCsv(rawCsvPath);
    fprintf('   原始数据: %d 行 x %d 列\n', size(rawData, 1), size(rawData, 2));

    %% 2. 提取 Tick 列，计算时间
    fprintf('[2/6] 处理时间戳...\n');
    tickIdx = findTickColumn(rawColNames);
    if isempty(tickIdx)
        error('未找到 Tick 列 (U20)。请确认 CSV 包含时间戳列。');
    end
    tick_abs = rawData(:, tickIdx);  % 绝对时间 (秒)
    % 跳过前导零值（MCU 启动初始化阶段 Tick=0 不是真实时间戳）
    first_positive = find(tick_abs > 0.1, 1);  % >100ms 视为有效
    if isempty(first_positive)
        error('Tick 列全为 0 或无效值，无法建立时间基准');
    end
    tick_start = tick_abs(first_positive);
    t_rel = tick_abs - tick_start;  % 相对时间（第一个有效 Tick 为 0）
    fprintf('   Tick 起点: %.3f s (第 %d 行)\n', tick_start, first_positive + 2);
    fprintf('   Tick 范围: %.3f - %.3f s (相对: %.3f - %.3f s)\n', ...
        tick_abs(first_positive), tick_abs(end), t_rel(first_positive), t_rel(end));

    %% 3. 处理哨兵值
    if isempty(opts.invalid_values)
        fprintf('[3/6] 保留原始数据（未指定哨兵值替换）\n');
        cleanData = rawData;
    else
        fprintf('[3/6] 替换哨兵值 (%s) 为 NaN...\n', mat2str(opts.invalid_values));
        cleanData = replaceInvalidValues(rawData, opts.invalid_values);
        nan_pct = sum(isnan(cleanData(:))) / numel(cleanData) * 100;
        fprintf('   NaN 占比: %.2f%%\n', nan_pct);
    end

    %% 4. 构造 clean 数据（13 列核心）
    fprintf('[4/6] 构造 clean 数据...\n');
    [cleanMat, colNames] = buildCleanData(cleanData, rawColNames, t_rel, tick_abs, opts);
    fprintf('   clean 数据: %d 行 x %d 列\n', size(cleanMat, 1), size(cleanMat, 2));
    fprintf('   列名: %s\n', strjoin(colNames, ', '));

    %% 5. 时间范围过滤
    if opts.interactive
        fprintf('[5/6] 交互式时间选择...\n');
        timeRange = interactiveTimeSelect(cleanMat(:, 1), cleanMat(:, 2:4), colNames);
        if ~isempty(timeRange)
            opts.timeRange = timeRange;
        end
    end

    if ~isempty(opts.timeRange)
        fprintf('[5/6] 按时间范围过滤: [%.2f, %.2f] s...\n', opts.timeRange(1), opts.timeRange(2));
        [cleanMat, keepIdx] = filterByTimeRange(cleanMat, opts.timeRange);
        fprintf('   过滤后: %d 行 (原 %d 行)\n', size(cleanMat, 1), size(cleanMat, 1) + sum(~keepIdx));
    else
        fprintf('[5/6] 全时间段（未指定过滤）\n');
    end

    %% 5.5 补齐到均匀时间网格
    if ~isempty(opts.uniform_dt) && opts.uniform_dt > 0
        fprintf('[5.5/6] 补齐到均匀 dt = %.4f s（去重 + 前值填充缺失点）...\n', opts.uniform_dt);
        [cleanMat, fillStats] = fillToUniformGrid(cleanMat, opts.uniform_dt);
        fprintf('   补齐后: %d 行 (填充了 %d 个缺失时间点)\n', ...
            size(cleanMat, 1), fillStats.filled);
    end

    %% 6. 检测雷达刷新行 + 保存
    fprintf('[6/6] 检测雷达刷新 + 保存...\n');
    radar_sync_mask = detectRadarRefresh(cleanMat, colNames);
    radar_sync_mat = cleanMat(radar_sync_mask, :);
    fprintf('   雷达刷新行: %d / %d (%.1f%%)\n', sum(radar_sync_mask), size(cleanMat, 1), ...
        sum(radar_sync_mask)/size(cleanMat,1)*100);

    %% 7. 保存输出
    [outClean, outRadar, outMeta, metadata] = saveCleanData(cleanMat, radar_sync_mat, ...
        colNames, rawCsvPath, opts, tick_start);

    %% 8. 构造返回值
    result.clean_csv_path = outClean;
    result.radar_sync_path = outRadar;
    result.metadata_path = outMeta;
    result.metadata = metadata;
    result.full_data = cleanMat;
    result.radar_sync_data = radar_sync_mat;
    result.column_names = colNames;

    fprintf('\n✅ 预处理完成\n');
    fprintf('   Clean CSV:     %s\n', outClean);
    fprintf('   Radar-sync CSV: %s\n', outRadar);
    fprintf('   Metadata:       %s\n', outMeta);
end


%% =========================================================================
%  选项默认值
% =========================================================================
function opts = fillDefaultOptions(userOpts)
    opts.timeRange = [];
    opts.interactive = false;
    opts.output_dir = '';
    opts.include_radar_speed = false;
    opts.include_yaw = false;
    opts.invalid_values = [];  % 默认保留原始数据（不替换）
    opts.uniform_dt = 0.01;    % 均匀时间间隔 (s)，默认 10ms；设为 [] 表示不补齐

    % 覆盖用户指定的字段
    flds = fieldnames(userOpts);
    for i = 1:length(flds)
        opts.(flds{i}) = userOpts.(flds{i});
    end
end


%% =========================================================================
%  加载原始 CSV
% =========================================================================
function [data, colNames] = loadRawCsv(csvPath)
    % 读列名 (第 2 行)
    fid = fopen(csvPath, 'r');
    if fid == -1
        error('无法打开文件: %s', csvPath);
    end
    % 跳过第 1 行（数字 header）
    fgetl(fid);
    % 读第 2 行（列名 header）
    headerLine = fgetl(fid);
    fclose(fid);

    % 解析列名
    colNames = strsplit(headerLine, ',');
    colNames = strtrim(colNames);
    colNames(cellfun(@isempty, colNames)) = [];  % 去末尾空列

    % 读数据（第 3 行起）
    nCols = length(colNames);
    opts = detectImportOptions(csvPath, 'NumVariables', nCols);
    opts.DataLines = [3, Inf];
    opts = setvartype(opts, 1:nCols, 'double');
    data = readmatrix(csvPath, opts);

    % 去全 NaN 行
    data(all(isnan(data), 2), :) = [];
end


%% =========================================================================
%  查找 Tick 列
% =========================================================================
function idx = findTickColumn(colNames)
    % 按名字匹配（"Tick" 或 "U20"）
    idx = [];
    for i = 1:length(colNames)
        name = colNames{i};
        if contains(name, 'Tick', 'IgnoreCase', true) || ...
           contains(name, 'U20', 'IgnoreCase', true)
            idx = i;
            return;
        end
    end
    % fallback: 如果列数正好 20，取最后一列
    if length(colNames) >= 20
        idx = 20;
    end
end


%% =========================================================================
%  替换无效值
% =========================================================================
function clean = replaceInvalidValues(raw, sentinels)
    clean = raw;
    for i = 1:length(sentinels)
        % 用容差比较（处理可能的浮点表示问题）
        clean(abs(clean - sentinels(i)) < 0.5) = NaN;
    end
end


%% =========================================================================
%  构造 clean 数据矩阵
% =========================================================================
function [mat, colNames] = buildCleanData(cleanData, rawColNames, t_rel, t_abs, opts)
    % 核心列索引（基于原始 CSV 的列位置）
    idxRadarPos = findColumnsByPrefix(rawColNames, 'U1', 'U2', 'U3');
    idxTargetPos = findColumnsByPrefix(rawColNames, 'U4', 'U5', 'U6');
    idxCmdSpeed = findColumnsByPrefix(rawColNames, 'U7', 'U8', 'U9');

    % 兜底：如果名字匹配失败，按位置取前 9 列
    if length(idxRadarPos) < 3, idxRadarPos = 1:3; end
    if length(idxTargetPos) < 3, idxTargetPos = 4:6; end
    if length(idxCmdSpeed) < 3, idxCmdSpeed = 7:9; end

    radar_pos = cleanData(:, idxRadarPos);
    target_pos = cleanData(:, idxTargetPos);
    cmd_vel = cleanData(:, idxCmdSpeed);
    error = target_pos - radar_pos;

    % 基础 13 列
    mat = [t_rel, t_abs, radar_pos, target_pos, cmd_vel, error];
    colNames = {'T_REL', 'T_ABS', ...
                'RADAR_POS_X', 'RADAR_POS_Y', 'RADAR_POS_Z', ...
                'TARGET_POS_X', 'TARGET_POS_Y', 'TARGET_POS_Z', ...
                'CMD_SPEED_X', 'CMD_SPEED_Y', 'CMD_SPEED_Z', ...
                'ERROR_X', 'ERROR_Y', 'ERROR_Z'};

    % 可选：雷达速度
    if opts.include_radar_speed
        idxRadarSpd = findColumnsByPrefix(rawColNames, 'U10', 'U11', 'U12');
        if length(idxRadarSpd) == 3
            mat = [mat, cleanData(:, idxRadarSpd)];
            colNames = [colNames, {'RADAR_SPD_X', 'RADAR_SPD_Y', 'RADAR_SPD_Z'}];
        end
    end

    % 可选：YAW 相关
    if opts.include_yaw
        % RADAR_YAW, CMD_YAWDPS, TARGET_YAW (位置按原 CSV 顺序)
        idxYaw = findYawColumns(rawColNames);
        if ~isempty(idxYaw)
            mat = [mat, cleanData(:, idxYaw)];
            colNames = [colNames, {'RADAR_YAW', 'CMD_YAWDPS', 'TARGET_YAW'}];
        end
    end
end

function idx = findColumnsByPrefix(colNames, varargin)
    % 按列名前缀（"U1:", "U2:" 等）找列索引
    idx = zeros(1, nargin-1);
    for k = 2:nargin
        prefix = varargin{k-1};
        for i = 1:length(colNames)
            if startsWith(colNames{i}, [prefix, ':'])
                idx(k-1) = i;
                break;
            end
        end
    end
end

function idx = findYawColumns(colNames)
    % 找 RADAR_YAW, CMD_YAWDPS, TARGET_YAW 的列索引
    idx = zeros(1, 3);
    for i = 1:length(colNames)
        name = colNames{i};
        if contains(name, 'RADAR_YAW', 'IgnoreCase', true)
            idx(1) = i;
        elseif contains(name, 'CMD_YAWDPS', 'IgnoreCase', true)
            idx(2) = i;
        elseif contains(name, 'TARGET_YAW', 'IgnoreCase', true)
            idx(3) = i;
        end
    end
    if all(idx == 0)
        idx = [];
    end
end


%% =========================================================================
%  时间范围过滤
% =========================================================================
function [filtered, keepMask] = filterByTimeRange(mat, timeRange)
    % mat 第 1 列是 T_REL
    t = mat(:, 1);
    keepMask = (t >= timeRange(1)) & (t <= timeRange(2));
    filtered = mat(keepMask, :);
end


%% =========================================================================
%  交互式时间选择
% =========================================================================
function timeRange = interactiveTimeSelect(t_rel, radar_pos_xyz, colNames)
% INTERACTIVETIMESELECT  弹出图让用户框选时间范围
    fprintf('   正在打开交互图，请用鼠标框选感兴趣的时间段...\n');
    fprintf('   (关闭窗口或按 Enter 使用全段)\n');

    fig = figure('Name', '选择分析时间范围', 'Color', 'w', 'Position', [100,100,1000,400]);
    hold on;
    colors = lines(3);
    axis_names = {'X', 'Y', 'Z'};
    for ax = 1:3
        plot(t_rel, radar_pos_xyz(:, ax), '-', 'Color', colors(ax,:), 'LineWidth', 1);
    end
    xlabel('相对时间 (s)');
    ylabel('位置');
    title('框选时间范围 (关闭此图使用全段)');
    legend('X', 'Y', 'Z', 'Location', 'best');
    grid on;

    % 等待用户框选
    try
        raw_timeRange = getrect;
        if ~isempty(raw_timeRange) && raw_timeRange(3) > 0
            timeRange = [raw_timeRange(1), raw_timeRange(1) + raw_timeRange(3)];
            fprintf('   选择的时间范围: [%.2f, %.2f] s\n', timeRange(1), timeRange(2));
        else
            timeRange = [];
            fprintf('   未选择，使用全段\n');
        end
    catch
        timeRange = [];
        fprintf('   未选择，使用全段\n');
    end

    % 安全关闭
    if isvalid(fig)
        close(fig);
    end
end


%% =========================================================================
%  检测雷达刷新行
% =========================================================================
function mask = detectRadarRefresh(mat, colNames)
% DETECTRADARREFRESH  检测 radar_pos 发生变化的行
%   思路：找到 RADAR_POS_X/Y/Z 列，比较相邻行差异
    % 找列索引
    radar_x_idx = find(contains(colNames, 'RADAR_POS_X'));
    radar_y_idx = find(contains(colNames, 'RADAR_POS_Y'));
    radar_z_idx = find(contains(colNames, 'RADAR_POS_Z'));

    if isempty(radar_x_idx) || isempty(radar_y_idx) || isempty(radar_z_idx)
        % fallback: 假设 3:5 列是 radar_pos
        radar_x_idx = 3; radar_y_idx = 4; radar_z_idx = 5;
    end

    radar_pos = mat(:, [radar_x_idx, radar_y_idx, radar_z_idx]);

    N = size(radar_pos, 1);
    mask = false(N, 1);
    mask(1) = true;  % 第一行总是视为刷新

    for k = 2:N
        prev = radar_pos(k-1, :);
        curr = radar_pos(k, :);

        % 情况 1: NaN 状态变化（从 NaN ↔ 非 NaN）
        prev_nan = isnan(prev);
        curr_nan = isnan(curr);
        if any(prev_nan ~= curr_nan)
            mask(k) = true;
            continue;
        end

        % 情况 2: 数值变化（在两个都不是 NaN 的位置上）
        valid = ~curr_nan & ~prev_nan;
        if any(valid & abs(curr - prev) > 0.01)
            mask(k) = true;
        end
    end
end


%% =========================================================================
%  补齐到均匀时间网格（去重 + 前值填充缺失点）
% =========================================================================
function [matOut, stats] = fillToUniformGrid(mat, dt_target)
% FILLTOUNIFORMGRID  把数据补齐到均匀 dt 网格
%   第 1 列必须是 T_REL（相对时间）
%   步骤:
%     1. 构建目标均匀时间向量
%     2. 对每个目标时间点，用"最近的之前数据"填充（前值填充 / zero-order hold）
%   注意：不去重！即使相邻行数据完全相同也保留，保证 0.01s 采样率不丢失时间点
    N = size(mat, 1);
    if N == 0
        matOut = mat;
        stats.filled = 0;
        stats.dedup_removed = 0;
        stats.final_rows = 0;
        return;
    end

    t_orig = mat(:, 1);

    %% Step 1: 构建目标均匀时间向量
    t_start = t_orig(1);
    t_end = t_orig(end);
    % 用 round 避免浮点误差导致的多余/缺失点
    nSteps = round((t_end - t_start) / dt_target);
    t_uniform = t_start + (0:nSteps)' * dt_target;

    %% Step 2: 前值填充
    % 对每个 t_uniform(k)，找最大的 j 使 t_orig(j) <= t_uniform(k)
    N_uniform = length(t_uniform);
    idx = zeros(N_uniform, 1);
    j = 1;
    for k = 1:N_uniform
        while j < N && t_orig(j+1) <= t_uniform(k) + 1e-9
            j = j + 1;
        end
        idx(k) = j;
    end

    matOut = mat(idx, :);
    matOut(:, 1) = t_uniform;  % 时间列用均匀值

    % 统计：被填充的点数 = 均匀点数 - 原始点数
    stats.filled = N_uniform - N;
    stats.dedup_removed = 0;
    stats.final_rows = N_uniform;
end


%% =========================================================================
%  保存输出
% =========================================================================
function [outClean, outRadar, outMeta, metadata] = saveCleanData(cleanMat, radarSyncMat, ...
    colNames, rawCsvPath, opts, tickStart)

    % 确保 rawCsvPath 为 char（防御性）
    rawCsvPath = char(rawCsvPath);

    if isempty(opts.output_dir)
        outDir = fileparts(rawCsvPath);
    else
        outDir = char(opts.output_dir);
    end
    if isempty(outDir)
        outDir = '.';
    end
    [~, baseName, ~] = fileparts(rawCsvPath);
    baseName = char(baseName);

    % Clean CSV (全采样率)
    outClean = char(fullfile(outDir, [baseName, '_clean.csv']));
    fprintf('   写入: %s\n', outClean);
    writeCleanCsv(outClean, cleanMat, colNames);

    % Radar-sync CSV
    outRadar = char(fullfile(outDir, [baseName, '_radar_sync.csv']));
    fprintf('   写入: %s\n', outRadar);
    writeCleanCsv(outRadar, radarSyncMat, colNames);

    % 元数据
    metadata.time_range_s = [cleanMat(1,1), cleanMat(end,1)];
    metadata.tick_start_s = tickStart;
    metadata.n_samples_full = size(cleanMat, 1);
    metadata.n_samples_radar_sync = size(radarSyncMat, 1);
    metadata.n_columns = size(cleanMat, 2);
    metadata.column_names = colNames;
    metadata.source_file = rawCsvPath;
    metadata.invalid_values = opts.invalid_values;
    metadata.dt_nominal_s = 0.01;  % 100Hz 标称
    if size(cleanMat, 1) > 1
        metadata.dt_actual_mean_s = mean(diff(cleanMat(:, 1)));
        metadata.dt_actual_max_s = max(diff(cleanMat(:, 1)));
    else
        metadata.dt_actual_mean_s = NaN;
        metadata.dt_actual_max_s = NaN;
    end

    outMeta = char(fullfile(outDir, [baseName, '_metadata.mat']));
    fprintf('   写入: %s\n', outMeta);
    save(outMeta, 'metadata');
end

function writeCleanCsv(path, mat, colNames)
    % 强制转为 char（防止 string 对象传入导致 fopen 失败）
    path = char(path);
    if isempty(path)
        error('writeCleanCsv: 文件路径为空');
    end

    % 构造 table（保留所有原始数值，包括 -32768、-1 等哨兵值）
    varData = cell(1, size(mat, 2));
    for c = 1:size(mat, 2)
        varData{c} = mat(:, c);
    end
    T = table(varData{:}, 'VariableNames', colNames);

    % writetable 会保留所有数值（不会把任何值变成空）
    writetable(T, path);
end
