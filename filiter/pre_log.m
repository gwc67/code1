function result = pre_log(csvPath, options)
% PRE_LOG  将飞控原始日志 CSV 按 Tick 去重并重采样到 0.1s
%
% 用法:
%   result = pre_log(csvPath)
%   result = pre_log(csvPath, options)
%
% 输入:
%   csvPath  - 原始 CSV 文件路径（默认: excel_csv/still_circle.csv）
%   options  - 可选 struct:
%       options.output_dir   - 输出目录，默认与输入文件同目录
%       options.output_name  - 输出文件名（不含扩展名），默认 'still_circle_filtered'
%       options.dt           - 目标时间间隔 (s)，默认 0.1
%
% 输出:
%   result - struct:
%       result.output_path   - 输出 CSV 路径
%       result.data          - 过滤后数据矩阵 (Mx14)
%       result.column_names  - 列名 cell array
%       result.n_raw         - 原始行数
%       result.n_dedup       - 去重后行数
%       result.n_output      - 最终输出行数
%
% 原始 CSV 格式:
%   第 1 行: 数字 header (1,2,3,...,20,)
%   第 2 行: 列名 header (U1:RADAR_POS_X, ...)
%   第 3 行起: 数据
%   U20 列: Tick (秒, MCU 启动后绝对时间)

    %% 0. 输入校验与默认选项
    if nargin < 1 || isempty(csvPath)
        csvPath = fullfile(fileparts(mfilename('fullpath')), '..', 'excel_csv', 'still_circle.csv');
        csvPath = char(csvPath);
    end
    csvPath = char(csvPath);
    if nargin < 2 || isempty(options)
        options = struct();
    end

    opts.dt = 0.1;
    opts.output_dir = '';
    opts.output_name = '';

    flds = fieldnames(options);
    for i = 1:length(flds)
        opts.(flds{i}) = options.(flds{i});
    end

    %% 1. 加载原始 CSV
    fprintf('\n[1/5] 加载原始 CSV: %s\n', csvPath);
    [rawData, colNames] = loadRawCsv(csvPath);
    nRaw = size(rawData, 1);
    fprintf('   原始数据: %d 行 x %d 列\n', nRaw, size(rawData, 2));

    %% 2. 按 Tick 列去重（保留最后一个）
    fprintf('[2/5] 按 Tick 去重（保留最后一个）...\n');
    tickCol = rawData(:, 20);  % U20 = Tick
    [dedupData, nDedup] = deduplicateByTick(rawData, tickCol);
    fprintf('   去重后: %d 行 (删除 %d 行重复)\n', nDedup, nRaw - nDedup);

    %% 3. 计算相对时间 + 重采样到 0.1s
    fprintf('[3/5] 重采样到 dt = %.2f s（前值填充）...\n', opts.dt);
    tick_abs = dedupData(:, 20);
    t_start = tick_abs(1);
    t_rel = tick_abs - t_start;

    fprintf('   Tick 范围: %.3f - %.3f s (相对: %.3f - %.3f s)\n', ...
        tick_abs(1), tick_abs(end), t_rel(1), t_rel(end));

    [resampledData, nOutput] = resampleToUniform(t_rel, dedupData, opts.dt);
    fprintf('   重采样后: %d 行\n', nOutput);

    %% 4. 构造输出矩阵（14 列）
    fprintf('[4/5] 构造输出数据...\n');
    radar_pos = resampledData(:, 1:3);    % U1-U3
    target_pos = resampledData(:, 4:6);   % U4-U6
    cmd_vel = resampledData(:, 7:9);      % U7-U9
    error = target_pos - radar_pos;

    t_rel_out = resampledData(:, 21);     % 均匀时间列
    t_abs_out = resampledData(:, 22);     % 绝对时间列

    outMat = [t_rel_out, t_abs_out, radar_pos, target_pos, cmd_vel, error];
    outColNames = {'T_REL', 'T_ABS', ...
                   'RADAR_POS_X', 'RADAR_POS_Y', 'RADAR_POS_Z', ...
                   'TARGET_POS_X', 'TARGET_POS_Y', 'TARGET_POS_Z', ...
                   'CMD_SPEED_X', 'CMD_SPEED_Y', 'CMD_SPEED_Z', ...
                   'ERROR_X', 'ERROR_Y', 'ERROR_Z'};

    fprintf('   输出: %d 行 x %d 列\n', size(outMat, 1), size(outMat, 2));
    fprintf('   列名: %s\n', strjoin(outColNames, ', '));

    %% 5. 保存输出
    fprintf('[5/5] 保存输出...\n');
    outPath = saveOutput(outMat, outColNames, csvPath, opts);
    fprintf('   写入: %s\n', outPath);

    %% 6. 构造返回值
    result.output_path = outPath;
    result.data = outMat;
    result.column_names = outColNames;
    result.n_raw = nRaw;
    result.n_dedup = nDedup;
    result.n_output = nOutput;

    fprintf('\n✅ 预处理完成\n');
    fprintf('   原始: %d 行 → 去重: %d 行 → 输出: %d 行\n', nRaw, nDedup, nOutput);
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
    % 注意：CSV 每行末尾有尾随逗号，导致实际列数比 colNames 多 1
    % 使用 NumVariables+1 确保正确读取所有数据列
    nCols = length(colNames);
    opts = detectImportOptions(csvPath, 'NumVariables', nCols + 1);
    opts.DataLines = [3, Inf];
    opts = setvartype(opts, 1:nCols+1, 'double');
    rawData = readmatrix(csvPath, opts);

    % 去掉最后一列（尾随逗号产生的空列）
    data = rawData(:, 1:nCols);

    % 去全 NaN 行
    data(all(isnan(data), 2), :) = [];
end


%% =========================================================================
%  按 Tick 去重（优先保留 CMD_SPEED 非零的行）
% =========================================================================
function [dedupData, nDedup] = deduplicateByTick(data, tickCol)
    N = size(data, 1);
    if N == 0
        dedupData = data;
        nDedup = 0;
        return;
    end

    % 对每个唯一 Tick 值，找最佳行的索引
    [uniqueTicks, ~, groupIdx] = unique(tickCol, 'stable');
    nUnique = length(uniqueTicks);

    keepIdx = zeros(nUnique, 1);
    for k = 1:nUnique
        groupRows = find(groupIdx == k);

        % 策略：优先保留 CMD_SPEED (列 7-9) 非零的行
        % 原因：飞控在同一 Tick 内可能先输出带速度的行，再输出速度=0 的行
        cmd_speed = data(groupRows, 7:9);
        has_speed = any(abs(cmd_speed) > 0.5, 2);  % 哪些行有非零速度

        if any(has_speed)
            % 有速度的行中，取最后一个（最新的状态）
            speedRows = groupRows(has_speed);
            keepIdx(k) = speedRows(end);
        else
            % 所有行速度都为零，取最后一行
            keepIdx(k) = groupRows(end);
        end
    end

    % 按原始顺序排序
    keepIdx = sort(keepIdx);
    dedupData = data(keepIdx, :);
    nDedup = length(keepIdx);
end


%% =========================================================================
%  重采样到均匀时间网格（前值填充）
% =========================================================================
function [outData, nOut] = resampleToUniform(t_rel, data, dt_target)
% RESAMPLETOUNIFORM  把数据重采样到均匀 dt 网格
%   第 1 列必须是 T_REL（相对时间）
%   步骤:
%     1. 构建目标均匀时间向量
%     2. 对每个目标时间点，用"最近的之前数据"填充（前值填充 / zero-order hold）
    N = size(data, 1);
    if N == 0
        outData = data;
        nOut = 0;
        return;
    end

    %% Step 1: 构建目标均匀时间向量
    t_start = t_rel(1);
    t_end = t_rel(end);
    nSteps = round((t_end - t_start) / dt_target);
    t_uniform = t_start + (0:nSteps)' * dt_target;

    %% Step 2: 前值填充
    % 对每个 t_uniform(k)，找最大的 j 使 t_rel(j) <= t_uniform(k)
    N_uniform = length(t_uniform);
    idx = zeros(N_uniform, 1);
    j = 1;
    for k = 1:N_uniform
        while j < N && t_rel(j+1) <= t_uniform(k) + 1e-9
            j = j + 1;
        end
        idx(k) = j;
    end

    outData = data(idx, :);
    % 添加均匀时间列和对应的绝对时间列
    tick_abs_orig = data(:, 20);  % 原始 Tick
    t_abs_start = tick_abs_orig(1);
    t_abs_uniform = t_abs_start + t_uniform;  % 绝对时间 = 起点 + 相对时间

    outData = [outData, t_uniform, t_abs_uniform];  % 追加两列
    nOut = N_uniform;
end


%% =========================================================================
%  保存输出
% =========================================================================
function outPath = saveOutput(mat, colNames, rawCsvPath, opts)
    if isempty(opts.output_dir)
        outDir = fileparts(rawCsvPath);
    else
        outDir = char(opts.output_dir);
    end
    if isempty(outDir)
        outDir = '.';
    end

    if isempty(opts.output_name)
        [~, baseName, ~] = fileparts(rawCsvPath);
        outName = [char(baseName), '_filtered'];
    else
        outName = char(opts.output_name);
    end

    outPath = char(fullfile(outDir, [outName, '.csv']));

    % 构造 table 并写入
    varData = cell(1, size(mat, 2));
    for c = 1:size(mat, 2)
        varData{c} = mat(:, c);
    end
    T = table(varData{:}, 'VariableNames', colNames);
    writetable(T, outPath);
end
