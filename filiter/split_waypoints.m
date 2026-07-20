function result = split_waypoints(filteredCsvPath, options)
% SPLIT_WAYPOINTS  将 pre_log 滤波后的数据按航点分离
%
% 用法:
%   result = split_waypoints()
%   result = split_waypoints(filteredCsvPath)
%   result = split_waypoints(filteredCsvPath, options)
%
% 输入:
%   filteredCsvPath - pre_log 输出的 filtered CSV 路径
%                     默认: excel_csv/still_circle_filtered.csv
%   options         - 可选 struct:
%       options.threshold  - TARGET_POS 变化阈值，默认 0.5
%       options.output_dir - 输出目录，默认与 filtered CSV 同级
%       options.skip_zero  - 是否跳过初始零值段，默认 true
%
% 输出:
%   result - struct:
%       result.output_dir      - 输出文件夹路径
%       result.segment_files   - 各航点 CSV 文件路径 cell array
%       result.segment_data    - 各航点数据 cell array
%       result.target_coords   - 各航点目标坐标矩阵 (Nx3)
%       result.n_segments      - 航点数量
%       result.n_total_rows    - 总数据行数

    %% 0. 输入校验与默认选项
    if nargin < 1 || isempty(filteredCsvPath)
        filteredCsvPath = fullfile(fileparts(mfilename('fullpath')), '..', 'excel_csv', 'still_circle_filtered.csv');
        filteredCsvPath = char(filteredCsvPath);
    end
    filteredCsvPath = char(filteredCsvPath);
    if nargin < 2 || isempty(options)
        options = struct();
    end

    opts.threshold = 0.5;
    opts.output_dir = '';
    opts.skip_zero = true;

    flds = fieldnames(options);
    for i = 1:length(flds)
        opts.(flds{i}) = options.(flds{i});
    end

    %% 1. 加载 filtered 数据
    fprintf('\n[1/5] 加载 filtered 数据: %s\n', filteredCsvPath);
    [data, colNames] = loadFilteredCsv(filteredCsvPath);
    nTotal = size(data, 1);
    fprintf('   数据: %d 行 x %d 列\n', nTotal, size(data, 2));
    fprintf('   列名: %s\n', strjoin(colNames, ', '));

    %% 2. 检测航点切换
    fprintf('[2/5] 检测航点切换（阈值: %.2f）...\n', opts.threshold);
    target_pos = data(:, 6:8);  % TARGET_POS_X/Y/Z (列 6-8)

    % 计算相邻行差异
    diff_target = diff(target_pos);
    change_mask = any(abs(diff_target) > opts.threshold, 2);

    % 找到切换点索引（+1 因为 diff 少一行）
    switch_indices = find(change_mask) + 1;
    nSwitches = length(switch_indices);
    fprintf('   检测到 %d 次航点切换\n', nSwitches);

    %% 3. 跳过初始零值段
    if opts.skip_zero
        fprintf('[3/5] 跳过初始零值段...\n');
        % 找到第一个非零 TARGET_POS 的行
        first_nonzero = find(any(abs(target_pos) > opts.threshold, 2), 1);
        if ~isempty(first_nonzero) && first_nonzero > 1
            fprintf('   跳过前 %d 行（TARGET_POS 全零）\n', first_nonzero - 1);
            data = data(first_nonzero:end, :);
            % 重新计算 switch_indices
            target_pos = data(:, 6:8);
            diff_target = diff(target_pos);
            change_mask = any(abs(diff_target) > opts.threshold, 2);
            switch_indices = find(change_mask) + 1;
            nSwitches = length(switch_indices);
        else
            first_nonzero = 1;
            fprintf('   无初始零值段\n');
        end
    else
        first_nonzero = 1;
        fprintf('[3/5] 保留初始零值段\n');
    end

    %% 4. 分割数据段
    fprintf('[4/5] 分割航点数据...\n');
    nData = size(data, 1);
    segments = cell(nSwitches + 1, 1);

    if nSwitches == 0
        % 无切换，整个数据为一个航点
        segments{1} = data;
    else
        % 第一段：从开始到第一个切换点
        segments{1} = data(1:switch_indices(1)-1, :);

        % 中间段
        for k = 2:nSwitches
            segments{k} = data(switch_indices(k-1):switch_indices(k)-1, :);
        end

        % 最后一段：从最后一个切换点到结束
        segments{nSwitches+1} = data(switch_indices(end):end, :);
    end

    % 移除空段
    empty_mask = cellfun(@(x) isempty(x), segments);
    segments(empty_mask) = [];
    nSegments = length(segments);
    fprintf('   分割为 %d 个航点段\n', nSegments);

    %% 5. 保存输出
    fprintf('[5/5] 保存航点数据...\n');
    if isempty(opts.output_dir)
        outDir = fullfile(fileparts(filteredCsvPath), 'still_circle_waypoints');
    else
        outDir = char(opts.output_dir);
    end
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    fprintf('   输出目录: %s\n', outDir);

    segment_files = cell(nSegments, 1);
    segment_data = cell(nSegments, 1);
    target_coords = zeros(nSegments, 3);

    for k = 1:nSegments
        seg = segments{k};

        % 提取 TARGET_POS 稳态值（中位数，避免过渡段噪声）
        target_steady = median(seg(:, 6:8), 1);
        target_coords(k, :) = target_steady;

        % 生成文件名：wp1_target_0_75_140.csv
        fname = sprintf('wp%d_target_%d_%d_%d.csv', k, ...
            round(target_steady(1)), round(target_steady(2)), round(target_steady(3)));
        outPath = fullfile(outDir, fname);

        % 保存 CSV
        varData = cell(1, size(seg, 2));
        for c = 1:size(seg, 2)
            varData{c} = seg(:, c);
        end
        T = table(varData{:}, 'VariableNames', colNames);
        writetable(T, outPath);

        segment_files{k} = outPath;
        segment_data{k} = seg;

        fprintf('   [%d/%d] %s (%d 行)\n', k, nSegments, fname, size(seg, 1));
    end

    %% 6. 构造返回值
    result.output_dir = outDir;
    result.segment_files = segment_files;
    result.segment_data = segment_data;
    result.target_coords = target_coords;
    result.n_segments = nSegments;
    result.n_total_rows = nTotal;

    fprintf('\n✅ 航点分离完成\n');
    fprintf('   总数据: %d 行 → %d 个航点\n', nTotal, nSegments);
    fprintf('   输出目录: %s\n', outDir);
end


%% =========================================================================
%  加载 filtered CSV
% =========================================================================
function [data, colNames] = loadFilteredCsv(csvPath)
    % 读列名 (第 1 行)
    fid = fopen(csvPath, 'r');
    if fid == -1
        error('无法打开文件: %s', csvPath);
    end
    headerLine = fgetl(fid);
    fclose(fid);

    % 解析列名
    colNames = strsplit(headerLine, ',');
    colNames = strtrim(colNames);
    colNames(cellfun(@isempty, colNames)) = [];

    % 读数据（第 2 行起）
    nCols = length(colNames);
    opts = detectImportOptions(csvPath, 'NumVariables', nCols);
    opts.DataLines = [2, Inf];
    opts = setvartype(opts, 1:nCols, 'double');
    data = readmatrix(csvPath, opts);

    % 去全 NaN 行
    data(all(isnan(data), 2), :) = [];
end
