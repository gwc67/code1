function result = my_pid_m(csvPath)

    if nargin < 1 || isempty(csvPath)
        error('csvPath 不能为空');
    end

    % X 轴（与 Y 相同）
    xy.Kp_base = 0.18;
    xy.Ki_base = 0.2;
    xy.Kd_base = 0.5;
    xy.output_max = 23;
    xy.output_min = -23;
    xy.integral_max = 5;
    xy.I_Band = 25;
    xy.d_filter_alpha = 0.8;
    xy.error_threshold_high = 25;
    xy.error_threshold_low = 25;
    xy.Kp_high_ratio = 2.0;
    xy.Ki_high_ratio = 0.7;
    xy.Kd_high_ratio = 0.58;
    xy.kv = 1.0;
    xy.ka = 2.27;

    % Z 轴
    z = xy;
    z.Kp_base = 0.23;
    z.Ki_base = 0.03;
    z.Kd_base = 0.08;
    z.output_max = 20;
    z.output_min = -20;
    z.I_Band = 20;
    z.error_threshold_high = 10;
    z.error_threshold_low = 2;
    z.Kd_high_ratio = 0.7;
    z.ka = 2.22;

end


function data = loadCsvData(csvPath)
% LOADCSVDATA  读取 CSV 并提取雷达位置/目标位置/速度指令
%   支持两种格式：
%     A. 原始格式（20 列）：第 1 行数字 header，第 2 行列名，第 3 行起数据
%        U1-U3: radar_pos, U4-U6: target_pos, U7-U9: cmd_vel
%     B. 预处理格式（14 列）：第 1 行列名 header，第 2 行起数据
%        T_REL, T_ABS, RADAR_POS_X/Y/Z, TARGET_POS_X/Y/Z, CMD_SPEED_X/Y/Z, ERROR_X/Y/Z

    csvPath = char(csvPath);  % 防御性：确保 char

    % 读第一行来判断格式
    fid = fopen(csvPath, 'r');
    if fid == -1
        error('无法打开文件: %s', csvPath);
    end
    firstLine = fgetl(fid);
    fclose(fid);
    firstLine = strtrim(firstLine);

    % 通过第一行内容判断格式
    isCleanFormat = contains(firstLine, 'T_REL') || contains(firstLine, 'RADAR_POS_X');
    % 如果第一行是纯数字逗号分隔 → raw 格式
    isRawNumericHeader = ~isempty(regexp(firstLine, '^\s*1\s*,\s*2\s*,', 'once'));

    if isCleanFormat || (~isRawNumericHeader && ~isempty(regexp(firstLine, '^[A-Z_]', 'once')))
        % 预处理格式：单行 header，列名即数据描述
        data = loadCleanFormat(csvPath);
    else
        % 原始格式：两行 header（数字 + 列名）
        data = loadRawFormat(csvPath);
    end
end

function data = loadCleanFormat(csvPath)
% 预处理后的 clean CSV 格式
    opts = detectImportOptions(csvPath);
    opts.DataLines = [2, Inf];  % 跳过第 1 行列名
    rawData = readmatrix(csvPath, opts);
    rawData(all(isnan(rawData), 2), :) = [];

    % 通过列名找列索引
    colNames = opts.VariableNames;
    radarIdx = find(contains(colNames, 'RADAR_POS'));
    targetIdx = find(contains(colNames, 'TARGET_POS'));
    cmdIdx = find(contains(colNames, 'CMD_SPEED'));
    trelIdx = find(contains(colNames, 'T_REL'));

    if length(radarIdx) < 3 || length(targetIdx) < 3 || length(cmdIdx) < 3
        error('clean CSV 缺少必要列 (RADAR_POS/TARGET_POS/CMD_SPEED)');
    end

    data.radar_pos = rawData(:, radarIdx(1:3));
    data.target_pos = rawData(:, targetIdx(1:3));
    data.cmd_vel = rawData(:, cmdIdx(1:3));

    % 从 T_REL 计算实际 dt
    if ~isempty(trelIdx)
        t = rawData(:, trelIdx(1));
        validDt = diff(t);
        validDt = validDt(validDt > 0.001);  % 过滤 0 或负值
        if ~isempty(validDt)
            data.dt = median(validDt);
        else
            data.dt = 0.01;  % fallback 100Hz
        end
    else
        data.dt = 0.01;
    end
    fprintf('   (clean 格式, %d 列, dt=%.4f s)\n', size(rawData, 2), data.dt);
end
