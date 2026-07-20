function result = analyze_pid_performance(csvPath, pidParams, tuningMode)
% ANALYZE_PID_PERFORMANCE  从飞行日志辨识被控对象并推荐 PID 参数
%
% 用法:
%   result = analyze_pid_performance(csvPath)
%   result = analyze_pid_performance(csvPath, pidParams)
%   result = analyze_pid_performance(csvPath, pidParams, tuningMode)
%
% 输入:
%   csvPath     - CSV 文件路径（必须）
%   pidParams   - 可选 struct，部分覆盖 PID 参数（未指定字段用 C 代码默认值）
%                 例如：pidParams.X.Kp_base = 0.25;
%                       pidParams.X.plant_init = [K0, T0, tau0];
%   tuningMode  - 调参档位：'balanced' (默认) / 'aggressive' / 'conservative'
%
% 输出:
%   result - struct，包含：
%       result.recommended.X/Y/Z.Kp_base, Ki_base, Kd_base  - 推荐参数
%       result.identified_plant.X/Y/Z                        - 被控对象 tf 对象
%       result.identified_plant.X/Y/Z_params                 - struct('K','T','tau')
%       result.metrics.X/Y/Z                                 - 性能指标 struct
%       result.model_fit.NRMSE                               - struct('X','Y','Z')
%
% 依赖: Optimization Toolbox, Control System Toolbox

    %% 0. 输入校验与默认值
    if nargin < 1 || isempty(csvPath)
        error('csvPath 不能为空');
    end
    if nargin < 2 || isempty(pidParams)
        pidParams = struct();
    end
    if nargin < 3 || isempty(tuningMode)
        tuningMode = 'balanced';
    end
    validateattributes(tuningMode, 'char', {'nonempty'});
    if ~ismember(tuningMode, {'balanced', 'aggressive', 'conservative'})
        error('tuningMode 必须是 balanced/aggressive/conservative');
    end

    %% S2: 合并用户参数与 C 代码默认值
    pid = mergePidParams(pidParams);

    %% S3: 加载 CSV 数据
    fprintf('\n[1/6] 读取数据: %s\n', csvPath);
    rawData = loadCsvData(csvPath);
    radar_pos = rawData.radar_pos;      % Nx3
    target_pos = rawData.target_pos;    % Nx3
    cmd_vel = rawData.cmd_vel;          % Nx3
    N = size(radar_pos, 1);
    dt = rawData.dt;
    t = (0:N-1)' * dt;
    fprintf('   采样点数: %d, dt = %.3f s, 总时长 = %.1f s\n', N, dt, (N-1)*dt);

    %% S4: 数据质量门控
    fprintf('\n[2/6] 数据质量检查...\n');
    quality = checkDataQuality(radar_pos, target_pos, cmd_vel, dt);

    %% 准备结果结构
    result = struct();
    result.recommended = struct();
    result.identified_plant = struct();
    result.metrics = struct();
    result.model_fit.NRMSE = struct();
    result.pid_validation = struct();   % Part A: PID 仿真对比
    result.steady_state = struct();      % 稳态分析
    axis_names = {'X', 'Y', 'Z'};

    %% ========== Part A: PID 仿真对比（所有数据都能做）==========
    fprintf('\n[3/6] PID 仿真对比 (MATLAB 模拟 vs 实际 cmd_vel)...\n');
    for ax = 1:3
        ax_name = axis_names{ax};
        pid_ax = pid.(ax_name);

        % 用 logged radar_pos 作为测量值，target_pos 作为设定值
        % 跑 MATLAB 版 PID，得到 cmd_vel_sim
        state = [];
        cmd_vel_sim = zeros(N, 1);
        for k = 1:N
            setpoint = target_pos(k, ax);
            measurement = radar_pos(k, ax);
            if isnan(setpoint) || isnan(measurement)
                if k > 1
                    cmd_vel_sim(k) = cmd_vel_sim(k-1);  % 保持上一拍
                end
                continue;
            end
            [cmd_vel_sim(k), state] = pidSimulate(pid_ax, setpoint, measurement, state, dt);
        end

        % 对比：仿真 vs 实际
        valid = ~isnan(cmd_vel(:,ax)) & ~isnan(cmd_vel_sim);
        if sum(valid) > 0
            err_sim = cmd_vel_sim(valid) - cmd_vel(valid);
            nrmse_sim = sqrt(mean(err_sim.^2)) / (max(cmd_vel(valid)) - min(cmd_vel(valid)) + 1e-6);
            corr_sim = corr(cmd_vel_sim(valid), cmd_vel(valid));
        else
            nrmse_sim = NaN;
            corr_sim = NaN;
        end

        result.pid_validation.(ax_name).cmd_vel_sim = cmd_vel_sim;
        result.pid_validation.(ax_name).nrmse = nrmse_sim;
        result.pid_validation.(ax_name).correlation = corr_sim;

        fprintf('   %s 轴: NRMSE = %.1f%%, 相关系数 = %.3f\n', ax_name, nrmse_sim*100, corr_sim);
        if nrmse_sim < 0.15
            fprintf('      ✅ MATLAB PID 仿真与 C 代码输出高度吻合\n');
        elseif nrmse_sim < 0.40
            fprintf('      ⚠️ 有一定差异（可能是 C 代码自适应/前馈的细微差别）\n');
        else
            fprintf('      ❌ 差异较大 —— 检查 C 代码是否有未复刻的逻辑\n');
        end
    end

    %% ========== 稳态分析（所有数据都能做）==========
    fprintf('\n[4/6] 稳态分析...\n');
    for ax = 1:3
        ax_name = axis_names{ax};
        pid_ax = pid.(ax_name);

        cmd_vel_sim_ax = result.pid_validation.(ax_name).cmd_vel_sim;
        ss = analyzeSteadyState(radar_pos(:,ax), target_pos(:,ax), ...
                                cmd_vel(:,ax), cmd_vel_sim_ax, pid_ax, dt);
        result.steady_state.(ax_name) = ss;

        fprintf('   %s 轴:\n', ax_name);
        fprintf('      平均误差: %.3f | 最大误差: %.3f\n', ss.mean_error, ss.max_error);
        fprintf('      有效 Kp (从数据估): %.3f (设定: %.3f)\n', ss.Kp_eff, pid_ax.Kp_base);
        fprintf('      控制量 std: %.3f | 控制量范围: [%.2f, %.2f]\n', ...
            ss.cmd_std, ss.cmd_min, ss.cmd_max);
        if ~isnan(ss.small_overshoot)
            fprintf('      小扰动超调: %.1f%%\n', ss.small_overshoot*100);
        end
    end

    %% ========== Part B: 被控对象辨识 + 调参（仅当激励足够时）==========
    if quality.sufficient
        fprintf('\n[5/6] 被控对象辨识与调参（数据激励充足）...\n');
        for ax = 1:3
            ax_name = axis_names{ax};
            pid_ax = pid.(ax_name);

            try
                [plant_tf, plant_params, nrmse] = identifyPlant(...
                    cmd_vel(:, ax), radar_pos(:, ax), dt, pid_ax);
                fprintf('   %s 轴 G(s) = %.3f * e^(-%.3fs) / (s * (%.3fs + 1)), NRMSE=%.1f%%\n', ...
                    ax_name, plant_params.K, plant_params.tau, plant_params.T, nrmse*100);

                [recommended, metrics] = tunePid(plant_params, pid_ax, tuningMode, dt);
                fprintf('      建议: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', ...
                    recommended.Kp, recommended.Ki, recommended.Kd);

                result.recommended.(ax_name) = recommended;
                result.identified_plant.(ax_name) = plant_tf;
                result.identified_plant.([ax_name '_params']) = plant_params;
                result.metrics.(ax_name) = metrics;
                result.model_fit.NRMSE.(ax_name) = nrmse;
            catch err
                fprintf('   ❌ %s 轴辨识失败: %s\n', ax_name, err.message);
            end
        end
    else
        fprintf('\n[5/6] 跳过被控对象辨识（数据激励不足，仅使用稳态分析）\n');
        for i = 1:length(quality.warnings)
            fprintf('   ⚠️ %s\n', quality.warnings{i});
        end
    end

    %% S1: 控制台报告 + 图表
    fprintf('\n[5/6] 生成分析报告...\n');
    printConsoleReport(result, pid, tuningMode);
    plotAnalysis(result, pid, radar_pos, target_pos, cmd_vel, t, dt);

    %% 保存 .mat
    fprintf('\n[6/6] 保存结果...\n');
    saveResult(result, csvPath);

    fprintf('\n✅ 分析完成\n');
end


%% =========================================================================
%  S2: 默认参数填充
%  从 PID_ctrl.c 的 PID_Init 硬编码值提取
% =========================================================================
function defaults = getDefaultPidParams()
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

    defaults.X = xy;
    defaults.Y = xy;
    defaults.Z = z;
end

function merged = mergePidParams(userParams)
% MERGEPIDPARAMS  把用户部分覆盖的 struct 与 C 代码默认值合并
    defaults = getDefaultPidParams();
    merged = defaults;
    if isempty(userParams)
        return;
    end
    axis_names = {'X', 'Y', 'Z'};
    for i = 1:3
        ax = axis_names{i};
        if isfield(userParams, ax)
            user_ax = userParams.(ax);
            flds = fieldnames(user_ax);
            for k = 1:length(flds)
                f = flds{k};
                % plant_init 不是 PID 参数，单独存储
                if strcmp(f, 'plant_init')
                    merged.(ax).plant_init = user_ax.(f);
                else
                    merged.(ax).(f) = user_ax.(f);
                end
            end
        end
    end
end


%% =========================================================================
%  S3: 数据加载（自动识别 raw 20 列或 clean 14 列格式）
% =========================================================================
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

function data = loadRawFormat(csvPath)
% 原始 20 列 CSV 格式
    % 先看有多少列
    opts0 = detectImportOptions(csvPath);
    nCols = length(opts0.VariableNames);
    opts = detectImportOptions(csvPath, 'NumVariables', nCols);
    opts.DataLines = [3, Inf];  % 跳过两行 header
    opts = setvartype(opts, 1:nCols, 'double');
    rawData = readmatrix(csvPath, opts);
    rawData(all(isnan(rawData), 2), :) = [];

    if nCols < 9
        error('原始 CSV 至少需要 9 列，实际只有 %d 列', nCols);
    end
    data.radar_pos = rawData(:, 1:3);
    data.target_pos = rawData(:, 4:6);
    data.cmd_vel = rawData(:, 7:9);

    % 尝试从 Tick 列（U20）获取真实 dt
    if nCols >= 20
        tickCol = find(contains(opts.VariableNames, 'Tick'));
        if ~isempty(tickCol)
            tick = rawData(:, tickCol(1));
            validTick = tick(tick > 0.1);
            if length(validTick) > 1
                dt_samples = diff(validTick);
                dt_samples = dt_samples(dt_samples > 0.001);
                if ~isempty(dt_samples)
                    data.dt = median(dt_samples);
                    fprintf('   (raw 格式, %d 列, dt=%.4f s 从 Tick 列)\n', nCols, data.dt);
                    return;
                end
            end
        end
    end
    data.dt = 0.01;  % fallback 100Hz
    fprintf('   (raw 格式, %d 列, dt=%.4f s 默认)\n', nCols, data.dt);
end


%% =========================================================================
%  S4: 数据质量门控
% =========================================================================
function quality = checkDataQuality(radar_pos, target_pos, cmd_vel, dt)
% CHECKDATAQUALITY  检查数据是否有足够的激励用于辨识
    quality = struct();
    quality.sufficient = true;
    quality.warnings = {};

    axis_names = {'X', 'Y', 'Z'};
    for ax = 1:3
        u = cmd_vel(:, ax);
        u_var = var(u);
        u_range = max(u) - min(u);
        % 阈值：cmd_vel 范围至少达到 output_max 的 10%
        if u_range < 2
            quality.sufficient = false;
            quality.warnings{end+1} = sprintf(...
                '%s 轴 cmd_vel 变化范围仅 %.2f (< 2)，激励不足', axis_names{ax}, u_range);
        end

        % 目标位置变化次数
        tgt = target_pos(:, ax);
        tgt_diff = abs(diff(tgt));
        n_changes = sum(tgt_diff > 0.5);  % 变化超过 0.5 单位视为一次
        if n_changes < 2
            quality.sufficient = false;
            quality.warnings{end+1} = sprintf(...
                '%s 轴目标位置仅变化 %d 次 (< 2)，难以辨识动态', axis_names{ax}, n_changes);
        end
    end

    for i = 1:length(quality.warnings)
        fprintf('   ⚠️ %s\n', quality.warnings{i});
    end
end


%% =========================================================================
%  S5: 被控对象正向仿真
% =========================================================================
function y = simulatePlant(u, K, T, tau, dt)
% SIMULATEPLANT  离散时间仿真 G(s) = K*e^(-tau*s) / (s*(T*s+1))
%   输入 u (cmd_vel) -> 输出 y (radar_pos)
%
%   结构：u -> [延迟 tau] -> [一阶惯性 K/(Ts+1)] -> [积分 1/s] -> y
%
%   离散化（ZOH, dt）：
%     一阶惯性: G1(z) = (1-a) / (z - a), a = exp(-dt/T)
%     积分:     G2(z) = dt / (z - 1)
%     延迟:     z^(-n), n = round(tau/dt)
    n = max(1, round(tau / dt));  % 至少 1 拍延迟
    a = exp(-dt / T);
    N = length(u);
    y = zeros(N, 1);
    % 状态：一阶惯性输出 w，积分输出 y
    w = 0;  % 一阶惯性状态
    yi = 0; % 积分状态
    % 延迟缓冲
    u_delayed = zeros(N + n, 1);
    u_delayed(n+1:end) = u;  % 前 n 拍为 0

    for k = 1:N
        % 延迟后的输入
        ud = u_delayed(k);
        % 一阶惯性: w[k+1] = a*w[k] + (1-a)*ud[k]
        w = a * w + (1 - a) * ud;
        % 增益 K
        w_scaled = K * w;
        % 积分: yi[k+1] = yi[k] + dt * w_scaled[k]
        yi = yi + dt * w_scaled;
        y(k) = yi;
    end
end


%% =========================================================================
%  S6: 被控对象辨识
% =========================================================================
function [plant_tf, plant_params, nrmse] = identifyPlant(u, y, dt, pid_ax)
% IDENTIFYPLANT  从 (u, y) 数据辨识 G(s) = K*e^(-tau*s) / (s*(T*s+1))
%   返回 tf 对象 + 参数 struct + NRMSE

    % 初始猜测
    if isfield(pid_ax, 'plant_init') && ~isempty(pid_ax.plant_init)
        p0 = pid_ax.plant_init;
        K0 = p0(1); T0 = p0(2); tau0 = p0(3);
    else
        % 自动估计
        [K0, T0, tau0] = estimateInitialGuess(u, y, dt);
    end

    % 归一化数据避免数值问题
    u_scale = max(abs(u));
    if u_scale < 1e-6
        u_scale = 1;
    end
    y_scale = max(abs(y));
    if y_scale < 1e-6
        y_scale = 1;
    end
    u_norm = u / u_scale;
    y_norm = y / y_scale;
    scale_ratio = y_scale / u_scale;

    % 优化
    opts = optimoptions('lsqnonlin', 'Display', 'off', 'MaxIterations', 500);
    lb = [1e-6, 1e-3, dt];              % K > 0, T > 0, tau >= dt
    ub = [1e3, 10, 1];                  % 合理上限

    params_opt = lsqnonlin(@(p) plantResidual(p, u_norm, y_norm, dt), ...
                           [K0*scale_ratio, T0, tau0], lb, ub, opts);

    K_opt = params_opt(1) * scale_ratio;  % 还原到原始尺度
    T_opt = params_opt(2);
    tau_opt = params_opt(3);

    % 构造传递函数（不含延迟，延迟单独记录）
    plant_tf = tf(K_opt, [T_opt, 1, 0], 'InputDelay', tau_opt);

    plant_params.K = K_opt;
    plant_params.T = T_opt;
    plant_params.tau = tau_opt;

    % 计算 NRMSE
    y_sim = simulatePlant(u, K_opt, T_opt, tau_opt, dt);
    nrmse = sqrt(mean((y - y_sim).^2)) / (max(y) - min(y) + 1e-6);
end

function [K0, T0, tau0] = estimateInitialGuess(u, y, dt)
% ESTIMATEINITIALGUESS  从阶跃响应粗略估计 (K, T, tau)
    N = length(u);
    % 找到第一个显著输入变化
    u_diff = abs(diff(u));
    [~, step_idx] = max(u_diff);
    if step_idx < 1 || step_idx >= N
        step_idx = 1;
    end
    u_step = u(step_idx);
    if abs(u_step) < 1e-6
        u_step = 1;
    end

    % 延迟估计：响应开始明显变化的时间
    y_baseline = y(step_idx);
    y_range = max(y) - min(y);
    if y_range < 1e-6
        K0 = 1; T0 = 0.2; tau0 = 0.1;
        return;
    end
    threshold = y_baseline + 0.1 * (max(y(step_idx:end)) - y_baseline);
    response_start = find(y(step_idx:end) > threshold, 1) + step_idx - 1;
    if isempty(response_start)
        response_start = step_idx;
    end
    tau0 = max(dt, (response_start - step_idx) * dt);

    % 稳态增益 K：从最终斜率估计（积分环节，速度 = K*u）
    last_quarter = round(N * 0.75):N;
    if length(last_quarter) > 2
        dy = y(last_quarter(end)) - y(last_quarter(1));
        dt_span = (last_quarter(end) - last_quarter(1)) * dt;
        if dt_span > 0
            vel_ss = dy / dt_span;
            K0 = vel_ss / u_step;
        else
            K0 = 1;
        end
    else
        K0 = 1;
    end
    if K0 < 1e-6
        K0 = 1;
    end

    % 时间常数 T：从响应 63% 时间估计
    y_target = y(step_idx) + 0.63 * (max(y(step_idx:end)) - y(step_idx));
    t63 = find(y(response_start:end) > y_target, 1);
    if ~isempty(t63)
        T0 = max(dt, t63 * dt);
    else
        T0 = 0.3;  % 默认 300ms
    end
end

function res = plantResidual(p, u, y, dt)
% PLANTRESIDUAL  仿真误差向量
    K = p(1); T = p(2); tau = p(3);
    y_sim = simulatePlant(u, K, T, tau, dt);
    res = y - y_sim;
end


%% =========================================================================
%  S7: IMC 解析公式
% =========================================================================
function [Kp, Ki, Kd] = imcPid(K, T, tau, lambda)
% IMCPID  内模控制公式（针对 G(s) = K*e^(-tau*s) / (s*(T*s+1))）
%   lambda 是期望闭环时间常数
    denom = K * (lambda + tau);
    if abs(denom) < 1e-10
        Kp = 0; Ki = 0; Kd = 0;
        return;
    end
    Kp = T / denom;
    Ki = 1 / denom;
    Kd = T * tau / denom;
end


%% =========================================================================
%  S8: ITAE 优化
% =========================================================================
function [recommended, metrics] = tunePid(plant_params, pid_ax, tuningMode, dt)
% TUNEPID  IMC + ITAE 组合调参
    % 选择 lambda
    switch tuningMode
        case 'conservative', lambda_mult = 3;
        case 'balanced',     lambda_mult = 2;
        case 'aggressive',   lambda_mult = 1;
    end
    lambda = lambda_mult * plant_params.tau;

    % IMC 初始猜测
    [Kp0, Ki0, Kd0] = imcPid(plant_params.K, plant_params.T, plant_params.tau, lambda);

    % 约束
    max_overshoot_frac = 0.05;  % 5%
    control_limit = 0.8 * pid_ax.output_max;

    % ITAE 优化
    opts = optimoptions('fmincon', 'Display', 'off', 'MaxIterations', 200);
    lb = [0, 0, 0];
    ub = [10, 10, 10];
    A = []; b = []; Aeq = []; beq = [];

    costFn = @(p) itaeCost(p, plant_params, dt, pid_ax.output_max, max_overshoot_frac);
    [p_opt, fval] = fmincon(costFn, [Kp0, Ki0, Kd0], A, b, Aeq, beq, lb, ub, ...
                            @(p) pidConstraints(p, plant_params, dt, control_limit), opts);

    recommended.Kp = p_opt(1);
    recommended.Ki = p_opt(2);
    recommended.Kd = p_opt(3);
    recommended.ITAE = fval;

    % 性能指标（推荐参数下）
    metrics = computeMetrics(p_opt, plant_params, pid_ax, dt);
end

function J = itaeCost(p, plant_params, dt, output_max, max_overshoot)
% ITAECOST  ITAE 代价 + 超调惩罚
    Kp = p(1); Ki = p(2); Kd = p(3);
    % 闭环仿真：单位阶跃响应
    t_sim = 0:dt:10;  % 10 秒仿真
    r = ones(size(t_sim));  % 单位阶跃
    % 被控对象 G(s) = K / (s(Ts+1)) with delay
    plant = tf(plant_params.K, [plant_params.T, 1, 0], 'InputDelay', plant_params.tau);
    % PID C(s) = Kd*s + Kp + Ki/s = (Kd*s^2 + Kp*s + Ki)/s
    C = tf([Kd, Kp, Ki], [1, 0]);
    % 闭环
    T_cl = feedback(C * plant, 1);
    y = step(T_cl, t_sim);
    e = r - y;
    % ITAE
    J = sum(t_sim' .* abs(e)) * dt;
    % 惩罚超调
    os = max(y) - 1;
    if os > max_overshoot
        J = J + 1000 * (os - max_overshoot)^2;
    end
end

function [c, ceq] = pidConstraints(p, plant_params, dt, control_limit)
% PIDCONSTRAINTS  不等式约束 c <= 0
    Kp = p(1); Ki = p(2); Kd = p(3);
    plant = tf(plant_params.K, [plant_params.T, 1, 0], 'InputDelay', plant_params.tau);
    C = tf([Kd, Kp, Ki], [1, 0]);
    L = C * plant;
    [Gm, Pm] = margin(L);
    % 相位裕度 >= 45°
    c = 45 - Pm;
    ceq = [];
end

function metrics = computeMetrics(p, plant_params, pid_ax, dt)
% COMPUTEMETRICS  计算给定 PID 参数在辨识模型上的性能指标
    Kp = p(1); Ki = p(2); Kd = p(3);
    plant = tf(plant_params.K, [plant_params.T, 1, 0], 'InputDelay', plant_params.tau);
    C = tf([Kd, Kp, Ki], [1, 0]);
    T_cl = feedback(C * plant, 1);
    t_sim = 0:dt:10;
    y = step(T_cl, t_sim);
    % 超调
    metrics.overshoot = max(0, max(y) - 1);
    % Settling time (2% 准则)
    idx = find(abs(y - 1) > 0.02, 1, 'last');
    if isempty(idx)
        metrics.settling_time = t_sim(1);
    else
        metrics.settling_time = t_sim(idx);
    end
    % 稳态误差
    metrics.steady_state_error = abs(y(end) - 1);
    % 控制量峰值（近似）
    u = step(C * T_cl / plant, t_sim);
    metrics.peak_control = max(abs(u));
    % 相位裕度
    L = C * plant;
    [~, Pm] = margin(L);
    metrics.phase_margin = Pm;
end


%% =========================================================================
%  S9: PID 模拟器（复刻 C 代码 PID_Update_l）
% =========================================================================
function [out, state] = pidSimulate(pid_ax, setpoint, measurement, state, dt)
% PIDSIMULATE  单步 PID 计算，完整复刻 C 代码 PID_Update_l 逻辑
%
% 输入:
%   pid_ax      - 该轴 PID 参数 struct
%   setpoint    - 设定值（标量）
%   measurement - 测量值（标量）
%   state       - PID 状态 struct（integral, prev_measurement, prev_d_filtered,
%                 pre_target_position, pre_target_vel）
%   dt          - 采样周期
%
% 输出:
%   out         - PID 输出
%   state       - 更新后的状态

    if isempty(state)
        state.integral = 0;
        state.prev_measurement = 0;
        state.prev_d_filtered = 0;
        state.pre_target_position = 0;
        state.pre_target_vel = 0;
    end

    error = setpoint - measurement;
    abs_error = abs(error);

    % 三区域自适应增益调度
    if abs_error > pid_ax.error_threshold_high
        Kp = pid_ax.Kp_base * pid_ax.Kp_high_ratio;
        Ki = pid_ax.Ki_base * pid_ax.Ki_high_ratio;
        Kd = pid_ax.Kd_base * pid_ax.Kd_high_ratio;
    elseif abs_error > pid_ax.error_threshold_low
        ratio = (abs_error - pid_ax.error_threshold_low) / ...
                (pid_ax.error_threshold_high - pid_ax.error_threshold_low + 1e-10);
        Kp = pid_ax.Kp_base * (1 + ratio * (pid_ax.Kp_high_ratio - 1));
        Ki = pid_ax.Ki_base * (1 - ratio * (1 - pid_ax.Ki_high_ratio));
        Kd = pid_ax.Kd_base * (1 - ratio * (1 - pid_ax.Kd_high_ratio));
    else
        Kp = pid_ax.Kp_base;
        Ki = pid_ax.Ki_base;
        Kd = pid_ax.Kd_base;
    end

    % 积分分离 + 限幅
    if abs_error <= pid_ax.I_Band
        state.integral = state.integral + error * dt;
        state.integral = max(min(state.integral, pid_ax.integral_max), -pid_ax.integral_max);
    else
        state.integral = 0;
    end

    % 微分低通滤波（基于测量值）
    d_raw = (measurement - state.prev_measurement) / dt;
    d_filtered = pid_ax.d_filter_alpha * d_raw + ...
                 (1 - pid_ax.d_filter_alpha) * state.prev_d_filtered;

    % PID 基础输出
    out = Kp * error + Ki * state.integral - Kd * d_filtered;

    % 修正后的前馈（使用 measurement，而不是 C 代码中 bug 的 measurement[k] - setpoint[k-1]）
    target_vel = (measurement - state.pre_target_position) / dt;
    target_acc = (target_vel - state.pre_target_vel) / dt;
    out = out + pid_ax.kv * target_vel + pid_ax.ka * target_acc;

    % 输出限幅 + 抗饱和
    if out > pid_ax.output_max
        out = pid_ax.output_max;
        state.integral = (out - Kp * error + Kd * d_filtered) / (Ki + 1e-10);
    elseif out < pid_ax.output_min
        out = pid_ax.output_min;
        state.integral = (out - Kp * error + Kd * d_filtered) / (Ki + 1e-10);
    end

    % 更新状态
    state.prev_measurement = measurement;
    state.prev_d_filtered = d_filtered;
    state.pre_target_position = setpoint;
    state.pre_target_vel = target_vel;
end


%% =========================================================================
%  S10: XY 坐标旋转（复刻 fc_ctrl.c Pos_Cmd_st 中的 POS_TRANS=1 逻辑）
% =========================================================================
function [cmd_vel_rot, rot_state] = xyRotateAndPid(pid_X, pid_Y, ...
    radar_pos_xy, target_pos_xy, rot_state, state_X, state_Y, dt)
% XYROTATEANDPID  XY 坐标旋转 + PID 计算
%
% 复刻 fc_ctrl.c 中 POS_TRANS=1 的逻辑：
%   1. 目标变化时，重设 start_point，计算 theta_1，reset PID 积分
%   2. 把 radar_pos 投影到"起点→目标"方向
%   3. X PID 跟踪距离，Y PID 跟踪 0（垂直偏差）
%   4. 输出反变换回雷达系
%
% 输入:
%   pid_X, pid_Y    - X/Y 轴 PID 参数
%   radar_pos_xy    - [x, y] 当前雷达位置（2 元素向量）
%   target_pos_xy   - [x, y] 当前目标位置
%   rot_state       - 旋转状态 struct（start_point, theta_1, setpoint_modulus,
%                     last_target, state_X, state_Y）
%   state_X, state_Y - X/Y PID 状态
%   dt              - 采样周期
%
% 输出:
%   cmd_vel_rot     - [vx, vy] 雷达系下的速度指令
%   rot_state       - 更新后的旋转状态

    if isempty(rot_state)
        rot_state.start_point = [0, 0];
        rot_state.theta_1 = 0;
        rot_state.setpoint_modulus = 0;
        rot_state.last_target = target_pos_xy;
        rot_state.state_X = [];
        rot_state.state_Y = [];
    end

    % 检测目标变化 → 重设坐标系
    if any(rot_state.last_target ~= target_pos_xy)
        rot_state.start_point = rot_state.last_target;
        delta = target_pos_xy - rot_state.start_point;
        if norm(delta) < 1e-6
            rot_state.theta_1 = 0;
        else
            rot_state.theta_1 = atan2(delta(2), delta(1));
        end
        rot_state.setpoint_modulus = norm(delta);
        rot_state.last_target = target_pos_xy;
        % 重置 PID 积分
        rot_state.state_X = [];
        rot_state.state_Y = [];
    end

    % 投影当前位置到"起点→目标"方向
    un_trans_pos = radar_pos_xy - rot_state.start_point;
    pos_modulus = norm(un_trans_pos);
    if pos_modulus < 1e-6
        theta_2 = 0;
    else
        theta_2 = atan2(un_trans_pos(2), un_trans_pos(1));
    end

    % 旋转后的坐标
    length_x = pos_modulus * cos(rot_state.theta_1 - theta_2);  % 沿路径方向
    length_y = pos_modulus * sin(rot_state.theta_1 - theta_2);  % 垂直方向

    % 在旋转坐标系中跑 PID
    [cmd_vel_X, rot_state.state_X] = pidSimulate(pid_X, rot_state.setpoint_modulus, ...
                                                  length_x, rot_state.state_X, dt);
    [cmd_vel_Y, rot_state.state_Y] = pidSimulate(pid_Y, 0, length_y, ...
                                                  rot_state.state_Y, dt);

    % 反变换回雷达系
    cmd_vel_rot = zeros(1, 2);
    cmd_vel_rot(1) = cmd_vel_X * cos(rot_state.theta_1) + cmd_vel_Y * sin(rot_state.theta_1);
    cmd_vel_rot(2) = cmd_vel_X * sin(rot_state.theta_1) - cmd_vel_Y * cos(rot_state.theta_1);
end


%% =========================================================================
%  稳态分析（适用于定点飞行等小扰动场景）
% =========================================================================
function ss = analyzeSteadyState(radar_pos_ax, target_pos_ax, cmd_vel_ax, cmd_vel_sim, pid_ax, dt)
% ANALYZESTEADYSTATE  从稳态/小扰动数据中提取 Kp、超调等指标
    error_ax = target_pos_ax - radar_pos_ax;
    valid = ~isnan(error_ax) & ~isnan(cmd_vel_ax);
    err = error_ax(valid);
    cmd = cmd_vel_ax(valid);
    cmd_sim = cmd_vel_sim(valid);

    % 基础统计
    ss.mean_error = mean(abs(err));
    ss.max_error = max(abs(err));
    ss.cmd_std = std(cmd);
    ss.cmd_min = min(cmd);
    ss.cmd_max = max(cmd);

    % 从数据估计有效 Kp：在小误差区间（|err| <= 5）内，cmd ≈ Kp * err
    % 用线性回归 cmd = Kp_eff * err 估计
    small_idx = abs(err) <= 5 & abs(err) > 0.1;  % 排除 0 附近（除零风险）和超大误差
    if sum(small_idx) > 10
        X = err(small_idx);
        Y = cmd(small_idx);
        % 稳健线性回归（忽略 I 和 D 项的贡献，仅估 P 项）
        Kp_eff = (X' * Y) / (X' * X + 1e-10);
        ss.Kp_eff = Kp_eff;
    else
        ss.Kp_eff = NaN;
    end

    % 小扰动超调估计：找到误差过零点，看之后反向超调多大
    ss.small_overshoot = estimateSmallOvershoot(err, dt);
end

function overshoot = estimateSmallOvershoot(err, dt)
% ESTIMATESMALLOVERSHOOT  从小扰动中估计超调
%   思路：找误差从正到负或从负到正的过零点，看之后最大反向偏置
    N = length(err);
    overshoots = [];
    % 找过零点
    for k = 2:N
        if err(k-1) * err(k) < 0  % 异号 = 过零
            % 之后 2 秒内的最大反向误差
            window = min(N, k + round(2/dt));
            after = err(k:window);
            if isempty(after), continue; end
            % 过零前误差方向
            sign_before = sign(err(k-1));
            % 之后的最大反向偏离
            max_reverse = max(-sign_before * after);
            if max_reverse > 0.1  % 排除噪声
                overshoots(end+1) = max_reverse;
            end
        end
    end
    if ~isempty(overshoots)
        % 相对超调 = 平均反向超调 / 过零前平均|误差|
        overshoot = median(overshoots);
    else
        overshoot = NaN;
    end
end


%% =========================================================================
%  S1: 输出
% =========================================================================
function result = buildPartialResult(quality, pid)
    result = struct();
    result.quality = quality;
    result.recommended = struct();
    result.recommended.warning = '数据激励不足，无法给出推荐';
end

function printConsoleReport(result, pid, tuningMode)
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════╗\n');
    fprintf('║        PID 性能诊断与调参建议报告                      ║\n');
    fprintf('╚══════════════════════════════════════════════════════════╝\n');
    fprintf('调参档位: %s\n', tuningMode);
    axis_names = {'X', 'Y', 'Z'};

    hasPidVal = isfield(result, 'pid_validation') && ~isempty(result.pid_validation);
    hasSteadyState = isfield(result, 'steady_state') && ~isempty(result.steady_state);
    hasRecommended = isfield(result, 'recommended') && ~isempty(result.recommended);
    hasMetrics = isfield(result, 'metrics') && ~isempty(result.metrics);

    for ax = 1:3
        ax_name = axis_names{ax};
        fprintf('\n========== %s 轴 ==========\n', ax_name);
        cur = pid.(ax_name);
        fprintf('  当前参数: Kp=%.3f  Ki=%.3f  Kd=%.3f\n', ...
            cur.Kp_base, cur.Ki_base, cur.Kd_base);

        %% Part A: PID 仿真对比
        if hasPidVal && isfield(result.pid_validation, ax_name)
            pv = result.pid_validation.(ax_name);
            fprintf('  [PID 仿真对比] (MATLAB PID 复刻 vs C 代码输出)\n');
            fprintf('    NRMSE = %.1f%%, 相关系数 = %.3f\n', pv.nrmse*100, pv.correlation);
            if pv.nrmse < 0.15
                fprintf('    ✅ 仿真与实际高度吻合\n');
            elseif pv.nrmse < 0.40
                fprintf('    ⚠️ 有一定差异（可能因自适应/前馈细微差别）\n');
            else
                fprintf('    ❌ 差异较大 —— 检查 C 代码是否有未复刻逻辑\n');
            end
        end

        %% 稳态分析
        if hasSteadyState && isfield(result.steady_state, ax_name)
            ss = result.steady_state.(ax_name);
            fprintf('  [稳态分析]\n');
            fprintf('    平均误差: %.3f | 最大误差: %.3f\n', ss.mean_error, ss.max_error);
            fprintf('    有效 Kp (从数据估): %.3f (设定: %.3f)\n', ss.Kp_eff, cur.Kp_base);
            if abs(ss.Kp_eff - cur.Kp_base) / (cur.Kp_base + 1e-10) > 0.3
                fprintf('    ⚠️ 有效 Kp 与设定值偏差 >30%%, 可能积分/微分项贡献显著\n');
            end
            fprintf('    控制量 std: %.3f | 范围: [%.2f, %.2f] / ±%d\n', ...
                ss.cmd_std, ss.cmd_min, ss.cmd_max, cur.output_max);
            if ~isnan(ss.small_overshoot)
                fprintf('    小扰动超调: %.1f%%\n', ss.small_overshoot*100);
                if ss.small_overshoot > 0.20
                    fprintf('    ⚠️ 超调 > 20%%, 考虑增大 Kd 以增强阻尼\n');
                end
            end
        end

        %% Part B: 调参建议（如果辨识成功）
        if hasRecommended && isfield(result.recommended, ax_name)
            rec = result.recommended.(ax_name);
            met = struct();
            if hasMetrics && isfield(result.metrics, ax_name)
                met = result.metrics.(ax_name);
            end
            fprintf('  [调参建议] (基于被控对象辨识)\n');
            fprintf('    建议: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', rec.Kp, rec.Ki, rec.Kd);
            fprintf('    预期: 超调=%.1f%%, settling=%.2fs, 相位裕度=%.1f°\n', ...
                met.overshoot*100, met.settling_time, met.phase_margin);
        end
    end
end

function plotAnalysis(result, pid, radar_pos, target_pos, cmd_vel, t, dt)
    figure('Name', 'PID 性能分析', 'Color', 'w', 'Position', [50, 50, 1400, 900]);
    axis_names = {'X', 'Y', 'Z'};
    colors = lines(3);
    hasPidVal = isfield(result, 'pid_validation') && ~isempty(result.pid_validation);

    %% 第一行：三轴位置跟踪
    for ax = 1:3
        subplot(3, 3, (ax-1)*3 + 1);
        plot(t, target_pos(:,ax), 'r--', 'LineWidth', 1.5); hold on;
        plot(t, radar_pos(:,ax), 'b-', 'LineWidth', 1.5);
        title(sprintf('%s 轴位置跟踪', axis_names{ax}));
        xlabel('时间 (s)'); ylabel('位置');
        legend('目标', '实际', 'Location', 'best');
        grid on;
    end

    %% 第二行：MATLAB PID 仿真 vs 实际 cmd_vel （核心对比图）
    for ax = 1:3
        subplot(3, 3, (ax-1)*3 + 2);
        if hasPidVal && isfield(result.pid_validation, axis_names{ax})
            cmd_sim = result.pid_validation.(axis_names{ax}).cmd_vel_sim;
            plot(t, cmd_vel(:,ax), 'b-', 'LineWidth', 1.5); hold on;
            plot(t, cmd_sim, 'r--', 'LineWidth', 1.5);
            nrmse = result.pid_validation.(axis_names{ax}).nrmse;
            title(sprintf('%s 轴 cmd_vel: 实际 (蓝) vs MATLAB PID 仿真 (红)  NRMSE=%.1f%%', ...
                axis_names{ax}, nrmse*100));
        else
            plot(t, cmd_vel(:,ax), 'k-', 'LineWidth', 1.5);
            title(sprintf('%s 轴 cmd_vel (无仿真结果)', axis_names{ax}));
        end
        xlabel('时间 (s)'); ylabel('cmd_vel');
        if hasPidVal
            legend('实际 cmd_vel', 'MATLAB PID 仿真', 'Location', 'best');
        end
        grid on;
    end

    %% 第三行：误差信号
    for ax = 1:3
        subplot(3, 3, (ax-1)*3 + 3);
        err = target_pos(:,ax) - radar_pos(:,ax);
        plot(t, err, 'Color', colors(ax,:), 'LineWidth', 1.2);
        % 稳态误差带
        hold on;
        if isfield(result, 'steady_state') && isfield(result.steady_state, axis_names{ax})
            ss = result.steady_state.(axis_names{ax});
            yline(ss.mean_error, 'g--', 'LineWidth', 1);
            yline(-ss.mean_error, 'g--', 'LineWidth', 1);
        end
        title(sprintf('%s 轴误差 (目标 - 实际)', axis_names{ax}));
        xlabel('时间 (s)'); ylabel('误差');
        grid on;
    end

    sgtitle('PID 性能分析: 仿真 vs 实际对比', 'FontSize', 14, 'FontWeight', 'bold');
end

function saveResult(result, csvPath)
    [~, name, ~] = fileparts(csvPath);
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    outName = sprintf('result_%s_%s.mat', name, timestamp);
    outPath = fullfile(fileparts(csvPath), outName);
    save(outPath, 'result');
    fprintf('   已保存: %s\n', outPath);
end
