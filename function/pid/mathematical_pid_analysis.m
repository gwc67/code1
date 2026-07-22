function result = mathematical_pid_analysis(csv_path)
% MATHEMATICAL_PID_ANALYSIS 使用 MATLAB 控制工具箱进行数学化 PID 分析
%
%   功能:
%     1. 从飞行数据辨识被控对象模型
%     2. 使用 pidtune 自动计算最优 PID
%     3. 频域分析 (Bode, 稳定裕度)
%     4. 时域仿真验证
%     5. 输出调参建议
%
%   输入:
%     csv_path — CSV 文件路径
%
%   输出:
%     result.plant — 辨识的植物模型 (tf 对象)
%     result.pid_auto — pidtune 自动计算的 PID
%     result.pid_manual — 手动调整的 PID
%     result.stability — 稳定裕度分析
%     result.metrics — 性能指标对比

    %% 1. 加载数据
    fprintf('\n========== 数学化 PID 分析 ==========\n\n');
    fprintf('[1/5] 加载数据...\n');

    opts = detectImportOptions(csv_path);
    opts.DataLines = [2, Inf];
    data_raw = readmatrix(csv_path, opts);
    data_raw(all(isnan(data_raw), 2), :) = [];

    col_names = opts.VariableNames;
    radar_idx = find(contains(col_names, 'RADAR_POS'));
    target_idx = find(contains(col_names, 'TARGET_POS'));
    cmd_idx = find(contains(col_names, 'CMD_SPEED'));
    trel_idx = find(contains(col_names, 'T_REL'));

    radar_pos = data_raw(:, radar_idx(1:3));
    target_pos = data_raw(:, target_idx(1:3));
    cmd_vel = data_raw(:, cmd_idx(1:3));
    t = data_raw(:, trel_idx(1));

    dt_array = diff(t);
    dt_array = [dt_array(1); dt_array];
    Ts = median(dt_array);  % 采样周期

    fprintf('  数据点: %d, 采样率: %.1f Hz\n', length(t), 1/Ts);

    %% 2. 系统辨识
    fprintf('\n[2/5] 系统辨识 (控制工具箱)...\n');

    % 选择一段好的数据 (目标恒定,有明显响应)
    [seg_starts, seg_ends] = find_step_segments(target_pos);

    % 选择 Z 轴 (通常最干净)
    ax = 3;  % Z 轴
    seg_idx = 3;  % 第 3 段

    if seg_idx <= length(seg_starts)
        si = seg_starts(seg_idx);
        ei = seg_ends(seg_idx);

        u = cmd_vel(si:ei, ax);  % 输入
        y = radar_pos(si:ei, ax);  % 输出

        % 去均值
        u = u - mean(u);
        y = y - mean(y);

        % 创建 iddata 对象
        data = iddata(y, u, Ts);

        % 辨识传递函数 (2 阶 + 延迟)
        fprintf('  辨识 2 阶传递函数...\n');
        sys_tf = tfest(data, 2);

        % 辨识状态空间模型
        fprintf('  辨识状态空间模型...\n');
        sys_ss = ssest(data, 2);

        % 转换为传递函数便于分析
        sys_tf_ss = tf(sys_ss);

        fprintf('\n  辨识结果 (传递函数):\n');
        sys_tf_display = tf(sys_tf);  % 转换为标准 tf 对象便于显示
        disp(sys_tf_display);

        % 提取 FOPDT 参数 (用于对比)
        [K, T, tau] = extract_fopdt_params(sys_tf);
        fprintf('\n  等效 FOPDT: K=%.3f, T=%.3fs, tau=%.3fs\n', K, T, tau);

        result.plant = sys_tf;
        result.fopdt = struct('K', K, 'T', T, 'tau', tau);
    else
        error('没有足够的数据段进行辨识');
    end

    %% 3. PID 自动调参
    fprintf('\n[3/5] PID 自动调参 (pidtune)...\n');

    % 使用辨识的模型
    G = sys_tf;

    % 自动调参
    C_auto = pidtune(G, 'PID');

    % 提取 PID 参数 (使用 piddata 函数，兼容性好)
    [Kp_auto, Ki_auto, Kd_auto] = piddata(C_auto);

    fprintf('  pidtune 结果:\n');
    fprintf('    Kp = %.4f\n', Kp_auto);
    fprintf('    Ki = %.4f\n', Ki_auto);
    fprintf('    Kd = %.4f\n', Kd_auto);

    % 分析稳定裕度
    L_auto = C_auto * G;  % 开环
    [Gm_auto, Pm_auto] = margin(L_auto);

    fprintf('\n  稳定裕度:\n');
    fprintf('    增益裕度: %.1f dB\n', 20*log10(Gm_auto));
    fprintf('    相位裕度: %.1f°\n', Pm_auto);

    result.pid_auto = struct('Kp', Kp_auto, 'Ki', Ki_auto, 'Kd', Kd_auto);
    result.stability_auto = struct('Gm_dB', 20*log10(Gm_auto), 'Pm_deg', Pm_auto);

    %% 4. 手动调参 (基于经验公式)
    fprintf('\n[4/5] 手动调参 (Ziegler-Nichols)...\n');

    % Ziegler-Nichols 公式 (基于 FOPDT)
    Kz = result.fopdt.K;
    Tz = result.fopdt.T;
    tauz = result.fopdt.tau;

    % 检查参数有效性
    if ~isfinite(Kz) || ~isfinite(Tz) || ~isfinite(tauz) || ...
       abs(Kz) < 1e-10 || abs(tauz) < 1e-10 || Tz < 1e-10
        fprintf('  ⚠ FOPDT 参数无效 (K=%.3f, T=%.3f, tau=%.3f)\n', Kz, Tz, tauz);
        fprintf('  跳过 Ziegler-Nichols 调参\n');

        % 使用 pidtune 结果作为手动调参的替代
        Kp_zn = Kp_auto;
        Ki_zn = Ki_auto;
        Kd_zn = Kd_auto;
        C_manual = C_auto;
        Gm_manual = Gm_auto;
        Pm_manual = Pm_auto;
    else
        % PID 参数 (Ziegler-Nichols)
        Kp_zn = 1.2 * Tz / (Kz * tauz);
        Ki_zn = Kp_zn / (2 * tauz);
        Kd_zn = Kp_zn * tauz / 2;

        % 检查计算结果
        if ~isfinite(Kp_zn) || ~isfinite(Ki_zn) || ~isfinite(Kd_zn)
            fprintf('  ⚠ Ziegler-Nichols 计算结果无效\n');
            Kp_zn = Kp_auto;
            Ki_zn = Ki_auto;
            Kd_zn = Kd_auto;
        end

        C_manual = pid(Kp_zn, Ki_zn, Kd_zn);

        fprintf('  Ziegler-Nichols 结果:\n');
        fprintf('    Kp = %.4f\n', Kp_zn);
        fprintf('    Ki = %.4f\n', Ki_zn);
        fprintf('    Kd = %.4f\n', Kd_zn);

        % 分析稳定裕度
        L_manual = C_manual * G;
        [Gm_manual, Pm_manual] = margin(L_manual);

        fprintf('\n  稳定裕度:\n');
        fprintf('    增益裕度: %.1f dB\n', 20*log10(Gm_manual));
        fprintf('    相位裕度: %.1f°\n', Pm_manual);
    end

    result.pid_manual = struct('Kp', Kp_zn, 'Ki', Ki_zn, 'Kd', Kd_zn);
    result.stability_manual = struct('Gm_dB', 20*log10(Gm_manual), 'Pm_deg', Pm_manual);

    %% 5. 仿真对比
    fprintf('\n[5/5] 仿真对比...\n');

    % 生成阶跃响应
    t_sim = 0:Ts:10;  % 10 秒仿真

    % 自动调参的闭环响应
    T_auto = feedback(C_auto * G, 1);
    y_auto = step(T_auto, t_sim);

    % 手动调参的闭环响应
    T_manual = feedback(C_manual * G, 1);
    y_manual = step(T_manual, t_sim);

    % 计算性能指标
    metrics_auto = compute_step_metrics(t_sim, y_auto);
    metrics_manual = compute_step_metrics(t_sim, y_manual);

    fprintf('\n  性能指标对比:\n');
    fprintf('  %-20s %-15s %-15s\n', '指标', 'pidtune', 'Ziegler-Nichols');
    fprintf('  ------------------------------------------------\n');
    fprintf('  %-20s %-15.2f %-15.2f\n', '超调量 (%)', metrics_auto.overshoot_pct, metrics_manual.overshoot_pct);
    fprintf('  %-20s %-15.3f %-15.3f\n', '调节时间 (s)', metrics_auto.settling_time, metrics_manual.settling_time);
    fprintf('  %-20s %-15.3f %-15.3f\n', '上升时间 (s)', metrics_auto.rise_time, metrics_manual.rise_time);
    fprintf('  %-20s %-15.3f %-15.3f\n', 'IAE', metrics_auto.IAE, metrics_manual.IAE);
    fprintf('  %-20s %-15.3f %-15.3f\n', 'ITAE', metrics_auto.ITAE, metrics_manual.ITAE);

    result.metrics = struct('auto', metrics_auto, 'manual', metrics_manual);

    %% 6. 绘图
    fprintf('\n========== 分析完成 ==========\n\n');

    % 图 1: Bode 图
    figure('Name', 'Bode 图', 'Position', [100, 100, 800, 600]);
    subplot(2,1,1);
    bode(L_auto, 'b-', L_manual, 'r--');
    legend('pidtune', 'Ziegler-Nichols');
    title('开环 Bode 图');
    grid on;

    % 图 2: 阶跃响应对比
    subplot(2,1,2);
    step(T_auto, t_sim, 'b-', T_manual, t_sim, 'r--');
    legend('pidtune', 'Ziegler-Nichols');
    title('闭环阶跃响应对比');
    grid on;

    % 图 3: 根轨迹
    figure('Name', '根轨迹', 'Position', [100, 100, 600, 500]);
    rlocus(C_auto * G);
    title('根轨迹 (pidtune)');
    grid on;

    % 图 4: Nyquist 图
    figure('Name', 'Nyquist 图', 'Position', [100, 100, 600, 500]);
    nyquist(L_auto, 'b-', L_manual, 'r--');
    legend('pidtune', 'Ziegler-Nichols');
    title('Nyquist 图');
    grid on;

    %% 7. 输出建议
    fprintf('========================================\n');
    fprintf('  调参建议\n');
    fprintf('========================================\n\n');

    % 选择更好的方案
    if metrics_auto.ITAE < metrics_manual.ITAE
        fprintf('推荐方案: pidtune 自动调参\n\n');
        fprintf('  Kp = %.4f\n', Kp_auto);
        fprintf('  Ki = %.4f\n', Ki_auto);
        fprintf('  Kd = %.4f\n', Kd_auto);
        result.recommended = struct('Kp', Kp_auto, 'Ki', Ki_auto, 'Kd', Kd_auto);
    else
        fprintf('推荐方案: Ziegler-Nichols\n\n');
        fprintf('  Kp = %.4f\n', Kp_zn);
        fprintf('  Ki = %.4f\n', Ki_zn);
        fprintf('  Kd = %.4f\n', Kd_zn);
        result.recommended = struct('Kp', Kp_zn, 'Ki', Ki_zn, 'Kd', Kd_zn);
    end

    fprintf('\n稳定裕度检查:\n');
    if result.stability_auto.Pm_deg > 45
        fprintf('  ✓ 相位裕度充足 (%.1f° > 45°)\n', result.stability_auto.Pm_deg);
    else
        fprintf('  ⚠ 相位裕度不足 (%.1f° < 45°), 可能振荡\n', result.stability_auto.Pm_deg);
    end

    if result.stability_auto.Gm_dB > 6
        fprintf('  ✓ 增益裕度充足 (%.1f dB > 6 dB)\n', result.stability_auto.Gm_dB);
    else
        fprintf('  ⚠ 增益裕度不足 (%.1f dB < 6 dB)\n', result.stability_auto.Gm_dB);
    end
end


%% ========== 辅助函数 ==========

function [seg_starts, seg_ends] = find_step_segments(target_pos)
    % 找目标不变的段
    n = size(target_pos, 1);
    starts = zeros(n, 1);
    ends_arr = zeros(n, 1);
    seg_count = 0;
    seg_start = 1;

    for k = 2:n
        if any(target_pos(k,:) ~= target_pos(k-1,:))
            seg_count = seg_count + 1;
            starts(seg_count) = seg_start;
            ends_arr(seg_count) = k - 1;
            seg_start = k;
        end
    end
    seg_count = seg_count + 1;
    starts(seg_count) = seg_start;
    ends_arr(seg_count) = n;

    seg_starts = starts(1:seg_count);
    seg_ends = ends_arr(1:seg_count);
end


function [K, T, tau] = extract_fopdt_params(sys)
    % 从传递函数提取 FOPDT 参数
    % G(s) = K / (Ts + 1) * e^(-tau*s)

    % 获取直流增益
    K = dcgain(sys);

    % 检查增益有效性
    if ~isfinite(K) || abs(K) < 1e-10
        warning('直流增益无效或为零，使用默认值');
        K = 1.0;
        T = 1.0;
        tau = 0.1;
        return;
    end

    % 估计时间常数 (从阶跃响应)
    [y, t] = step(sys);
    y_final = y(end);

    % 检查阶跃响应有效性
    if ~isfinite(y_final) || abs(y_final) < 1e-10
        warning('阶跃响应无效，使用默认参数');
        T = 1.0;
        tau = 0.1;
        return;
    end

    % 估计延迟 (从阶跃响应首次响应)
    threshold = 0.05 * abs(y_final);
    if y_final > 0
        idx_delay = find(y >= threshold, 1, 'first');
    else
        idx_delay = find(y <= -threshold, 1, 'first');
    end

    if ~isempty(idx_delay)
        tau = t(idx_delay);
    else
        tau = 0;
    end

    % 找到 63.2% 的时间
    threshold_632 = 0.632 * abs(y_final);
    if y_final > 0
        idx_632 = find(y >= threshold_632, 1, 'first');
    else
        idx_632 = find(y <= -threshold_632, 1, 'first');
    end

    if ~isempty(idx_632)
        T = t(idx_632) - tau;
    else
        T = 1;  % 默认
    end

    % 确保参数合理
    T = max(T, 0.1);
    tau = max(tau, 0);
end


function metrics = compute_step_metrics(t, y)
    % 计算阶跃响应指标

    % 确保 t 和 y 是列向量
    t = t(:);
    y = y(:);

    y_final = y(end);

    % 超调量
    y_max = max(y);
    if y_max > y_final
        metrics.overshoot_pct = (y_max - y_final) / y_final * 100;
    else
        metrics.overshoot_pct = 0;
    end

    % 上升时间 (10% 到 90%)
    idx_10 = find(y >= 0.1 * y_final, 1, 'first');
    idx_90 = find(y >= 0.9 * y_final, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90)
        metrics.rise_time = t(idx_90) - t(idx_10);
    else
        metrics.rise_time = NaN;
    end

    % 调节时间 (2% 准则)
    threshold = 0.02 * abs(y_final);
    outside = find(abs(y - y_final) > threshold);
    if ~isempty(outside)
        metrics.settling_time = t(outside(end));
    else
        metrics.settling_time = 0;
    end

    % IAE
    dt = diff(t);
    dt = [dt(1); dt];  % 补齐到与 y 等长
    error = y_final - y;
    metrics.IAE = sum(abs(error) .* dt);

    % ITAE
    t_rel = t - t(1);  % 相对时间
    metrics.ITAE = sum(abs(error) .* t_rel .* dt);
end
