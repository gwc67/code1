function result = identify_and_tune(csv_path, lambda_factor)
% IDENTIFY_AND_TUNE 从阶跃响应数据辨识被控对象 + IMC 计算 PID 参数
%
%   result = identify_and_tune(csv_path)
%   result = identify_and_tune(csv_path, lambda_factor)
%
%   输入:
%     csv_path      — CSV 路径（需含 RADAR_POS, TARGET_POS, CMD_SPEED, T_REL）
%     lambda_factor — IMC 调节参数 (默认 1.0, 越大越保守, 越小越激进)
%                      建议范围: 0.3 ~ 3.0
%
%   输出:
%     result.plant        — 辨识结果: K, T, tau (速度域 FOPDT)
%     result.recommended  — 推荐的 PID 参数 (X/Y 和 Z 轴)
%     result.validation   — 仿真对比结果
%
%   原理:
%     1. 对每个航点段, 提取 cmd_vel→位置响应 的阶跃数据
%     2. 对位置做差分得到速度响应
%     3. 对速度响应拟合 FOPDT: G(s) = K*e^(-τs)/(Ts+1)
%     4. 用 IMC 公式计算 PID 参数

    if nargin < 2 || isempty(lambda_factor)
        lambda_factor = 1.0;
    end

    %% 加载数据
    fprintf('\n[1/4] 加载数据: %s\n', csv_path);
    opts = detectImportOptions(csv_path);
    opts.DataLines = [2, Inf];
    data_raw = readmatrix(csv_path, opts);
    data_raw(all(isnan(data_raw), 2), :) = [];

    col_names = opts.VariableNames;
    radar_idx  = find(contains(col_names, "RADAR_POS"));
    target_idx = find(contains(col_names, "TARGET_POS"));
    cmd_idx    = find(contains(col_names, "CMD_SPEED"));
    trel_idx   = find(contains(col_names, 'T_REL'));

    radar_pos  = data_raw(:, radar_idx(1:3));
    target_pos = data_raw(:, target_idx(1:3));
    cmd_vel    = data_raw(:, cmd_idx(1:3));
    t = data_raw(:, trel_idx(1));

    dt_array = diff(t);
    dt_array = [dt_array(1); dt_array];
    dt_median = median(dt_array);

    fprintf('  数据点数: %d, 采样率: %.1f Hz\n', length(t), 1/dt_median);

    %% 分段
    [seg_starts, seg_ends] = find_step_segments(target_pos);
    num_segs = length(seg_starts);
    fprintf('  检测到 %d 个航点段\n\n', num_segs);

    %% 对每段辨识 + 合并结果
    fprintf('[2/4] 辨识被控对象 (FOPDT 模型拟合)...\n\n');

    all_models = struct();
    valid_count = 0;

    for i = 1:num_segs
        si = seg_starts(i);
        ei = seg_ends(i);
        seg_len = ei - si + 1;

        if seg_len < 20
            fprintf('  段 %d: 数据不足 (%d 点), 跳过\n', i, seg_len);
            continue;
        end

        seg_radar = radar_pos(si:ei, :);
        seg_cmd   = cmd_vel(si:ei, :);
        seg_t     = t(si:ei);
        seg_dt    = dt_array(si:ei);
        seg_target = target_pos(si, :);

        % 对 XYZ 各轴独立辨识
        for ax = 1:3
            ax_name = {'X','Y','Z'};

            % 提取该轴的阶跃响应
            [model, quality] = identify_fopdt_axis(...
                seg_radar(:,ax), seg_cmd(:,ax), seg_t, seg_dt, seg_target(ax));

            if ~isempty(model)
                valid_count = valid_count + 1;
                fprintf('  段%d %s轴: K=%.4f, T=%.4fs, tau=%.4fs (拟合质量: %.1f%%)\n', ...
                    i, ax_name{ax}, model.K, model.T, model.tau, quality*100);

                all_models(valid_count).axis = ax;
                all_models(valid_count).seg = i;
                all_models(valid_count).model = model;
                all_models(valid_count).quality = quality;
            end
        end
    end

    if valid_count == 0
        error('没有成功辨识的模型, 请检查数据');
    end

    %% 汇总各轴模型 (取加权平均)
    fprintf('\n[3/4] 汇总模型参数 + IMC 调参 (lambda=%.2f)...\n\n', lambda_factor);

    result.plant = struct();
    result.recommended = struct();

    ax_names = {'X','Y','Z'};
    for ax = 1:3
        ax_name = ax_names{ax};

        % 收集该轴所有辨识结果
        K_vals = []; T_vals = []; tau_vals = []; w = [];
        for m = 1:valid_count
            if all_models(m).axis == ax
                mdl = all_models(m).model;
                K_vals = [K_vals; mdl.K];
                T_vals = [T_vals; mdl.T];
                tau_vals = [tau_vals; mdl.tau];
                w = [w; all_models(m).quality];  % 用拟合质量作权重
            end
        end

        if isempty(K_vals)
            fprintf('  %s轴: 无有效辨识结果\n', ax_name);
            continue;
        end

        % 加权平均
        w = w / sum(w);
        K_avg = sum(K_vals .* w);
        T_avg = sum(T_vals .* w);
        tau_avg = sum(tau_vals .* w);

        fprintf('  %s轴: K=%.4f, T=%.4fs, tau=%.4fs (共 %d 组辨识)\n', ...
            ax_name, K_avg, T_avg, tau_avg, length(K_vals));

        result.plant.(ax_name) = struct('K', K_avg, 'T', T_avg, 'tau', tau_avg);

        % IMC 调参
        pid_params = imc_pid_tune(K_avg, T_avg, tau_avg, lambda_factor);
        result.recommended.(ax_name) = pid_params;

        fprintf('    IMC 推荐: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', ...
            pid_params.Kp, pid_params.Ki, pid_params.Kd);
    end

    %% 仿真验证
    fprintf('\n[4/4] 仿真验证...\n');
    result.validation = validate_pid_tuning(...
        radar_pos, target_pos, cmd_vel, t, dt_array, result.recommended);

    %% 绘图
    figure('Name', '系统辨识 + PID 调参验证', 'Position', [50, 50, 1100, 800]);

    % 子图1: 阶跃响应对比 (选质量最好的一段)
    [~, best_idx] = max([all_models.quality]);
    best = all_models(best_idx);
    si = seg_starts(best.seg);
    ei = seg_ends(best.seg);

    ax_labels = {'X', 'Y', 'Z'};
    for ax = 1:3
        subplot(3, 2, 2*ax-1);
        seg_t = t(si:ei) - t(si);
        meas = radar_pos(si:ei, ax);
        tgt = target_pos(si, ax);

        plot(seg_t, meas, 'b-', 'LineWidth', 1.2); hold on;
        yline(tgt, 'r--', 'LineWidth', 1);

        % 叠加模型预测
        if ~isempty(result.plant.(ax_labels{ax}))
            mdl = result.plant.(ax_labels{ax});
            step_response = mdl.K * (1 - exp(-(seg_t - mdl.tau) / mdl.T));
            step_response(seg_t < mdl.tau) = 0;
            plot(seg_t, tgt + step_response * (meas(end) - tgt), 'g-', 'LineWidth', 1);
            legend('实测', '目标', '模型预测', 'Location', 'best');
        else
            legend('实测', '目标', 'Location', 'best');
        end

        title(sprintf('%s 轴阶跃响应 (段 %d)', ax_labels{ax}, best.seg));
        xlabel('时间 (s)'); ylabel('位置');
        grid on;
    end

    % 子图2: 调参仿真对比
    if isfield(result, 'validation') && isfield(result.validation, 'sim_vel')
        for ax = 1:3
            subplot(3, 2, 2*ax);
            plot(t, cmd_vel(:,ax), 'b-', 'LineWidth', 0.8); hold on;
            plot(t, result.validation.sim_vel(:,ax), 'r--', 'LineWidth', 0.8);
            title(sprintf('%s 轴: 实际 cmd_vel vs 新 PID 仿真', ax_labels{ax}));
            legend('实际', '新PID仿真', 'Location', 'best');
            xlabel('时间 (s)'); ylabel('速度');
            grid on;
        end
    end

    fprintf('\n========================================\n');
    fprintf('  调参完成! 推荐参数:\n');
    fprintf('========================================\n');
    for ax = 1:3
        ax_name = ax_names{ax};
        if isfield(result.recommended, ax_name)
            p = result.recommended.(ax_name);
            fprintf('\n  %s 轴:\n', ax_name);
            fprintf('    Kp = %.4f\n', p.Kp);
            fprintf('    Ki = %.4f\n', p.Ki);
            fprintf('    Kd = %.4f\n', p.Kd);
        end
    end
    fprintf('\n');
end


%% ========== 子函数 ==========

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


function [model, quality] = identify_fopdt_axis(pos_data, cmd_data, t_vec, dt_vec, target_val)
    % 对单轴数据辨识 FOPDT 模型
    % 模型: G(s) = K * e^(-tau*s) / (T*s + 1)   (速度域)
    %
    % 输入: pos_data — 位置列向量
    %       cmd_data — 速度指令列向量
    %       t_vec    — 时间列向量
    %       dt_vec   — dt 列向量
    %       target_val — 目标值 (标量)

    model = [];
    quality = 0;

    n = length(pos_data);
    if n < 20
        return;
    end

    % 计算位置变化量
    pos_start = pos_data(1);
    pos_end = pos_data(end);
    delta_pos = pos_end - pos_start;

    % 如果没有明显的位置变化, 跳过
    if abs(delta_pos) < 0.5
        return;
    end

    % 对位置做差分得到速度 (用中心差分减少噪声)
    vel = zeros(n, 1);
    vel(2:end-1) = (pos_data(3:end) - pos_data(1:end-2)) ./ ...
                   (t_vec(3:end) - t_vec(1:end-2));
    vel(1) = (pos_data(2) - pos_data(1)) / (t_vec(2) - t_vec(1));
    vel(end) = (pos_data(end) - pos_data(end-1)) / (t_vec(end) - t_vec(end-1));

    % 平滑速度 (简单移动平均, 不依赖工具箱)
    win = min(5, floor(n/10));
    if win > 1
        kernel = ones(win, 1) / win;
        vel = conv(vel, kernel, 'same');
    end

    % 计算平均输入 (cmd_vel 在该段的均值)
    u_mean = mean(cmd_data);

    % 估计稳态速度增益
    % 对速度响应取后 30% 的平均作为稳态速度
    tail_start = max(1, round(n * 0.7));
    vel_ss = mean(vel(tail_start:end));

    if abs(vel_ss) < 1e-6 || abs(u_mean) < 1e-6
        return;
    end

    K = vel_ss / u_mean;  % 速度域增益: Δvel_ss / u

    % 估计死区时间 tau: 找到速度响应首次超过稳态值 5% 的时刻
    threshold = 0.05 * vel_ss;
    if vel_ss > 0
        first_response = find(vel >= threshold, 1, 'first');
    else
        first_response = find(vel <= threshold, 1, 'first');
    end

    if isempty(first_response)
        return;
    end

    tau = t_vec(first_response) - t_vec(1);
    tau = max(tau, dt_vec(1));  % 至少 1 个 dt

    % 估计时间常数 T: 找到速度达到稳态值 63.2% 的时刻
    threshold_632 = 0.632 * vel_ss;
    if vel_ss > 0
        t_632 = find(vel >= threshold_632, 1, 'first');
    else
        t_632 = find(vel <= threshold_632, 1, 'first');
    end

    if isempty(t_632)
        T = 3 * tau;  % 默认估计
    else
        T = (t_vec(t_632) - t_vec(1)) - tau;
        T = max(T, dt_vec(1));
    end

    % 拟合质量: 模型预测 vs 实测的 NRMSE
    vel_pred = K * u_mean * (1 - exp(-(t_vec - t_vec(1) - tau) / T));
    vel_pred(t_vec < t_vec(1) + tau) = 0;

    rmse = sqrt(mean((vel - vel_pred).^2));
    vel_range = max(vel) - min(vel);
    if vel_range > 1e-6
        quality = max(0, 1 - rmse / vel_range);
    else
        quality = 0;
    end

    model.K = K;
    model.T = T;
    model.tau = tau;
end


function pid = imc_pid_tune(K, T, tau, lambda)
    % IMC-PID 调参公式
    % 被控对象: G(s) = K * e^(-tau*s) / (T*s + 1)
    %
    % IMC 滤波器: f(s) = 1 / (lambda*s + 1)
    %
    % 对于积分过程 (位置 = 积分速度):
    % PID 参数由 IMC 推导

    % 位置环 PID (从位置误差到速度指令)
    % 等效被控对象: G_pos(s) = K / (s * (T*s + 1)) * e^(-tau*s)
    %
    % IMC 调参公式 (适用于积分 + 一阶滞后):
    %   Kp = (T + tau/2) / (K * (lambda + tau))
    %   Ki = 1 / (K * (lambda + tau))
    %   Kd = (T * tau/2) / (K * (lambda + tau))

    denom = K * (lambda + tau);

    if abs(denom) < 1e-10
        pid.Kp = 0; pid.Ki = 0; pid.Kd = 0;
        return;
    end

    pid.Kp = (T + tau/2) / denom;
    pid.Ki = 1 / denom;
    pid.Kd = (T * tau / 2) / denom;

    % 保证非负
    pid.Kp = max(pid.Kp, 0);
    pid.Ki = max(pid.Ki, 0);
    pid.Kd = max(pid.Kd, 0);
end


function val = validate_pid_tuning(radar_pos, target_pos, cmd_vel, t, dt_array, rec_pid)
    % 用推荐的 PID 参数做仿真, 对比实际 cmd_vel
    val = struct();

    n = size(radar_pos, 1);
    sim_vel = zeros(n, 3);

    % 简化的单轴 PID 仿真 (不做坐标旋转)
    for ax = 1:3
        ax_name = {'X','Y','Z'};
        if ~isfield(rec_pid, ax_name{ax})
            continue;
        end

        pid_ax = rec_pid.(ax_name{ax});
        integral = 0;
        prev_meas = 0;
        prev_d_filtered = 0;

        for k = 1:n
            sp = target_pos(k, ax);
            meas = radar_pos(k, ax);
            dt = dt_array(k);

            err = sp - meas;
            err_abs = abs(err);

            % 简化: 使用固定增益 (不启用自适应)
            Kp = pid_ax.Kp;
            Ki = pid_ax.Ki;
            Kd = pid_ax.Kd;

            % 积分 (带限幅)
            integral = integral + err * dt;
            integral = max(min(integral, 5), -5);

            % 微分滤波
            d_raw = (meas - prev_meas) / max(dt, 0.001);
            d_filtered = 0.8 * d_raw + 0.2 * prev_d_filtered;

            out = Kp * err + Ki * integral - Kd * d_filtered;

            % 输出限幅
            out = max(min(out, 23), -23);

            sim_vel(k, ax) = out;

            prev_meas = meas;
            prev_d_filtered = d_filtered;
        end
    end

    val.sim_vel = sim_vel;
end
