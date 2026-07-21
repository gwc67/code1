function overshoot_result = analyze_overshoot(csv_path)
% ANALYZE_OVERSHOOT 分析定点保持的超调量、调节时间、稳态误差
%
%   result = analyze_overshoot(csv_path)
%
%   输入： CSV 路径（需含 RADAR_POS_X/Y/Z, TARGET_POS_X/Y/Z, T_REL）
%
%   输出：
%     result.os.X/Y/Z       — 每轴超调指标
%     result.time_domain     — 时域指标汇总
%     result.fig             — 图形句柄

    %% 加载数据
    opts = detectImportOptions(csv_path);
    opts.DataLines = [2, Inf];
    data_raw = readmatrix(csv_path, opts);
    data_raw(all(isnan(data_raw), 2), :) = [];  % 去除全NaN行

    col_names = opts.VariableNames;
    radar_idx  = find(contains(col_names, "RADAR_POS"));
    target_idx = find(contains(col_names, "TARGET_POS"));
    trel_idx   = find(contains(col_names, 'T_REL'));

    radar_pos  = data_raw(:, radar_idx(1:3));
    target_pos = data_raw(:, target_idx(1:3));
    t = data_raw(:, trel_idx(1));

    %% 找定点区间（目标不变的段）
    [seg_starts, seg_ends] = find_segment_boundaries(target_pos);

    fprintf('\n========================================\n');
    fprintf('   定点保持 超调分析报告\n');
    fprintf('========================================\n\n');

    num_segs = length(seg_starts);
    fprintf('检测到 %d 个目标段\n\n', num_segs);

    if num_segs == 0
        error('未找到任何目标段，请检查 CSV 数据');
    end

    overshoot_result = struct();
    axis_names = {'X', 'Y', 'Z'};

    for seg_i = 1:num_segs
        si = seg_starts(seg_i);
        ei = seg_ends(seg_i);

        idx_range = si:ei;
        seg_len = length(idx_range);
        seg_duration = t(ei) - t(si);

        fprintf('--- 段 %d/%d: 行 [%d ~ %d], 时长 %.2fs ---\n', ...
            seg_i, num_segs, si, ei, seg_duration);
        fprintf('目标: [%.1f, %.1f, %.1f]\n\n', ...
            target_pos(si,1), target_pos(si,2), target_pos(si,3));

        if seg_len < 2
            fprintf('  (数据点不足, 跳过)\n\n');
            continue;
        end

        seg_result = struct();

        for ax = 1:3
            ax_name = axis_names{ax};
            sp = target_pos(idx_range, ax);
            meas = radar_pos(idx_range, ax);
            error_sig = meas - sp;
            sp_val = sp(1);

            % 基本统计
            error_mean = mean(error_sig);
            error_rms = sqrt(mean(error_sig.^2));
            error_max = max(abs(error_sig));

            % === 超调量 ===
            % 定义: 从初始位置到目标, 越过目标的最大距离
            initial_pos = meas(1);
            direction = sp_val - initial_pos;  % 期望方向

            if abs(direction) > 1e-3  % 有明显运动
                % 找到首次到达目标的时间
                if direction > 0
                    reach_idx = find(meas >= sp_val, 1, 'first');
                else
                    reach_idx = find(meas <= sp_val, 1, 'first');
                end

                if ~isempty(reach_idx)
                    % 首次到达后, 找最远 overshoot
                    if direction > 0
                        peak_val = max(meas(reach_idx:end));
                    else
                        peak_val = min(meas(reach_idx:end));
                    end
                    overshoot_abs = abs(peak_val - sp_val);
                    overshoot_pct = overshoot_abs / abs(direction) * 100;
                else
                    overshoot_abs = 0;
                    overshoot_pct = 0;
                end
            else
                overshoot_abs = 0;
                overshoot_pct = 0;
            end

            % === 调节时间 (2% 准则) ===
            threshold_2pct = max(abs(sp_val) * 0.02, 1);  % 至少1单位
            % 从后往前找, 最后超出阈值的位置
            outside = find(abs(error_sig) > threshold_2pct);
            if ~isempty(outside)
                last_outside_idx = outside(end);
                settling_time = t(idx_range(last_outside_idx)) - t(si);
            else
                settling_time = 0;
            end

            % === 振荡次数 ===
            % 过零次数 (相对于setpoint)
            zero_cross = sum(diff(sign(error_sig)) ~= 0);
            oscillation_count = floor(zero_cross / 2);

            % === 稳态误差 (最后 20% 数据) ===
            tail_start = max(1, round(seg_len * 0.8));
            ss_error = mean(abs(error_sig(tail_start:end)));
            ss_max_error = max(abs(error_sig(tail_start:end)));

            % === IAE & ITAE ===
            seg_t = t(idx_range);
            dt_seg = diff(seg_t);
            dt_seg = [dt_seg(1); dt_seg];  % 补齐
            iae = sum(abs(error_sig) .* dt_seg);
            itae = sum(abs(error_sig) .* (seg_t - seg_t(1)) .* dt_seg);

            % 输出
            fprintf('  %s 轴:\n', ax_name);
            fprintf('    超调量: %.2f (%.1f%%)\n', overshoot_abs, overshoot_pct);
            fprintf('    调节时间(2%%): %.3f s\n', settling_time);
            fprintf('    振荡次数: %d\n', oscillation_count);
            fprintf('    平均误差: %.2f\n', error_mean);
            fprintf('    RMS 误差: %.2f\n', error_rms);
            fprintf('    最大偏差: %.2f\n', error_max);
            fprintf('    稳态误差(mean|e|): %.2f (尾部20%%)\n', ss_error);
            fprintf('    稳态最大|e|: %.2f\n', ss_max_error);
            fprintf('    IAE: %.2f, ITAE: %.2f\n', iae, itae);
            fprintf('\n');

            seg_result.(ax_name).overshoot_abs = overshoot_abs;
            seg_result.(ax_name).overshoot_pct = overshoot_pct;
            seg_result.(ax_name).settling_time = settling_time;
            seg_result.(ax_name).oscillation_count = oscillation_count;
            seg_result.(ax_name).error_mean = error_mean;
            seg_result.(ax_name).error_rms = error_rms;
            seg_result.(ax_name).error_max = error_max;
            seg_result.(ax_name).ss_error = ss_error;
            seg_result.(ax_name).ss_max_error = ss_max_error;
            seg_result.(ax_name).IAE = iae;
            seg_result.(ax_name).ITAE = itae;
        end

        overshoot_result.([sprintf('seg%d', seg_i)]) = seg_result;
    end

    %% 绘图
    overshoot_result.fig = plot_overshoot_analysis(t, radar_pos, target_pos);

    fprintf('========================================\n');
    fprintf('  诊断建议:\n');
    fprintf('========================================\n');
    diagnose_stability(radar_pos, target_pos, t);
end


%% ========== 辅助函数 ==========

function [starts, ends] = find_segment_boundaries(target_pos)
    % 将数据按目标是否变化分成多段，返回起止行号数组
    n = size(target_pos, 1);
    if n == 0
        starts = []; ends = [];
        return;
    end

    starts = zeros(n, 1);
    ends   = zeros(n, 1);
    seg_count = 0;
    seg_start = 1;

    for k = 2:n
        if any(target_pos(k,:) ~= target_pos(k-1,:))
            % 目标发生变化: 关闭上一段, 开始新段
            seg_count = seg_count + 1;
            starts(seg_count) = seg_start;
            ends(seg_count)   = k - 1;
            seg_start = k;
        end
    end

    % 关闭最后一段（循环结束后统一处理，逻辑更清晰）
    seg_count = seg_count + 1;
    starts(seg_count) = seg_start;
    ends(seg_count)   = n;

    starts = starts(1:seg_count);
    ends   = ends(1:seg_count);
end


function fig = plot_overshoot_analysis(t, radar_pos, target_pos)
    fig = figure('Name', '定点保持 超调分析', 'Position', [100, 100, 900, 700]);

    axis_names = {'X', 'Y', 'Z'};
    for ax = 1:3
        subplot(3, 1, ax);
        plot(t, radar_pos(:,ax), 'b-', 'LineWidth', 1.2); hold on;
        plot(t, target_pos(:,ax), 'r--', 'LineWidth', 1.0);

        % 画 ±2% 调节带
        sp_mean = mean(target_pos(:,ax));
        band = max(abs(sp_mean) * 0.02, 1);
        yline(sp_mean + band, 'g:', 'LineWidth', 0.8);
        yline(sp_mean - band, 'g:', 'LineWidth', 0.8);
        yline(sp_mean, 'k:', 'LineWidth', 0.5);

        grid on;
        title(sprintf('%s 轴 — 雷达位置 vs 目标 (绿色虚线=2%%调节带)', axis_names{ax}));
        ylabel('位置');
        xlabel('时间 (s)');
        legend('雷达位置', '目标', 'Location', 'best');
    end

    % 第4子图: 误差时序
    figure(fig);
    subplot(4, 1, 4);
    error_xyz = radar_pos - target_pos;
    plot(t, error_xyz, 'LineWidth', 1.0);
    grid on;
    title('位置误差 (雷达 - 目标)');
    ylabel('误差');
    xlabel('时间 (s)');
    legend('X误差', 'Y误差', 'Z误差', 'Location', 'best');
end


function diagnose_stability(radar_pos, target_pos, t)
    % 简易诊断
    error_xyz = radar_pos - target_pos;
    error_mag = sqrt(sum(error_xyz(:,1:2).^2, 2));  % XY 平面误差

    % 检查是否有持续振荡
    % 对误差做 FFT, 看是否有明显的主频
    dt_median = median(diff(t));
    fs = 1 / dt_median;

    fprintf('\n  采样率: %.1f Hz\n', fs);

    for ax = 1:3
        ax_name = {'X','Y','Z'};
        e = error_xyz(:,ax);
        e = e - mean(e);  % 去直流

        N = length(e);
        if N < 64
            continue;
        end

        % 简单 FFT
        E = fft(e);
        P2 = abs(E(1:floor(N/2)+1)) / N;
        P2(2:end-1) = 2 * P2(2:end-1);
        f = fs * (0:floor(N/2)) / N;

        % 找主频 (忽略 DC)
        P2(1) = 0;
        [peak_val, peak_idx] = max(P2);
        peak_freq = f(peak_idx);

        fprintf('  %s 轴 主振荡频率: %.2f Hz (幅值: %.2f)\n', ...
            ax_name{ax}, peak_freq, peak_val);
    end

    % 总评
    rms_xy = sqrt(mean(error_mag.^2));
    max_xy = max(error_mag);
    fprintf('\n  XY 平面 RMS 误差: %.2f\n', rms_xy);
    fprintf('  XY 平面最大偏差: %.2f\n', max_xy);

    if rms_xy > 10
        fprintf('\n  ⚠ RMS > 10: 定点精度较差, 建议检查:\n');
        fprintf('    1. 内环姿态控制是否稳定\n');
        fprintf('    2. 机体系/雷达系坐标变换是否正确\n');
        fprintf('    3. 雷达数据延迟是否过大\n');
        fprintf('    4. PID 参数是否偏激进\n');
    end
end
