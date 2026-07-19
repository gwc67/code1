function analyze_pid_performance(filePath)
% ANALYZE_PID_PERFORMANCE 分析无人机雷达PID控制性能并给出调参建议
%   输入: filePath - CSV/Excel 数据文件路径
%   列定义: 
%       A-C (雷达实际位置 X,Y,Z)
%       D-F (目标位置 X,Y,Z)
%       G-I (PID输出速度指令 X,Y,Z)

    %% 1. 读取数据
    fprintf('正在读取数据: %s ...\n', filePath);
    opts = detectImportOptions(filePath);
    opts.DataLines = [2, Inf]; 
    data = readmatrix(filePath, opts);
    
    % 提取位置与输出数据
    radar_pos = data(:, 1:3);   % A-C: 实际位置 (测量值)
    target_pos = data(:, 4:6);  % D-F: 目标位置 (设定值)
    cmd_vel = data(:, 7:9);     % G-I: PID输出 (控制量)
    
    % 计算时间向量 (假设采样周期为 10ms，根据你的 fc_ctrl.c 中的 10ms task 推断)
    dt = 0.01; 
    t = (0:size(data,1)-1)' * dt;
    
    % 计算误差 (目标 - 实际)
    error = target_pos - radar_pos;
    
    %% 2. 逆向还原 C 代码中的 PID 各项贡献 (以 X 轴为例进行深度分析)
    % 注：这里我们提取 C 代码中的默认参数来模拟，以便分离 P, I, D 项
    % X/Y 轴参数 (来自 PID_Init)
    Kp_base = 0.18; Ki_base = 0.2; Kd_base = 0.5;
    integral_max = 5; I_Band = 25; output_max = 23;
    d_filter_alpha = 0.8;
    
    axis_names = {'X轴 (前后)', 'Y轴 (左右)', 'Z轴 (高度)'};
    
    figure('Name', 'PID 性能深度分析', 'Color', 'w', 'Position', [50, 50, 1400, 900]);
    
    for axis = 1:3
        % 如果是 Z 轴，切换为 Z 轴的 PID 参数
        if axis == 3
            Kp_base = 0.23; Ki_base = 0.03; Kd_base = 0.08;
            integral_max = 5; I_Band = 20; output_max = 20;
        end
        
        err = error(:, axis);
        meas = radar_pos(:, axis);
        out_actual = cmd_vel(:, axis);
        
        % 模拟 C 代码中的积分和微分计算
        integral_sim = zeros(size(err));
        d_filtered_sim = zeros(size(err));
        P_term = zeros(size(err));
        I_term = zeros(size(err));
        D_term = zeros(size(err));
        
        for k = 2:length(err)
            % 积分分离逻辑
            if abs(err(k)) <= I_Band
                integral_sim(k) = integral_sim(k-1) + err(k) * dt;
                integral_sim(k) = max(min(integral_sim(k), integral_max), -integral_max);
            else
                integral_sim(k) = 0;
            end
            
            % 微分低通滤波逻辑 (基于测量值微分)
            d_raw = (meas(k) - meas(k-1)) / dt;
            d_filtered_sim(k) = d_filter_alpha * d_raw + (1 - d_filter_alpha) * d_filtered_sim(k-1);
            
            % 计算各项贡献 (使用基础参数近似，忽略自适应变化以看清基准趋势)
            P_term(k) = Kp_base * err(k);
            I_term(k) = Ki_base * integral_sim(k);
            D_term(k) = -Kd_base * d_filtered_sim(k); % 注意 C 代码中 D 项是减去
        end
        
        %% 3. 性能指标计算
        % 寻找阶跃响应区间 (目标位置发生显著变化的区间)
        target_diff = abs(diff(target_pos(:, axis)));
        step_idx = find(target_diff > max(target_diff)*0.5, 1); % 找到最大阶跃点
        
        if isempty(step_idx)
            step_idx = 1;
        end
        
        % 计算稳态误差 (最后 20% 数据的平均误差)
        steady_state_start = round(length(err) * 0.8);
        steady_state_error = mean(abs(err(steady_state_start:end)));
        
        % 计算超调量 (粗略估计)
        max_overshoot = max(radar_pos(step_idx:end, axis) - target_pos(step_idx:end, axis));
        
        %% 4. 绘图
        subplot(3, 2, (axis-1)*2 + 1);
        yyaxis left;
        plot(t, target_pos(:, axis), 'r--', 'LineWidth', 1.5); hold on;
        plot(t, radar_pos(:, axis), 'b-', 'LineWidth', 1.5);
        ylabel('位置 (m)');
        title(sprintf('%s 位置跟踪曲线', axis_names{axis}));
        legend('目标位置', '实际位置', 'Location', 'best');
        grid on;
        
        yyaxis right;
        plot(t, out_actual, 'k-', 'LineWidth', 1.2);
        ylabel('PID输出 (速度指令)');
        
        subplot(3, 2, (axis-1)*2 + 2);
        plot(t, P_term, 'r-', 'LineWidth', 1.2); hold on;
        plot(t, I_term, 'g-', 'LineWidth', 1.2);
        plot(t, D_term, 'm-', 'LineWidth', 1.2);
        plot(t, out_actual, 'k--', 'LineWidth', 1.5);
        ylabel('控制量分解');
        title(sprintf('%s PID 各项贡献分解 (模拟)', axis_names{axis}));
        legend('P项 (比例)', 'I项 (积分)', 'D项 (微分)', '实际总输出', 'Location', 'best');
        grid on;
        
        %% 5. 控制台输出调参建议
        fprintf('\n========== %s PID 性能诊断与调参建议 ==========\n', axis_names{axis});
        fprintf('当前基础参数: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp_base, Ki_base, Kd_base);
        fprintf('稳态误差: %.4f | 最大超调估计: %.4f\n', steady_state_error, max_overshoot);
        
        % 启发式调参建议逻辑
        if steady_state_error > 2.0
            fprintf('⚠️ 【诊断】稳态误差过大 (>2.0)。\n');
            fprintf('💡 【建议】增大 Ki (当前 %.2f -> 建议 %.2f) 以消除静差；或增大 Kp 提高系统刚度。\n', Ki_base, Ki_base * 1.5);
            fprintf('   注意：检查 I_Band (%.0f) 是否过小导致积分项频繁被清零。\n', I_Band);
        elseif max_overshoot > 5.0
            fprintf('⚠️ 【诊断】超调量过大 (>5.0)，系统阻尼不足。\n');
            fprintf('💡 【建议】增大 Kd (当前 %.2f -> 建议 %.2f) 以增强阻尼抑制震荡；或适当减小 Kp。\n', Kd_base, Kd_base * 1.3);
        elseif max(abs(D_term)) < max(abs(P_term)) * 0.1
            fprintf('⚠️ 【诊断】微分项 (D) 贡献极小，可能未起到抑制毛刺的作用。\n');
            fprintf('💡 【建议】检查传感器噪声，若噪声大需减小 d_filter_alpha (当前 %.1f)；若噪声小则增大 Kd。\n', d_filter_alpha);
        elseif max(abs(I_term)) >= integral_max * 0.9
            fprintf('⚠️ 【诊断】积分项频繁触及限幅 (%.0f)，存在积分饱和风险。\n', integral_max);
            fprintf('💡 【建议】增大 integral_max，或检查抗饱和逻辑是否生效。当前 C 代码已包含抗饱和，表现良好。\n');
        else
            fprintf('✅ 【诊断】系统响应良好，无明显稳态误差或剧烈超调。\n');
            fprintf('💡 【建议】当前参数较优。若需更快响应，可微调 Kp (增加 10%%) 并同步微调 Kd。\n');
        end
    end
    
    sgtitle('无人机雷达 PID 控制系统深度分析报告', 'FontSize', 16, 'FontWeight', 'bold');
end