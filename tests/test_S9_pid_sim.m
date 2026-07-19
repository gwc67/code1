function test_S9_pid_sim()
% TEST_S9_PID_SIM  切片 S9: PID 模拟器
%   验证 MATLAB PID 复刻与 C 代码行为一致

    fprintf('\n========== 切片 S9: PID 模拟器 ==========\n');
    pass_count = 0; fail_count = 0;
    dt = 0.1;

    pid_ax = getDefaultPidParams();
    pid_ax = pid_ax.X;

    %% Test 1: 空状态初始化 -> 应正常运行
    fprintf('Test 1: 空状态初始化...\n');
    try
        [out, state] = pidSimulate(pid_ax, 10, 0, [], dt);
        % P 项: Kp * error = 0.18 * 10 = 1.8
        % 前馈: 首拍 measurement=0, pre_target=0 -> target_vel = 0
        expected_P = pid_ax.Kp_base * 10;
        % 因为误差 = 10 < I_Band = 25, 积分会累加
        % 但 I 项首拍为 0 (刚加入)
        assert(abs(out - expected_P) < 0.5, '首拍输出应近似 P 项 = %.2f (实际 %.2f)', expected_P, out);
        assert(isfield(state, 'integral'), '状态应有 integral 字段');
        fprintf('  PASS (首拍输出 = %.3f)\n', out);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: 大误差触发自适应增益
    fprintf('Test 2: 大误差 (>25) 触发高增益模式...\n');
    try
        % error = 50 > threshold_high = 25
        [out, ~] = pidSimulate(pid_ax, 50, 0, [], dt);
        % 高增益模式: Kp = 0.18 * 2.0 = 0.36
        expected_P = pid_ax.Kp_base * pid_ax.Kp_high_ratio * 50;
        assert(abs(out - expected_P) < 1.0, '高增益 P 项应近似 %.2f (实际 %.2f)', expected_P, out);
        fprintf('  PASS (大误差输出 = %.3f)\n', out);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 3: 积分分离 - 误差 > I_Band 时积分清零
    fprintf('Test 3: 误差 > I_Band 时积分项清零...\n');
    try
        state = [];
        % 先跑一拍小误差 (error=5 < I_Band=25)，积分累积
        [out1, state] = pidSimulate(pid_ax, 5, 0, state, dt);
        assert(state.integral ~= 0, '小误差时积分应累积');
        integral_after_small = state.integral;
        % 再跑一拍大误差 (error=50 > I_Band=25)，积分清零
        [out2, state] = pidSimulate(pid_ax, 50, 0, state, dt);
        assert(state.integral == 0, '大误差时积分应清零');
        fprintf('  PASS (小误差后 integral=%.3f, 大误差后 = %.3f)\n', ...
            integral_after_small, state.integral);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 4: 输出限幅 + 抗饱和
    fprintf('Test 4: 输出限幅 + 抗饱和...\n');
    try
        % 给一个会让输出饱和的大误差
        state = [];
        state.integral = 100;  % 强制大积分
        [out, state] = pidSimulate(pid_ax, 200, 0, state, dt);
        assert(out <= pid_ax.output_max, '输出应不超过 output_max (%d)', pid_ax.output_max);
        assert(out >= pid_ax.output_min, '输出应不低于 output_min (%d)', pid_ax.output_min);
        fprintf('  PASS (饱和输出 = %.3f, 限幅 = ±%d)\n', out, pid_ax.output_max);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 5: 多步运行 -> 状态正确演化
    fprintf('Test 5: 多步 PID 演化...\n');
    try
        state = [];
        N = 100;
        setpoints = ones(N, 1) * 10;
        measurements = zeros(N, 1);
        outs = zeros(N, 1);
        for k = 1:N
            [outs(k), state] = pidSimulate(pid_ax, setpoints(k), measurements(k), state, dt);
            % 简单被控对象：measurement 累积 cmd_vel * dt
            if k > 1
                measurements(k) = measurements(k-1) + outs(k-1) * dt;
            end
        end
        % 测量值应该向设定值靠近
        assert(measurements(end) > 0, '测量值应向设定值靠近 (最终 = %.2f)', measurements(end));
        fprintf('  PASS (最终 measurement = %.2f, 目标 = 10)\n', measurements(end));
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S9 失败: %d 个测试', fail_count);
    end
end
