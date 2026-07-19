function test_S5_plant_sim()
% TEST_S5_PLANT_SIM  切片 S5：被控对象正向仿真
%   合成阶跃输入，验证输出形态（积分特性 + 延迟 + 时间常数）

    fprintf('\n========== 切片 S5: 被控对象正向仿真 ==========\n');
    pass_count = 0; fail_count = 0;
    dt = 0.1;

    %% Test 1: 阶跃输入 -> 输出应是斜坡（积分特性）
    fprintf('Test 1: 单位阶跃输入 -> 斜坡输出...\n');
    try
        N = 200;
        u = [zeros(10,1); ones(N-10, 1)];  % 阶跃
        K = 1.0; T = 0.3; tau = 0.1;
        y = simulatePlant(u, K, T, tau, dt);
        % 延迟：前 tau/dt 拍应为 0
        n_delay = round(tau/dt);
        assert(all(abs(y(1:n_delay)) < 1e-6), '延迟段应为 0');
        % 斜坡：后期近似线性增长
        slope = diff(y(end-50:end)) / dt;
        assert(all(abs(slope - K) < 0.1*K), ...
            '稳态斜率应接近 K (实际: %.3f)', mean(slope));
        fprintf('  PASS (稳态斜率 = %.3f, 期望 K = %.3f)\n', mean(slope), K);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: K=2 应使斜率翻倍
    fprintf('Test 2: 增益 K 线性影响斜率...\n');
    try
        N = 200;
        u = [zeros(10,1); ones(N-10, 1)];
        y1 = simulatePlant(u, 1.0, 0.3, 0.1, dt);
        y2 = simulatePlant(u, 2.0, 0.3, 0.1, dt);
        slope1 = (y1(end) - y1(end-10)) / (10*dt);
        slope2 = (y2(end) - y2(end-10)) / (10*dt);
        ratio = slope2 / slope1;
        assert(abs(ratio - 2) < 0.1, 'K 翻倍应使斜率翻倍 (实际比: %.3f)', ratio);
        fprintf('  PASS (斜率比 = %.3f)\n', ratio);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 3: 延迟越大，响应启动越晚
    fprintf('Test 3: 延迟 tau 决定响应起始...\n');
    try
        N = 200;
        u = [zeros(10,1); ones(N-10, 1)];
        y_short = simulatePlant(u, 1.0, 0.3, 0.1, dt);
        y_long  = simulatePlant(u, 1.0, 0.3, 0.5, dt);
        % 找首次非零
        idx_short = find(abs(y_short) > 0.01, 1);
        idx_long  = find(abs(y_long) > 0.01, 1);
        assert(idx_long > idx_short, '长延迟应比短延迟启动更晚');
        fprintf('  PASS (短延迟启动 idx=%d, 长延迟 idx=%d)\n', idx_short, idx_long);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 4: 零输入 -> 零输出
    fprintf('Test 4: 零输入 -> 零输出...\n');
    try
        u = zeros(100, 1);
        y = simulatePlant(u, 1.0, 0.3, 0.1, dt);
        assert(all(abs(y) < 1e-10), '零输入应得零输出');
        fprintf('  PASS\n');
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S5 失败: %d 个测试', fail_count);
    end
end
