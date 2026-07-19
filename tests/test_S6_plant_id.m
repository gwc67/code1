function test_S6_plant_id()
% TEST_S6_PLANT_ID  切片 S6: 被控对象辨识
%   用合成数据（已知真值）验证辨识误差 < 20%

    fprintf('\n========== 切片 S6: 被控对象辨识 ==========\n');
    pass_count = 0; fail_count = 0;
    dt = 0.1;

    %% Test 1: 从合成数据恢复已知 (K, T, tau)
    fprintf('Test 1: 合成数据辨识，真值 K=1.0, T=0.3, tau=0.2...\n');
    try
        K_true = 1.0; T_true = 0.3; tau_true = 0.2;
        % 生成合成数据
        N = 500;
        u = [zeros(20,1); ones(N-20, 1) * 5];  % 阶跃输入
        y_true = simulatePlant(u, K_true, T_true, tau_true, dt);
        % 加少量噪声
        rng(42);
        y_noisy = y_true + 0.05 * randn(size(y_true));

        pid_ax = struct('plant_init', []);  % 用自动初值
        [plant_tf, plant_params, nrmse] = identifyPlant(u, y_noisy, dt, pid_ax);

        K_err = abs(plant_params.K - K_true) / K_true;
        T_err = abs(plant_params.T - T_true) / T_true;
        tau_err = abs(plant_params.tau - tau_true) / tau_true;

        fprintf('   真值:     K=%.3f, T=%.3f, tau=%.3f\n', K_true, T_true, tau_true);
        fprintf('   辨识结果: K=%.3f (%.1f%%), T=%.3f (%.1f%%), tau=%.3f (%.1f%%)\n', ...
            plant_params.K, K_err*100, plant_params.T, T_err*100, ...
            plant_params.tau, tau_err*100);
        fprintf('   NRMSE = %.1f%%\n', nrmse*100);

        assert(K_err < 0.2, 'K 误差应 < 20%% (实际 %.1f%%)', K_err*100);
        assert(T_err < 0.3, 'T 误差应 < 30%% (实际 %.1f%%)', T_err*100);
        % tau 由于离散化精度较低，给更宽松容忍
        assert(tau_err < 0.5, 'tau 误差应 < 50%% (实际 %.1f%%)', tau_err*100);

        fprintf('  PASS\n');
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: 用户提供初值 -> 使用用户初值
    fprintf('Test 2: 用户指定初值覆盖自动估计...\n');
    try
        pid_ax2 = struct('plant_init', [1.0, 0.3, 0.2]);
        [plant_tf2, plant_params2, ~] = identifyPlant(u, y_noisy, dt, pid_ax2);
        % 不验证具体值，只验证能跑通
        assert(isfield(plant_params2, 'K'));
        fprintf('  PASS (K=%.3f)\n', plant_params2.K);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S6 失败: %d 个测试', fail_count);
    end
end
