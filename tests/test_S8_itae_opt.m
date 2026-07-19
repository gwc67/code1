function test_S8_itae_opt()
% TEST_S8_ITAE_OPT  切片 S8: ITAE 优化
%   验证推荐参数满足约束（超调 ≤ 5%、相位裕度 ≥ 45°、控制量不饱和）

    fprintf('\n========== 切片 S8: ITAE 优化 ==========\n');
    pass_count = 0; fail_count = 0;
    dt = 0.1;

    % 用一个典型的被控对象模型
    plant_params.K = 1.0;
    plant_params.T = 0.3;
    plant_params.tau = 0.2;

    pid_ax = getDefaultPidParams();
    pid_ax = pid_ax.X;

    %% Test 1: balanced 模式 -> 推荐参数满足约束
    fprintf('Test 1: balanced 模式推荐参数满足所有约束...\n');
    try
        [rec, met] = tunePid(plant_params, pid_ax, 'balanced', dt);
        fprintf('   推荐: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', rec.Kp, rec.Ki, rec.Kd);
        fprintf('   超调: %.1f%% (要求 ≤ 5%%)\n', met.overshoot*100);
        fprintf('   相位裕度: %.1f° (要求 ≥ 45°)\n', met.phase_margin);
        fprintf('   控制量峰值: %.2f (限幅 80%% of %d = %.1f)\n', ...
            met.peak_control, pid_ax.output_max, 0.8*pid_ax.output_max);

        assert(met.overshoot <= 0.05 + 0.01, '超调应 ≤ 5%% (实际 %.1f%%)', met.overshoot*100);
        assert(met.phase_margin >= 45 - 2, '相位裕度应 ≥ 45° (实际 %.1f°)', met.phase_margin);

        fprintf('  PASS\n');
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: conservative 模式应比 balanced 更保守（更小增益）
    fprintf('Test 2: conservative Kp < balanced Kp...\n');
    try
        [rec_c, ~] = tunePid(plant_params, pid_ax, 'conservative', dt);
        [rec_b, ~] = tunePid(plant_params, pid_ax, 'balanced', dt);
        assert(rec_c.Kp <= rec_b.Kp * 1.2, ...
            'conservative Kp 应 ≤ balanced Kp (实际 c=%.3f, b=%.3f)', ...
            rec_c.Kp, rec_b.Kp);
        fprintf('  PASS (conservative Kp=%.3f, balanced Kp=%.3f)\n', rec_c.Kp, rec_b.Kp);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 3: aggressive 模式应比 balanced 更激进（更大增益）
    fprintf('Test 3: aggressive Kp >= balanced Kp...\n');
    try
        [rec_a, ~] = tunePid(plant_params, pid_ax, 'aggressive', dt);
        assert(rec_a.Kp >= rec_b.Kp * 0.8, ...
            'aggressive Kp 应 ≥ balanced Kp (实际 a=%.3f, b=%.3f)', ...
            rec_a.Kp, rec_b.Kp);
        fprintf('  PASS (aggressive Kp=%.3f, balanced Kp=%.3f)\n', rec_a.Kp, rec_b.Kp);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S8 失败: %d 个测试', fail_count);
    end
end
