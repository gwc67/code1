function test_S7_imc_formula()
% TEST_S7_IMC_FORMULA  切片 S7: IMC 解析公式
%   针对 G(s) = K*e^(-tau*s) / (s(Ts+1)) 的 IMC 公式验证

    fprintf('\n========== 切片 S7: IMC 解析公式 ==========\n');
    pass_count = 0; fail_count = 0;

    %% Test 1: 标准参数 -> Ki = 1/(K*(lambda+tau))
    fprintf('Test 1: Ki 公式验证...\n');
    try
        K = 2.0; T = 0.5; tau = 0.1; lambda = 0.2;
        [Kp, Ki, Kd] = imcPid(K, T, tau, lambda);
        Ki_expected = 1 / (K * (lambda + tau));
        assert(abs(Ki - Ki_expected) < 1e-6, ...
            'Ki 应为 1/(K*(lambda+tau)) = %.4f (实际 %.4f)', Ki_expected, Ki);
        fprintf('  PASS (Ki = %.4f)\n', Ki);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: Kp = T / (K*(lambda+tau))
    fprintf('Test 2: Kp 公式验证...\n');
    try
        Kp_expected = T / (K * (lambda + tau));
        assert(abs(Kp - Kp_expected) < 1e-6, ...
            'Kp 应为 T/(K*(lambda+tau)) = %.4f (实际 %.4f)', Kp_expected, Kp);
        fprintf('  PASS (Kp = %.4f)\n', Kp);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 3: Kd = T*tau / (K*(lambda+tau))
    fprintf('Test 3: Kd 公式验证...\n');
    try
        Kd_expected = T * tau / (K * (lambda + tau));
        assert(abs(Kd - Kd_expected) < 1e-6, ...
            'Kd 应为 T*tau/(K*(lambda+tau)) = %.4f (实际 %.4f)', Kd_expected, Kd);
        fprintf('  PASS (Kd = %.4f)\n', Kd);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 4: lambda 增大 -> 增益减小（更保守）
    fprintf('Test 4: lambda 越大 -> Kp/Ki/Kd 越小...\n');
    try
        [Kp1, Ki1, Kd1] = imcPid(K, T, tau, 0.1);
        [Kp2, Ki2, Kd2] = imcPid(K, T, tau, 0.5);
        assert(Kp2 < Kp1 && Ki2 < Ki1 && Kd2 < Kd1, ...
            'lambda 增大应使所有增益减小');
        fprintf('  PASS\n');
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 5: K -> 0 应返回零增益（防除零）
    fprintf('Test 5: K 接近 0 -> 返回零增益...\n');
    try
        [Kp, Ki, Kd] = imcPid(1e-12, T, tau, 0.2);
        assert(Kp == 0 && Ki == 0 && Kd == 0, 'K 极小应返回 0');
        fprintf('  PASS\n');
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S7 失败: %d 个测试', fail_count);
    end
end
