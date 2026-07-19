function test_S2_default_params()
% TEST_S2_DEFAULT_PARAMS  切片 S2 的 RED 测试：默认 PID 参数填充
%   验证 pidParams struct 与 C 代码 PID_Init 默认值的合并逻辑
%   当前状态：RED（函数尚未实现）

    fprintf('\n========== 切片 S2: 默认参数填充 ==========\n');
    pass_count = 0;
    fail_count = 0;

    %% Test 1: 空输入 -> 返回完整默认 struct
    fprintf('Test 1: 空 pidParams -> 所有字段使用 C 代码默认值...\n');
    try
        defaults = getDefaultPidParams();
        assert(isfield(defaults, 'X'), '缺少 X 轴');
        assert(isfield(defaults, 'Y'), '缺少 Y 轴');
        assert(isfield(defaults, 'Z'), '缺少 Z 轴');
        % X 轴默认值来自 PID_ctrl.c PID_Init
        assert(defaults.X.Kp_base == 0.18, 'X Kp_base 应为 0.18');
        assert(defaults.X.Ki_base == 0.2,  'X Ki_base 应为 0.2');
        assert(defaults.X.Kd_base == 0.5,  'X Kd_base 应为 0.5');
        assert(defaults.X.output_max == 23, 'X output_max 应为 23');
        assert(defaults.X.integral_max == 5, 'X integral_max 应为 5');
        assert(defaults.X.I_Band == 25, 'X I_Band 应为 25');
        assert(defaults.X.d_filter_alpha == 0.8, 'X d_filter_alpha 应为 0.8');
        assert(defaults.X.error_threshold_high == 25, 'X th_high 应为 25');
        assert(defaults.X.error_threshold_low == 25, 'X th_low 应为 25');
        assert(defaults.X.Kp_high_ratio == 2.0, 'X Kp_high_ratio 应为 2.0');
        assert(defaults.X.Ki_high_ratio == 0.7, 'X Ki_high_ratio 应为 0.7');
        assert(defaults.X.Kd_high_ratio == 0.58, 'X Kd_high_ratio 应为 0.58');
        assert(defaults.X.kv == 1.0, 'X kv 应为 1.0');
        assert(defaults.X.ka == 2.27, 'X ka 应为 2.27');
        % Y 默认值应与 X 相同（C 代码：loc_xyz_pst[Y_em] = loc_xyz_pst[X_em]）
        assert(defaults.Y.Kp_base == defaults.X.Kp_base, 'Y 默认应复制 X');
        % Z 轴不同默认值
        assert(defaults.Z.Kp_base == 0.23, 'Z Kp_base 应为 0.23');
        assert(defaults.Z.Ki_base == 0.03, 'Z Ki_base 应为 0.03');
        assert(defaults.Z.Kd_base == 0.08, 'Z Kd_base 应为 0.08');
        assert(defaults.Z.output_max == 20, 'Z output_max 应为 20');
        assert(defaults.Z.I_Band == 20, 'Z I_Band 应为 20');
        assert(defaults.Z.error_threshold_high == 10, 'Z th_high 应为 10');
        assert(defaults.Z.error_threshold_low == 2, 'Z th_low 应为 2');
        assert(defaults.Z.ka == 2.22, 'Z ka 应为 2.22');
        fprintf('  PASS\n'); pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: 部分覆盖 -> 仅覆盖字段改变，其他保持默认
    fprintf('Test 2: 部分覆盖 -> 仅用户指定的字段改变...\n');
    try
        user = struct();
        user.X.Kp_base = 0.25;   % 只改 X Kp
        merged = mergePidParams(user);
        assert(merged.X.Kp_base == 0.25, 'X Kp_base 应被覆盖为 0.25');
        assert(merged.X.Ki_base == 0.2,  'X Ki_base 应保持默认 0.2');
        assert(merged.X.Kd_base == 0.5,  'X Kd_base 应保持默认 0.5');
        assert(merged.Y.Kp_base == 0.18, 'Y Kp_base 应保持默认 0.18');
        assert(merged.Z.Kp_base == 0.23, 'Z Kp_base 应保持默认 0.23');
        fprintf('  PASS\n'); pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 3: 空 struct -> 等价于空输入
    fprintf('Test 3: 空 struct 输入 -> 全部默认...\n');
    try
        merged = mergePidParams(struct());
        assert(merged.X.Kp_base == 0.18);
        assert(merged.Z.Kd_base == 0.08);
        fprintf('  PASS\n'); pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 4: 不传入 pidParams（调用方传 []）-> 全部默认
    fprintf('Test 4: 传入 [] -> 全部默认...\n');
    try
        merged = mergePidParams([]);
        assert(merged.X.Kp_base == 0.18);
        fprintf('  PASS\n'); pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 5: 覆盖整轴 -> 该轴所有字段都来自用户
    fprintf('Test 5: 覆盖整轴 -> 该轴所有字段被替换...\n');
    try
        user = struct();
        user.Z = struct('Kp_base', 0.5, 'Ki_base', 0.1, 'Kd_base', 0.2, ...
                        'output_max', 30, 'output_min', -30, ...
                        'integral_max', 10, 'I_Band', 30, ...
                        'd_filter_alpha', 0.9, ...
                        'error_threshold_high', 15, 'error_threshold_low', 5, ...
                        'Kp_high_ratio', 1.5, 'Ki_high_ratio', 0.8, 'Kd_high_ratio', 0.6, ...
                        'kv', 0, 'ka', 0);
        merged = mergePidParams(user);
        assert(merged.Z.Kp_base == 0.5);
        assert(merged.Z.output_max == 30);
        % 其他轴仍是默认
        assert(merged.X.Kp_base == 0.18);
        fprintf('  PASS\n'); pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% 结果汇总
    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S2 RED: %d 个测试失败（预期，因为函数尚未实现）', fail_count);
    end
end
