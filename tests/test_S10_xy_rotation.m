function test_S10_xy_rotation()
% TEST_S10_XY_ROTATION  切片 S10: XY 坐标旋转
%   验证目标变化时正确重设坐标系

    fprintf('\n========== 切片 S10: XY 坐标旋转 ==========\n');
    pass_count = 0; fail_count = 0;
    dt = 0.1;

    pid_X = getDefaultPidParams();
    pid_X = pid_X.X;
    pid_Y = pid_X;  % Y 与 X 相同

    %% Test 1: 首次调用 -> 初始化旋转状态
    fprintf('Test 1: 首次调用正确初始化...\n');
    try
        radar_pos = [0, 0];
        target_pos = [10, 0];  % 目标在 X 方向 10 单位
        [cmd_vel, rot_state] = xyRotateAndPid(pid_X, pid_Y, ...
            radar_pos, target_pos, [], [], [], dt);
        assert(~isempty(rot_state), '旋转状态应初始化');
        assert(abs(rot_state.theta_1 - 0) < 0.01, ...
            '目标在 X 轴上, theta_1 应为 0 (实际 %.3f)', rot_state.theta_1);
        assert(abs(rot_state.setpoint_modulus - 10) < 0.01, ...
            'setpoint_modulus 应为 10 (实际 %.3f)', rot_state.setpoint_modulus);
        fprintf('  PASS (theta_1=%.3f, modulus=%.3f)\n', ...
            rot_state.theta_1, rot_state.setpoint_modulus);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: 目标在 45° 方向 -> theta_1 = pi/4
    fprintf('Test 2: 目标在 45° 方向 -> theta_1 ≈ pi/4...\n');
    try
        target_pos = [10, 10];  % 45° 方向
        [~, rot_state] = xyRotateAndPid(pid_X, pid_Y, ...
            [0, 0], target_pos, [], [], [], dt);
        assert(abs(rot_state.theta_1 - pi/4) < 0.01, ...
            'theta_1 应为 pi/4 (实际 %.3f)', rot_state.theta_1);
        expected_mod = sqrt(200);
        assert(abs(rot_state.setpoint_modulus - expected_mod) < 0.1, ...
            'modulus 应 ≈ %.2f (实际 %.3f)', expected_mod, rot_state.setpoint_modulus);
        fprintf('  PASS (theta_1=%.3f rad = %.1f°)\n', rot_state.theta_1, rot_state.theta_1*180/pi);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 3: 目标变化 -> 重设坐标系，PID 状态清零
    fprintf('Test 3: 目标变化触发坐标系重设...\n');
    try
        % 第一次：目标在 [10, 0]
        [~, rs1] = xyRotateAndPid(pid_X, pid_Y, [0,0], [10,0], [], [], [], dt);
        % 构造一个有积分累积的状态
        rs1.state_X = struct('integral', 5, 'prev_measurement', 0, ...
                             'prev_d_filtered', 0, 'pre_target_position', 0, 'pre_target_vel', 0);
        rs1.state_Y = struct('integral', 3, 'prev_measurement', 0, ...
                             'prev_d_filtered', 0, 'pre_target_position', 0, 'pre_target_vel', 0);
        % 第二次：目标变化到 [0, 10]
        [~, rs2] = xyRotateAndPid(pid_X, pid_Y, [0,0], [0,10], rs1, rs1.state_X, rs1.state_Y, dt);
        % 目标变化后，PID 状态应被清空
        assert(isempty(rs2.state_X) || rs2.state_X.integral == 0, ...
            '目标变化后 X PID 状态应清零');
        assert(abs(rs2.theta_1 - pi/2) < 0.01, ...
            '新目标在 Y 轴, theta_1 应为 pi/2 (实际 %.3f)', rs2.theta_1);
        fprintf('  PASS (新 theta_1 = %.3f rad)\n', rs2.theta_1);
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 4: 目标不变 -> 坐标系保持
    fprintf('Test 4: 目标不变时坐标系保持...\n');
    try
        target = [10, 0];
        [~, rs1] = xyRotateAndPid(pid_X, pid_Y, [0,0], target, [], [], [], dt);
        theta_before = rs1.theta_1;
        % 第二次调用，相同目标
        [~, rs2] = xyRotateAndPid(pid_X, pid_Y, [1,0], target, rs1, [], [], dt);
        assert(rs2.theta_1 == theta_before, '目标不变，theta_1 应保持');
        fprintf('  PASS\n');
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S10 失败: %d 个测试', fail_count);
    end
end
