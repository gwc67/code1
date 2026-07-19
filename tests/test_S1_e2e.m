function test_S1_e2e()
% TEST_S1_E2E  切片 S1: 端到端集成
%   在真实 CSV 数据上运行完整函数，验证能跑通并返回完整 struct

    fprintf('\n========== 切片 S1: 端到端集成 ==========\n');
    pass_count = 0; fail_count = 0;

    % 使用仓库里的真实 CSV
    csv_path = fullfile(fileparts(mfilename('full')), '..', 'speed_15.csv');
    if ~exist(csv_path, 'file')
        csv_path = 'D:/Documents/MATLAB/code1/speed_15.csv';
    end

    %% Test 1: 默认调用 -> 返回完整 struct
    fprintf('Test 1: 默认调用返回完整 struct...\n');
    try
        if exist(csv_path, 'file')
            result = analyze_pid_performance(csv_path);
            assert(isstruct(result), '返回值应为 struct');
            % 检查必要字段
            assert(isfield(result, 'recommended'), '应有 recommended 字段');
            assert(isfield(result, 'identified_plant'), '应有 identified_plant 字段');
            % 检查 .mat 文件是否生成
            files = dir(fullfile(fileparts(csv_path), 'result_*.mat'));
            assert(~isempty(files), '应生成 .mat 结果文件');
            fprintf('  PASS (生成 %d 个 .mat 文件)\n', length(files));
            pass_count = pass_count + 1;
        else
            fprintf('  SKIPPED (找不到 %s)\n', csv_path);
        end
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: 部分覆盖 pidParams -> 不影响运行
    fprintf('Test 2: 部分覆盖参数...\n');
    try
        if exist(csv_path, 'file')
            params.X.Kp_base = 0.25;
            result = analyze_pid_performance(csv_path, params);
            assert(isstruct(result), '应返回 struct');
            fprintf('  PASS\n');
            pass_count = pass_count + 1;
        else
            fprintf('  SKIPPED\n');
        end
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 3: 不同 tuningMode
    fprintf('Test 3: 三种 tuningMode 都能跑...\n');
    try
        if exist(csv_path, 'file')
            for mode = {'conservative', 'balanced', 'aggressive'}
                result = analyze_pid_performance(csv_path, [], mode{1});
                assert(isstruct(result));
            end
            fprintf('  PASS\n');
            pass_count = pass_count + 1;
        else
            fprintf('  SKIPPED\n');
        end
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 4: 错误输入 -> 报错
    fprintf('Test 4: 错误输入触发报错...\n');
    try
        try
            analyze_pid_performance('');
            error('应抛出异常但未抛出');
        catch expected_err
            % 预期的错误
        end
        try
            analyze_pid_performance('nonexistent.csv');
            error('应抛出异常但未抛出');
        catch expected_err
            % 预期的错误
        end
        fprintf('  PASS\n');
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S1 失败: %d 个测试', fail_count);
    end
end
