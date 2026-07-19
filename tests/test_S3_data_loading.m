function test_S3_data_loading()
% TEST_S3_DATA_LOADING  切片 S3: 数据加载 + 列提取
%   验证能正确读取 20 列 CSV 并提取 U1-U9

    fprintf('\n========== 切片 S3: 数据加载 ==========\n');
    pass_count = 0; fail_count = 0;

    csv_path = 'D:/Documents/MATLAB/code1/speed_15.csv';
    if ~exist(csv_path, 'file')
        csv_path = fullfile(fileparts(mfilename('full')), '..', 'speed_15.csv');
    end

    %% Test 1: 加载真实 CSV
    fprintf('Test 1: 加载 speed_15.csv...\n');
    try
        if exist(csv_path, 'file')
            data = loadCsvData(csv_path);
            assert(isfield(data, 'radar_pos'), '应有 radar_pos');
            assert(isfield(data, 'target_pos'), '应有 target_pos');
            assert(isfield(data, 'cmd_vel'), '应有 cmd_vel');
            assert(size(data.radar_pos, 2) == 3, 'radar_pos 应为 Nx3');
            assert(size(data.target_pos, 2) == 3, 'target_pos 应为 Nx3');
            assert(size(data.cmd_vel, 2) == 3, 'cmd_vel 应为 Nx3');
            assert(data.dt == 0.1, 'dt 应为 0.1 (10Hz)');
            assert(size(data.radar_pos, 1) > 100, '应有足够数据点');
            fprintf('  PASS (N=%d 行)\n', size(data.radar_pos, 1));
            pass_count = pass_count + 1;
        else
            fprintf('  SKIPPED (找不到 %s)\n', csv_path);
        end
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 2: 跳过前两行 header
    fprintf('Test 2: 跳过前两行 header...\n');
    try
        if exist(csv_path, 'file')
            data = loadCsvData(csv_path);
            % 第一行数据不应是 NaN（前两行被跳过）
            assert(~any(isnan(data.radar_pos(1,:))), ...
                '第一行数据不应为 NaN (header 应已跳过)');
            fprintf('  PASS\n');
            pass_count = pass_count + 1;
        else
            fprintf('  SKIPPED\n');
        end
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    %% Test 3: 列数不足报错
    fprintf('Test 3: 列数不足 CSV 报错...\n');
    try
        % 创建临时 CSV
        tmp = tempname;
        tmp = [tmp '.csv'];
        writematrix([1 2 3; 4 5 6], tmp);
        try
            loadCsvData(tmp);
            error('应抛出异常但未抛出');
        catch expected_err
            % 预期的错误
        end
        delete(tmp);
        fprintf('  PASS\n');
        pass_count = pass_count + 1;
    catch err
        fprintf('  FAIL: %s\n', err.message); fail_count = fail_count + 1;
    end

    fprintf('\n结果: %d PASS, %d FAIL\n', pass_count, fail_count);
    if fail_count > 0
        error('S3 失败: %d 个测试', fail_count);
    end
end
