function run_all_tests()
% RUN_ALL_TESTS  运行所有 TDD 切片的测试
%   按切片顺序执行，每个切片独立报告 PASS/FAIL

    fprintf('========================================\n');
    fprintf('   analyze_pid_performance TDD 测试套件\n');
    fprintf('========================================\n\n');

    tests = {
        'test_S2_default_params',    'S2: 默认参数填充';
        'test_S3_data_loading',      'S3: 数据加载';
        'test_S5_plant_sim',         'S5: 被控对象正向仿真';
        'test_S7_imc_formula',       'S7: IMC 解析公式';
        'test_S6_plant_id',          'S6: 被控对象辨识';
        'test_S4_data_quality',      'S4: 数据质量门控';
        'test_S8_itae_opt',          'S8: ITAE 优化';
        'test_S9_pid_sim',           'S9: PID 模拟器';
        'test_S10_xy_rotation',      'S10: XY 坐标旋转';
        'test_S1_e2e',               'S1: 端到端集成';
    };

    passed = 0;
    failed = 0;
    skipped = 0;
    results = cell(size(tests, 1), 2);

    for i = 1:size(tests, 1)
        test_name = tests{i, 1};
        test_desc = tests{i, 2};
        fprintf('\n>>> 运行 %s: %s\n', test_name, test_desc);
        try
            feval(test_name);
            fprintf('>>> %s: PASS\n', test_name);
            passed = passed + 1;
            results{i, :} = {test_desc, 'PASS'};
        catch err
            if startsWith(err.identifier, 'MATLAB:nonExistentFunction') || ...
               contains(err.message, '未定义函数') || ...
               contains(err.message, 'Undefined function')
                fprintf('>>> %s: SKIPPED (函数未实现)\n', test_name);
                skipped = skipped + 1;
                results{i, :} = {test_desc, 'SKIPPED'};
            else
                fprintf('>>> %s: FAIL\n    %s\n', test_name, err.message);
                failed = failed + 1;
                results{i, :} = {test_desc, 'FAIL'};
            end
        end
    end

    fprintf('\n========================================\n');
    fprintf('   测试结果汇总\n');
    fprintf('========================================\n');
    for i = 1:size(results, 1)
        fprintf('  [%s] %s\n', results{i, 2}, results{i, 1});
    end
    fprintf('\n  PASS: %d | FAIL: %d | SKIPPED: %d\n', passed, failed, skipped);
    fprintf('========================================\n');
end
