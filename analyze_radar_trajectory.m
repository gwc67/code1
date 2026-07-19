function analyze_radar_trajectory(filePath)
% ANALYZE_RADAR_TRAJECTORY 分析雷达3D轨迹与目标点的差异
%   analyze_radar_trajectory(filePath) 
%   输入: filePath - CSV/Excel文件路径(需包含6列数据: A-C为轨迹, D-F为目标点)

    %% 1. 读取雷达实际轨迹 (A-C列)
    % 自动检测行数，避免硬编码106850导致换文件报错
    opts = detectImportOptions(filePath);
    opts.DataLines = [2, Inf]; % 从第2行读到末尾
    opts.SelectedVariableNames = opts.VariableNames(1:6);
    
    data = readmatrix(filePath, opts);
    x = data(:,1); y = data(:,2); z = data(:,3);
    
    %% 2. 读取并去重目标点 (D-F列)
    targets_raw = data(:,4:6);
    validRows = ~any(isnan(targets_raw), 2);
    targets_clean = targets_raw(validRows, :);
    targets = unique(targets_clean, 'rows', 'stable');
    
    fprintf('原始目标点: %d 个 → 去重后: %d 个 (去除 %.1f%%)\n', ...
            size(targets_raw,1), size(targets,1), ...
            (1 - size(targets,1)/max(size(targets_raw,1),1))*100);
    
    %% 3. 计算最近距离 (修复了原代码欧氏距离公式缺少括号的Bug)
    nTargets = size(targets, 1);
    minDist = zeros(nTargets, 1);
    nearestIdx = zeros(nTargets, 1);
    
    % 💡 强烈建议: 若轨迹>1万点，请取消下方注释使用pointCloud加速
    % pc = pointCloud([x, y, z]);
    % [nearestIdx, minDist] = findNearestNeighbors(pc, targets, 1);
    
    for i = 1:nTargets
        dists = sqrt((x - targets(i,1)).^2 + ...
                     (y - targets(i,2)).^2 + ...
                     (z - targets(i,3)).^2);  % ← 修复: 补上缺失的括号
        [minDist(i), nearestIdx(i)] = min(dists);
    end
    
    %% 4. 绘图
    figure('Color','w','Position',[100 100 1200 900]);
    plot3(x, y, z, 'b-', 'LineWidth', 1); hold on;
    scatter3(targets(:,1), targets(:,2), targets(:,3), ...
             120, 'r', 'filled', 'MarkerFaceAlpha', 0.8);
    
    % 💡 修复: 原代码 "for i = i:nTargets" 是死循环Bug，已改为 1:nTargets
    for i = 1:nTargets  
        idx = nearestIdx(i);
        plot3([targets(i,1), x(idx)], ...
              [targets(i,2), y(idx)], ...
              [targets(i,3), z(idx)], ...
              'r--', 'LineWidth', 1.5);
        
        midX = (targets(i,1) + x(idx)) / 2;
        midY = (targets(i,2) + y(idx)) / 2;
        midZ = (targets(i,3) + z(idx)) / 2;
        text(midX, midY, midZ, sprintf('%.3f', minDist(i)), ...
             'FontSize', 9, 'Color', 'm', 'FontWeight', 'bold');
    end
    
    axis equal; grid on; box on;
    xlabel('radar\_x'); ylabel('radar\_y'); zlabel('radar\_z');
    title(['雷达3D轨迹 vs 目标点差异分析: ', fileparts(filePath)]);
    legend('雷达实际轨迹', '目标点', '位置误差', 'Location', 'best');
    hold off;
    
    %% 5. 控制台输出量化差异
    fprintf('\n===== 目标点差异分析结果 =====\n');
    fprintf('%-8s %-12s %-12s\n', '目标编号', '最小距离(m)', '最近轨迹点索引');
    fprintf('%s\n', repmat('-', 1, 35));
    for i = 1:nTargets
        fprintf('%-8d %-12.4f %-12d\n', i, minDist(i), nearestIdx(i));
    end
    fprintf('平均偏差: %.4f m\n', mean(minDist));
    fprintf('最大偏差: %.4f m\n', max(minDist));
end