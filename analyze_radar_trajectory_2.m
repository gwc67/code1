function analyze_radar_trajectory_2(filePath)
% ANALYZE_RADAR_TRAJECTORY 分析雷达3D轨迹、目标点差异及速度切线方向
%   输入: filePath - CSV/Excel文件路径
%   列定义: A-C(轨迹xyz), D-F(目标点xyz), N(FC_SEN_SPD_X), O(FC_SEN_SPD_Y)

    %% 1. 读取数据并提取指定列
    opts = detectImportOptions(filePath);
    opts.DataLines = [2, Inf]; 
    
    % 读取全部数据（A-O共15列）
    data = readmatrix(filePath, opts);
    
    x = data(:,1); y = data(:,2); z = data(:,3);
    targets_raw = data(:,4:6);
    spd_x = data(:,14); % N列: FC_SEN_SPD_X
    spd_y = data(:,15); % O列: FC_SEN_SPD_Y
    
    %% 2. 计算合速度并处理缺失值
    spd_x(isnan(spd_x)) = 0;
    spd_y(isnan(spd_y)) = 0;
    speed = sqrt(spd_x.^2 + spd_y.^2);
    
    %% 3. 目标点去重
    validRows = ~any(isnan(targets_raw), 2);
    targets_clean = targets_raw(validRows, :);
    targets = unique(targets_clean, 'rows', 'stable');
    
    fprintf('轨迹点数: %d | 有效目标点: %d\n', length(x), size(targets,1));
    
    %% 4. 计算目标点最近距离 (使用pointCloud加速)
    nTargets = size(targets, 1);
    minDist = zeros(nTargets, 1);
    nearestIdx = zeros(nTargets, 1);
    
    if ~isempty(x) && nTargets > 0
        pc = pointCloud([x, y, z]);
        [nearestIdx, minDist] = findNearestNeighbors(pc, targets, 1);
    end
    
        %% 5. 绘图准备 (已修复放大消失问题)
    figure('Color','w','Position',[100 100 1200 900]);
    
    % ✅ 修复1: 将线宽从 0.8 提升到 1.5，防止缩放时线条低于光栅化阈值
    plot3(x, y, z, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5); 
    hold on;
    
    % 降采样提取箭头数据 (保持不变)
    numArrows = 80; 
    step = max(1, floor(length(x) / numArrows));
    idx_arrow = 1:step:length(x);
    qx = x(idx_arrow); qy = y(idx_arrow); qz = z(idx_arrow);
    qu = spd_x(idx_arrow); qv = spd_y(idx_arrow); 
    
    % ✅ 修复2: Z分量赋予极小非零值，打破严格共面，防止OpenGL裁剪
    qw = ones(size(qu)) * 1e-6;  
    
    % ✅ 修复3: 强制关闭 AutoScale，使用固定缩放因子，避免缩放时箭头重算消失
    %    同时设置 'Clipping','off' 禁止视锥体裁剪
    hQuiver = quiver3(qx, qy, qz, qu, qv, qw, 1.0, ...  
        'Color', 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, ...
        'AutoScale', 'off', 'Clipping', 'off');
    
    % 目标点与误差线绘制 (保持不变)
    scatter3(targets(:,1), targets(:,2), targets(:,3), ...
             120, 'r', 'filled', 'MarkerFaceAlpha', 0.8);
    for i = 1:size(targets,1)
        idx = nearestIdx(i);
        plot3([targets(i,1), x(idx)], [targets(i,2), y(idx)], ...
              [targets(i,3), z(idx)], 'k--', 'LineWidth', 1.2, 'Clipping','off'); 
    end
    
    axis equal; grid on; box on;
    % ✅ 额外保险: 手动扩展Z轴范围，确保XY平面上的元素不被裁切边界贴脸
    zlim_current = zlim;
    zRange = diff(zlim_current);
    if zRange < 1e-3  % 如果Z轴范围极小(说明轨迹几乎在一个平面上)
        zMid = mean(zlim_current);
        zlim([zMid - 1, zMid + 1]); 
    end
    
    xlabel('radar\_x'); ylabel('radar\_y'); zlabel('radar\_z');
    title(['雷达轨迹(含速度切线) vs 目标点差异']);
    % ... 图例代码同上一版，此处省略 ...
    hold off;
    
    %% 6. 图面美化与标注
    axis equal; grid on; box on;
    xlabel('radar\_x'); ylabel('radar\_y'); zlabel('radar\_z');
    title(['雷达轨迹(含速度切线) vs 目标点差异: ', fileparts(filePath)]);
    
    % 手动构建图例 (quiver3 不支持直接 legend，需借助辅助空图)
    hLine = plot3(NaN, NaN, NaN, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
    hArrow = plot3(NaN, NaN, NaN, 'b->', 'LineWidth', 1.5, 'MarkerSize', 8);
    hTarget = plot3(NaN, NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    hError = plot3(NaN, NaN, NaN, 'k--', 'LineWidth', 1.2);
    
    legend([hLine, hArrow, hTarget, hError], ...
           {'雷达实际轨迹', '速度切线方向 (X/Y)', '目标点', '位置误差'}, ...
           'Location', 'best');
    hold off;
    
    %% 7. 控制台输出量化差异
    fprintf('\n===== 目标点差异分析结果 =====\n');
    fprintf('%-8s %-12s %-12s\n', '目标编号', '最小距离(m)', '最近轨迹点索引');
    fprintf('%s\n', repmat('-', 1, 35));
    for i = 1:nTargets
        fprintf('%-8d %-12.4f %-12d\n', i, minDist(i), nearestIdx(i));
    end
    fprintf('平均偏差: %.4f m | 最大偏差: %.4f m\n', mean(minDist), max(minDist));
    fprintf('速度范围: %.2f ~ %.2f m/s\n', min(speed), max(speed));
end