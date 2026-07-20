function plot_radar_3d_optimized(csv_file)
    % 读取数据
    data = readtable(csv_file);
    
    % 创建图形窗口
    figure('Color', 'w', 'Position', [100, 100, 1200, 800]);
    hold on;
    
    % 1. 绘制雷达轨迹 (蓝线)
    % 优化点：适当增加线宽，避免过细导致放大后不可见
    p = plot3(data.RADAR_POS_X, data.RADAR_POS_Y, data.RADAR_POS_Z, ...
        'b-', 'LineWidth', 1.2, 'DisplayName', 'Radar Trajectory');
    
    % 2. 绘制目标点 (大红点)
    % 优化点：使用 filled 标记，增加边缘对比度
    scatter3(data.TARGET_POS_X, data.TARGET_POS_Y, data.TARGET_POS_Z, ...
        80, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.5, ...
        'DisplayName', 'Target Positions');
    
    % 3. 关键优化：解决显示问题
    axis equal;             % 保持XYZ比例一致，防止变形
    grid on;                % 开启网格辅助观察
    box on;                 % 开启边框
    
    % 4. 标签与标题
    xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Z Position (m)');
    title('3D Radar Trajectory & Targets (Optimized View)');
    legend('Location', 'best');
    
    % 5. 切换渲染器 (核心修复步骤)
    % 'painters' 渲染器对线条的支持比 OpenGL 更好，不易出现断线
    drawnow; 
    set(gcf, 'Renderer', 'painters'); 
    
    % 6. 视角微调 (可选)
    view(45, 30); 
end