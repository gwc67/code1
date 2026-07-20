function plot_radar_3d_pro(csv_file)
    % PLOT_RADAR_3D_PRO - 专业级3D雷达轨迹可视化
    % 平衡3D视觉效果与显示稳定性
    
    % 1. 读取数据
    if ~exist(csv_file, 'file')
        error('文件未找到: %s', csv_file);
    end
    data = readtable(csv_file);
    
    % 2. 创建图形窗口 (强制使用OpenGL以支持真3D效果)
    fig = figure('Color', [0.1 0.1 0.1], ... % 深色背景增强3D对比度
                 'Position', [50, 50, 1400, 900], ...
                 'Renderer', 'opengl'); % 必须用OpenGL才有透视和光影
    
    ax = axes('Parent', fig, ...
              'Color', 'none', ...
              'XColor', [0.8 0.8 0.8], ...
              'YColor', [0.8 0.8 0.8], ...
              'ZColor', [0.8 0.8 0.8], ...
              'GridLineStyle', '-', ...
              'GridAlpha', 0.3);
    hold(ax, 'on');
    
    % 3. 绘制雷达轨迹 (优化抗锯齿和深度排序)
    h_radar = plot3(ax, data.RADAR_POS_X, data.RADAR_POS_Y, data.RADAR_POS_Z, ...
        'b-', 'LineWidth', 1.5, ...
        'DisplayName', 'Radar Trajectory', ...
        'LineJoin', 'round', ... % 圆角连接，减少折线锯齿
        'LineSmoothing', 'on'); % 开启线条平滑
    
    % 4. 绘制目标点 (增加立体感)
    h_target = scatter3(ax, data.TARGET_POS_X, data.TARGET_POS_Y, data.TARGET_POS_Z, ...
        60, [0.9 0.2 0.2], 'filled', ...
        'MarkerEdgeColor', 'w', ...
        'LineWidth', 0.5, ...
        'DisplayName', 'Target Positions');
    
    % 5. 【关键】设置真3D视觉参数
    view(ax, 3); % 恢复默认3D视角
    camproj(ax, 'perspective'); % 开启透视投影(近大远小)
    axis(ax, 'equal'); % 保持物理比例
    
    % 6. 智能相机定位 (解决"看起来像2D"的核心)
    % 自动计算数据范围，将相机放在合适距离
    xlim_data = [min(data.RADAR_POS_X) max(data.RADAR_POS_X)];
    ylim_data = [min(data.RADAR_POS_Y) max(data.RADAR_POS_Y)];
    zlim_data = [min(data.RADAR_POS_Z) max(data.RADAR_POS_Z)];
    
    % 设置坐标轴范围并留白
    padding_x = range(xlim_data) * 0.1;
    padding_y = range(ylim_data) * 0.1;
    padding_z = range(zlim_data) * 0.1;
    xlim(ax, xlim_data + [-padding_x, padding_x]);
    ylim(ax, ylim_data + [-padding_y, padding_y]);
    zlim(ax, zlim_data + [-padding_z, padding_z]);
    
    % 调整相机角度以获得最佳立体感
    camorbit(ax, -30, 25); % 旋转视角
    camzoom(ax, 0.9);      % 轻微缩放
    
    % 7. 添加辅助元素增强空间感
    grid(ax, 'on');
    xlabel(ax, 'X (m)', 'Color', 'w');
    ylabel(ax, 'Y (m)', 'Color', 'w');
    zlabel(ax, 'Z (m)', 'Color', 'w');
    title(ax, '3D Radar Trajectory Visualization', 'Color', 'w', 'FontSize', 14);
    legend(ax, 'Location', 'bestoutside', 'TextColor', 'w');
    
    % 8. 解决放大消失的终极补丁
    % 当用户放大时，MATLAB可能会因为深度缓冲精度丢失线段
    % 设置此属性可强制重绘顺序
    set(ax, 'SortMethod', 'childorder'); 
    
    disp('提示：如果放大后仍有极少量线段闪烁，请尝试旋转视角或稍微缩小视图。');
end