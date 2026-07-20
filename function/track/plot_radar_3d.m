function plot_radar_3d(csv_file, varargin)
% PLOT_RADAR_3D 三维雷达轨迹与目标点可视化
%   plot_radar_3d(csv_file) 从指定CSV文件绘制雷达轨迹（蓝线）和目标点（大红点）
%
%   可选参数（使用'参数名',值对指定）:
%     'LineWidth'   : 雷达轨迹线宽 (默认 2.5)
%     'MarkerSize'  : 目标点大小 (默认 80)
%     'RadarColor'  : 雷达轨迹颜色 (默认 'b')
%     'TargetColor' : 目标点颜色 (默认 [0.8 0 0])
%     'ShowGrid'    : 是否显示网格 (true/false, 默认 true)
%     'EqualAxis'   : 是否保持坐标轴比例相等 (true/false, 默认 true)
%     'Title'       : 图表标题 (默认 '3D Radar Trajectory and Target Positions')
%
% 示例:
%   plot_radar_3d('radar_data.csv');
%   plot_radar_3d('data.csv', 'LineWidth', 3, 'MarkerSize', 100, 'Title', '实验1');

% 默认参数设置
params = struct(...
    'LineWidth', 2.5, ...
    'MarkerSize', 80, ...
    'RadarColor', 'b', ...
    'TargetColor', [0.8 0 0], ...
    'ShowGrid', true, ...
    'EqualAxis', true, ...
    'Title', '3D Radar Trajectory and Target Positions'...
    );

% 解析可选参数
pnames = fieldnames(params);
for i = 1:2:length(varargin)
    if i+1 <= length(varargin)
        param_name = varargin{i};
        param_val = varargin{i+1};
        
        if ismember(param_name, pnames)
            params.(param_name) = param_val;
        else
            warning('未知参数: %s，将使用默认值', param_name);
        end
    end
end

% 读取CSV数据
if ~isfile(csv_file)
    error('文件不存在: %s', csv_file);
end
data = readtable(csv_file);

% 验证必需列是否存在
required_cols = {'RADAR_POS_X','RADAR_POS_Y','RADAR_POS_Z',...
                 'TARGET_POS_X','TARGET_POS_Y','TARGET_POS_Z'};
for i = 1:length(required_cols)
    if ~isvarname(required_cols{i}) || ~ismember(required_cols{i}, data.Properties.VariableNames)
        error('CSV文件缺少必需列: %s', required_cols{i});
    end
end

% 提取雷达位置数据（轨迹）
radar_x = data.RADAR_POS_X;
radar_y = data.RADAR_POS_Y;
radar_z = data.RADAR_POS_Z;

% 提取目标位置数据
target_x = data.TARGET_POS_X;
target_y = data.TARGET_POS_Y;
target_z = data.TARGET_POS_Z;

% 创建三维图形
figure('Position', [100, 100, 1200, 800], 'Color', 'white');
hold on;

% 绘制雷达轨迹
plot3(radar_x, radar_y, radar_z, [params.RadarColor '-'], ...
    'LineWidth', params.LineWidth, ...
    'DisplayName', 'Radar Trajectory');

% 绘制目标点（带黑色边框增强可见性）
scatter3(target_x, target_y, target_z, params.MarkerSize, ...
    'o', 'filled', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', params.TargetColor, ...
    'LineWidth', 1.2, ...
    'DisplayName', 'Target Positions');

% 设置坐标轴标签
xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
title(params.Title, 'FontSize', 14, 'FontWeight', 'bold');

% 网格和坐标轴设置
if params.ShowGrid
    grid on;
else
    grid off;
end

if params.EqualAxis
    axis equal;  % 保持各轴比例相同
end
box on;

% 图例和视角
legend('show', 'Location', 'bestoutside');
view(3);
camlight left;
lighting gouraud;

% 自动调整坐标轴范围
all_x = [radar_x; target_x];
all_y = [radar_y; target_y];
all_z = [radar_z; target_z];
margin = 0.05 * [max(all_x) - min(all_x), max(all_y) - min(all_y), max(all_z) - min(all_z)];
xlim([min(all_x) - margin(1), max(all_x) + margin(1)]);
ylim([min(all_y) - margin(2), max(all_y) + margin(2)]);
zlim([min(all_z) - margin(3), max(all_z) + margin(3)]);

% 添加数据统计标签
text(mean(radar_x), min(radar_y)-margin(2)*1.5, min(radar_z), ...
    ['Radar: ' num2str(height(data)) ' positions'], ...
    'FontSize', 10, 'HorizontalAlignment', 'center');
text(mean(target_x), max(target_y)+margin(2)*1.5, max(target_z), ...
    ['Targets: ' num2str(height(data)) ' points'], ...
    'FontSize', 10, 'HorizontalAlignment', 'center');

hold off;
end