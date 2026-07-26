function plot_radar_3d(csv_file, varargin)
% PLOT_RADAR_3D 三维雷达轨迹与目标点可视化
%   plot_radar_3d(csv_file) 从指定CSV文件绘制雷达轨迹（蓝线）和目标点（大红点）
%
%   可选参数（使用'参数名',值对指定）:
%     'LineWidth'    : 雷达轨迹线宽 (默认 2.5)
%     'MarkerSize'   : 目标点大小 (默认 80)
%     'RadarColor'   : 雷达轨迹颜色 (默认 'b')
%     'TargetColor'  : 目标点颜色 (默认 [0.8 0 0])
%     'ShowGrid'     : 是否显示网格 (true/false, 默认 true)
%     'EqualAxis'    : 是否保持坐标轴比例相等 (true/false, 默认 true)
%     'Title'        : 图表标题 (默认 '3D Radar Trajectory and Target Positions')
%     'ShowYaw'      : 是否显示机头朝向箭头 (true/false, 默认 true)
%     'YawInterval'  : 机头箭头间隔点数 (默认自动: floor(N/50))
%     'YawArrowScale': 箭头长度相对于轨迹跨度的比例 (默认 0.02)
%
% 悬停数据提示 (DataTip) - 鼠标悬停自动显示:
%   雷达轨迹点: X,Y,Z + CMD_SPEED_X/Y + RT_TAR_VEL_X/Y + YAW(°)
%   目标点:     X,Y,Z (指令速度/实时目标速度仅雷达轨迹有意义)
%   (对应列为可选，缺失时自动跳过)
%
% 示例:
%   plot_radar_3d('radar_data.csv');
%   plot_radar_3d('data.csv', 'LineWidth', 3, 'MarkerSize', 100, 'Title', '实验1');
%   plot_radar_3d('data.csv', 'ShowYaw', false);
%   plot_radar_3d('data.csv', 'YawInterval', 20, 'YawArrowScale', 0.04);

% 默认参数设置
params = struct(...
    'LineWidth', 2.5, ...
    'MarkerSize', 80, ...
    'RadarColor', 'b', ...
    'TargetColor', [0.8 0 0], ...
    'ShowGrid', true, ...
    'EqualAxis', true, ...
    'Title', '3D Radar Trajectory and Target Positions', ...
    'ShowYaw', true, ...
    'YawInterval', [], ...
    'YawArrowScale', 0.02);

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

%% ==== 可选列读取 (向后兼容) ====
vars = data.Properties.VariableNames;
hasCmdSpd = false;  hasRtTar = false;  hasQuat = false;
cmd_spd_x = [];  cmd_spd_y = [];
rt_tar_x = [];   rt_tar_y = [];
yaw_rad = [];    yaw_deg = [];

% CMD_SPEED
if ismember('CMD_SPEED_X', vars) && ismember('CMD_SPEED_Y', vars)
    cmd_spd_x = data.CMD_SPEED_X;
    cmd_spd_y = data.CMD_SPEED_Y;
    hasCmdSpd = true;
end

% RT_TAR_VEL (实时目标速度)
if ismember('rt_tar_vel_x', vars) && ismember('rt_tar_vel_y', vars)
    rt_tar_x = data.rt_tar_vel_x;
    rt_tar_y = data.rt_tar_vel_y;
    hasRtTar = true;
end

% 四元数 -> YAW
if ismember('qw', vars) && ismember('qx', vars) && ...
   ismember('qy', vars) && ismember('qz', vars)
    qw = data.qw;  qx = data.qx;  qy = data.qy;  qz = data.qz;
    % 从四元数计算 YAW (绕 Z 轴欧拉角)
    yaw_rad = atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));
    yaw_deg = rad2deg(yaw_rad);
    hasQuat = true;
else
    params.ShowYaw = false;
end

%% ==== 创建三维图形 ====
figure('Position', [100, 100, 1200, 800], 'Color', 'white');
hold on;

%% ==== 绘制雷达轨迹 (保留句柄用于配置 DataTip) ====
h_radar = plot3(radar_x, radar_y, radar_z, [params.RadarColor '-'], ...
    'LineWidth', params.LineWidth, ...
    'DisplayName', 'Radar Trajectory');

% 配置雷达轨迹的悬停数据提示
if hasCmdSpd || hasRtTar || hasQuat
    cfgRadarDataTips(h_radar, hasCmdSpd, hasRtTar, hasQuat, ...
        cmd_spd_x, cmd_spd_y, rt_tar_x, rt_tar_y, yaw_deg);
end

%% ==== 绘制目标点 ====
scatter3(target_x, target_y, target_z, params.MarkerSize, ...
    'o', 'filled', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', params.TargetColor, ...
    'LineWidth', 1.2, ...
    'DisplayName', 'Target Positions');

%% ==== 预创建悬停箭头 (初始不可见, 鼠标悬停时动态显示) ====
% 缩放因子
span_xy = max(range(radar_x), range(radar_y));
if span_xy < 0.01, span_xy = 1; end

% CMD 速度箭头 — 蓝色
h_cmd = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0, ...
    'Color', 'b', 'LineWidth', 2.0, 'MaxHeadSize', 0.7, ...
    'DisplayName', 'CMD Speed', 'Visible', 'off');

% RT_TAR 速度箭头 — 红色
h_rt = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0, ...
    'Color', 'r', 'LineWidth', 2.0, 'MaxHeadSize', 0.7, ...
    'DisplayName', 'RT TAR Speed', 'Visible', 'off');

% YAW 机头朝向箭头 — 青绿色
h_yaw = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0, ...
    'Color', [0 0.7 0.5], 'LineWidth', 2.0, 'MaxHeadSize', 0.7, ...
    'DisplayName', 'YAW Heading', 'Visible', 'off');

%% ==== 缓存数据到 figure, 设置鼠标悬停回调 ====
fig = gcf;
ud.radar_x  = radar_x;
ud.radar_y  = radar_y;
ud.radar_z  = radar_z;
ud.cmd_x    = cmd_spd_x;
ud.cmd_y    = cmd_spd_y;
ud.rt_x     = rt_tar_x;
ud.rt_y     = rt_tar_y;
ud.yaw_rad  = yaw_rad;
ud.hasCmdSpd = hasCmdSpd;
ud.hasRtTar  = hasRtTar;
ud.hasQuat   = hasQuat;
ud.span_xy   = span_xy;
ud.h_cmd   = h_cmd;
ud.h_rt    = h_rt;
ud.h_yaw   = h_yaw;
ud.lastIdx = 0;
ud.hoverThreshold = span_xy * 0.03;
fig.UserData = ud;

set(fig, 'WindowButtonMotionFcn', @(src, ~) onRadarHover(src));

%% ==== 坐标轴与显示 ====
xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
title(params.Title, 'FontSize', 14, 'FontWeight', 'bold');

if params.ShowGrid, grid on; else, grid off; end
if params.EqualAxis, axis equal; end
box on;

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

% 数据统计标签
text(mean(radar_x), min(radar_y)-margin(2)*1.5, min(radar_z), ...
    ['Radar: ' num2str(height(data)) ' positions'], ...
    'FontSize', 10, 'HorizontalAlignment', 'center');
text(mean(target_x), max(target_y)+margin(2)*1.5, max(target_z), ...
    ['Targets: ' num2str(height(data)) ' points'], ...
    'FontSize', 10, 'HorizontalAlignment', 'center');

hold off;
end


%% =========================================================================
%  配置雷达轨迹的悬停 DataTip
%  把 CMD_SPEED / RT_TAR_VEL / YAW 绑定到 plot3 线条的所有数据点上
%  鼠标悬停时自动显示这些额外数据
% =========================================================================
function cfgRadarDataTips(h, hasCmdSpd, hasRtTar, hasQuat, ...
    cmd_x, cmd_y, rt_tar_x, rt_tar_y, yaw_deg)
    dt = h.DataTipTemplate;

    if hasCmdSpd
        r1 = dataTipTextRow('CMD_SPD_X', cmd_x);
        r1.Format = '%.2f m/s';
        dt.DataTipRows(end+1) = r1;
        r2 = dataTipTextRow('CMD_SPD_Y', cmd_y);
        r2.Format = '%.2f m/s';
        dt.DataTipRows(end+1) = r2;
    end

    if hasRtTar
        r3 = dataTipTextRow('RT_TAR_X', rt_tar_x);
        r3.Format = '%.2f m/s';
        dt.DataTipRows(end+1) = r3;
        r4 = dataTipTextRow('RT_TAR_Y', rt_tar_y);
        r4.Format = '%.2f m/s';
        dt.DataTipRows(end+1) = r4;
    end

    if hasQuat
        r5 = dataTipTextRow('YAW', yaw_deg);
        r5.Format = '%.1f°';
        dt.DataTipRows(end+1) = r5;
    end

    % 重命名默认 X,Y,Z 行带单位
    dt.DataTipRows(1).Label = 'X';
    dt.DataTipRows(1).Format = '%.3f m';
    dt.DataTipRows(2).Label = 'Y';
    dt.DataTipRows(2).Format = '%.3f m';
    dt.DataTipRows(3).Label = 'Z';
    dt.DataTipRows(3).Format = '%.3f m';
end


%% =========================================================================
%  鼠标悬停回调 — 在最近雷达点绘制速度/朝向箭头
% =========================================================================
function onRadarHover(fig)
    ud = fig.UserData;
    if isempty(ud) || ~isfield(ud, 'radar_x')
        return;
    end
    ax = fig.CurrentAxes;
    if isempty(ax), return; end

    % 获取鼠标在 axes 中的 3D 坐标
    cp = ax.CurrentPoint;       % 2×3: [前表面; 后表面]
    mouse = cp(1, :);           % [x, y, z]

    % 最近邻搜索 (向量化)
    dx = ud.radar_x - mouse(1);
    dy = ud.radar_y - mouse(2);
    dz = ud.radar_z - mouse(3);
    [minDist, idx] = min(dx.^2 + dy.^2 + dz.^2);
    minDist = sqrt(minDist);

    if minDist < ud.hoverThreshold
        % 鼠标靠近某个数据点
        if idx ~= ud.lastIdx
            ud.lastIdx = idx;
            fig.UserData = ud;
            updateArrows(ud, idx);
        end
    else
        % 鼠标远离
        if ud.lastIdx ~= 0
            ud.lastIdx = 0;
            fig.UserData = ud;
            hideArrows(ud);
        end
    end
end

%% -------------------------------------------------------------------------
function updateArrows(ud, idx)
% 更新三组 quiver3 箭头到指定数据点
    scale = ud.span_xy * 0.005;   % 速度 -> 箭头长度 缩放

    % --- CMD 速度箭头 (蓝色) ---
    if ud.hasCmdSpd
        cmd_mag = sqrt(ud.cmd_x(idx)^2 + ud.cmd_y(idx)^2);
        cmd_ang = atan2(ud.cmd_y(idx), ud.cmd_x(idx));
        len = cmd_mag * scale;
        set(ud.h_cmd, ...
            'XData', ud.radar_x(idx), 'YData', ud.radar_y(idx), 'ZData', ud.radar_z(idx), ...
            'UData', len * cos(cmd_ang), 'VData', len * sin(cmd_ang), 'WData', 0, ...
            'Visible', 'on');
    end

    % --- RT_TAR 速度箭头 (红色) ---
    if ud.hasRtTar
        rt_mag = sqrt(ud.rt_x(idx)^2 + ud.rt_y(idx)^2);
        rt_ang = atan2(ud.rt_y(idx), ud.rt_x(idx));
        len = rt_mag * scale;
        set(ud.h_rt, ...
            'XData', ud.radar_x(idx), 'YData', ud.radar_y(idx), 'ZData', ud.radar_z(idx), ...
            'UData', len * cos(rt_ang), 'VData', len * sin(rt_ang), 'WData', 0, ...
            'Visible', 'on');
    end

    % --- YAW 机头朝向箭头 (青绿) ---
    if ud.hasQuat
        fixedLen = ud.span_xy * 0.02;
        ang = ud.yaw_rad(idx);
        set(ud.h_yaw, ...
            'XData', ud.radar_x(idx), 'YData', ud.radar_y(idx), 'ZData', ud.radar_z(idx), ...
            'UData', fixedLen * cos(ang), 'VData', fixedLen * sin(ang), 'WData', 0, ...
            'Visible', 'on');
    end
end

%% -------------------------------------------------------------------------
function hideArrows(ud)
% 隐藏所有悬停箭头
    set(ud.h_cmd,  'Visible', 'off');
    set(ud.h_rt,   'Visible', 'off');
    set(ud.h_yaw,  'Visible', 'off');
end