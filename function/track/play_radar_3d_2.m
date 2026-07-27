function play_radar_3d_2(csv_file, varargin)
% PLAY_RADAR_3D 三维雷达轨迹动画播放器 (方案B: animatedline + hgtransform)
%
%   核心改进:
%     1. animatedline: 高性能轨迹绘制，支持滑动窗口拖尾效果
%     2. hgtransform: 官方推荐的3D对象变换，无人机模型+箭头只需修改变换矩阵
%     3. patch: 自定义3D箭头锥体，大小基于数据坐标，自适应缩放
%     4. 完整播放控制: 播放/暂停/进度条/速度调节/单步前进后退
%     5. 四元数姿态: 直接驱动无人机3D模型，零误差
%
%   用法:
%     play_radar_3d('radar_data.csv');
%     play_radar_3d('data.csv', 'AutoPlay', true, 'TrailLength', 500);

    %% ===== 默认参数 =====
    params = struct(...
        'AutoPlay', false, ...          % 是否自动开始播放
        'TrailLength', 0, ...           % 轨迹拖尾长度(0=全部显示)
        'DroneScale', 1.0, ...          % 无人机模型缩放倍数
        'ArrowScaleFactor', 1.5, ...    % 箭头整体放大倍数
        'ArrowHeadFraction', 0.3, ...   % 箭头头部占箭身比例
        'PlaySpeed', 1.0, ...           % 播放速度倍率
        'FPS', 30);                     % 动画帧率

    % 解析可选参数
    pnames = fieldnames(params);
    for i = 1:2:length(varargin)
        if i+1 <= length(varargin)
            pname = varargin{i};
            pval = varargin{i+1};
            if ismember(pname, pnames)
                params.(pname) = pval;
            end
        end
    end

    %% ===== 读取CSV数据 =====
    if ~isfile(csv_file)
        error('文件不存在: %s', csv_file);
    end
    data = readtable(csv_file);

    % 验证必需列
    required = {'RADAR_POS_X','RADAR_POS_Y','RADAR_POS_Z',...
                'TARGET_POS_X','TARGET_POS_Y','TARGET_POS_Z'};
    for i = 1:length(required)
        if ~ismember(required{i}, data.Properties.VariableNames)
            error('CSV缺少必需列: %s', required{i});
        end
    end

    % 提取数据
    rx = data.RADAR_POS_X;  ry = data.RADAR_POS_Y;  rz = data.RADAR_POS_Z;
    tx = data.TARGET_POS_X; ty = data.TARGET_POS_Y; tz = data.TARGET_POS_Z;
    N = length(rx);

    % 可选列
    vars = data.Properties.VariableNames;
    hasCmd = ismember('CMD_SPEED_X', vars) && ismember('CMD_SPEED_Y', vars);
    hasRt  = ismember('rt_tar_vel_x', vars) && ismember('rt_tar_vel_y', vars);
    hasFc  = ismember('fc_sen_vel_x', vars) && ismember('fc_sen_vel_y', vars);
    hasQuat = ismember('qw', vars) && ismember('qx', vars) && ...
              ismember('qy', vars) && ismember('qz', vars);

    if hasCmd, cmd_x = data.CMD_SPEED_X; cmd_y = data.CMD_SPEED_Y; end
    if hasRt,  rt_x = data.rt_tar_vel_x;  rt_y = data.rt_tar_vel_y; end
    if hasFc,  fc_x = data.fc_sen_vel_x;  fc_y = data.fc_sen_vel_y; end
    if hasQuat
        qw = data.qw; qx = data.qx; qy = data.qy; qz = data.qz;
        yaw_rad = atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));
    else
        yaw_rad = zeros(N, 1);
    end

    % 计算缩放基准
    span = max([range(rx), range(ry), range(rz)]);
    if span < 0.01, span = 1; end
    arrowBaseLen = span * 0.04 * params.ArrowScaleFactor;
    droneSize = span * 0.015 * params.DroneScale;

    %% ===== 创建图形界面 =====
    fig = figure('Position', [100 100 1400 900], ...
        'Color', [0.05 0.05 0.08], ...
        'Name', '3D Radar Trajectory Player', ...
        'NumberTitle', 'off');
    
    % 主绘图区
    ax = axes('Parent', fig, 'Position', [0.05 0.15 0.9 0.8]);
    hold(ax, 'on');
    axis(ax, 'equal'); grid(ax, 'on');
    set(ax, 'Color', [0.08 0.08 0.12], ...
        'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], 'ZColor', [0.7 0.7 0.7]);
    xlabel(ax, 'X (m)', 'Color', 'w', 'FontWeight', 'bold');
    ylabel(ax, 'Y (m)', 'Color', 'w', 'FontWeight', 'bold');
    zlabel(ax, 'Z (m)', 'Color', 'w', 'FontWeight', 'bold');
    title(ax, '3D Radar Trajectory Animation', 'Color', 'w', 'FontSize', 14);
    view(ax, 3);
    camlight(ax, 'left'); lighting(ax, 'gouraud');

    % 设置坐标轴范围
    all_x = [rx; tx]; all_y = [ry; ty]; all_z = [rz; tz];
    margin = 0.1 * [max(all_x)-min(all_x), max(all_y)-min(all_y), max(all_z)-min(all_z)];
    xlim(ax, [min(all_x)-margin(1), max(all_x)+margin(1)]);
    ylim(ax, [min(all_y)-margin(2), max(all_y)+margin(2)]);
    zlim(ax, [min(all_z)-margin(3), max(all_z)+margin(3)]);

    %% ===== 1. 轨迹线 (animatedline) =====
    maxPts = N;
    if params.TrailLength > 0
        maxPts = params.TrailLength;
    end
    trail = animatedline(ax, 'Color', [0.3 0.5 1], 'LineWidth', 2.5, ...
        'MaximumNumPoints', maxPts);

    %% ===== 2. 目标点 (scatter3) =====
    scatter3(ax, tx, ty, tz, 80, 'o', 'filled', ...
        'MarkerFaceColor', [0.8 0 0], 'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Target');

    %% ===== 3. 无人机模型 (hgtransform) =====
    t_uav = hgtransform('Parent', ax);
    buildDroneModel(t_uav, droneSize);

    %% ===== 4. 速度箭头组 (hgtransform) =====
    t_vel = hgtransform('Parent', ax);
    
    % CMD箭头 - 蓝色
    h_cmd = buildArrow3D(t_vel, [0.2 0.4 1], 'CMD');
    % RT箭头 - 红色
    h_rt = buildArrow3D(t_vel, [1 0.2 0.2], 'RT_TAR');
    % FC箭头 - 品红
    h_fc = buildArrow3D(t_vel, [1 0 0.6], 'FC_SEN');
    % YAW箭头 - 青绿
    h_yaw = buildArrow3D(t_vel, [0 0.8 0.5], 'YAW');

    %% ===== 5. 速度标注文本 =====
    txt_info = text(ax, NaN, NaN, NaN, '', ...
        'Interpreter', 'tex', 'FontSize', 11, 'FontWeight', 'bold', ...
        'BackgroundColor', [0 0 0 0.8], 'Color', 'w', ...
        'EdgeColor', [0.5 0.5 0.5], 'Margin', 4, 'Visible', 'off');

    %% ===== 6. UI 控制面板 =====
    uipanel(fig, 'Position', [0 0 1 0.12], 'BackgroundColor', [0.15 0.15 0.2]);
    
    % 播放/暂停按钮
    btn_play = uicontrol(fig, 'Style', 'pushbutton', 'String', '▶ Play', ...
        'Position', [20 30 80 40], 'FontSize', 12, ...
        'BackgroundColor', [0.2 0.6 0.3], 'ForegroundColor', 'w');
    
    % 单步后退
    btn_back = uicontrol(fig, 'Style', 'pushbutton', 'String', '◀◀', ...
        'Position', [110 30 50 40], 'FontSize', 12);
    
    % 单步前进
    btn_fwd = uicontrol(fig, 'Style', 'pushbutton', 'String', '▶▶', ...
        'Position', [170 30 50 40], 'FontSize', 12);
    
    % 进度滑动条
    slider = uicontrol(fig, 'Style', 'slider', ...
        'Position', [240 40 800 20], ...
        'Min', 1, 'Max', N, 'Value', 1, ...
        'SliderStep', [1/N, 10/N]);
    
    % 当前帧标签
    lbl_frame = uicontrol(fig, 'Style', 'text', ...
        'Position', [1060 35 120 30], 'FontSize', 11, ...
        'BackgroundColor', [0.15 0.15 0.2], 'ForegroundColor', 'w', ...
        'String', sprintf('Frame: 1/%d', N));
    
    % 速度下拉框
    uicontrol(fig, 'Style', 'text', 'String', 'Speed:', ...
        'Position', [1200 45 50 20], 'BackgroundColor', [0.15 0.15 0.2], ...
        'ForegroundColor', 'w');
    popup_speed = uicontrol(fig, 'Style', 'popupmenu', ...
        'String', {'0.5x', '1x', '2x', '5x', '10x'}, ...
        'Position', [1250 35 60 30], 'Value', 2);
    speed_map = [0.5, 1, 2, 5, 10];

    %% ===== 7. 状态管理 (存入 fig.UserData, 所有回调共享) =====
    ud.isPlaying = false;
    ud.currentIdx = 1;
    ud.timer = [];
    ud.playSpeed = params.PlaySpeed;

    ud.rx = rx; ud.ry = ry; ud.rz = rz;
    ud.yaw_rad = yaw_rad;
    ud.hasCmd = hasCmd; ud.hasRt = hasRt; ud.hasFc = hasFc; ud.hasQuat = hasQuat;
    if hasCmd, ud.cmd_x = cmd_x; ud.cmd_y = cmd_y; end
    if hasRt, ud.rt_x = rt_x; ud.rt_y = rt_y; end
    if hasFc, ud.fc_x = fc_x; ud.fc_y = fc_y; end
    if hasQuat, ud.qw = qw; ud.qx = qx; ud.qy = qy; ud.qz = qz; end
    ud.arrowBaseLen = arrowBaseLen;
    ud.headFrac = params.ArrowHeadFraction;
    ud.h_cmd = h_cmd; ud.h_rt = h_rt; ud.h_fc = h_fc; ud.h_yaw = h_yaw;
    ud.txt_info = txt_info;
    ud.span = span;
    ud.trail = trail;
    ud.t_uav = t_uav;
    ud.t_vel = t_vel;
    ud.lbl_frame = lbl_frame;
    ud.N = N;
    ud.fps = params.FPS;
    fig.UserData = ud;

    %% ===== 8. 回调函数绑定 =====
    set(btn_play, 'Callback', @(src,~) onPlayPause(src, fig));
    set(btn_back, 'Callback', @(src,~) onStep(-1, slider, fig));
    set(btn_fwd,  'Callback', @(src,~) onStep(1, slider, fig));
    set(slider, 'Callback', @(src,~) onSliderChange(src, fig));
    set(popup_speed, 'Callback', @(src,~) onSpeedChange(src, fig, speed_map));

    %% ===== 9. 初始化显示第1帧 =====
    updateFrame(fig, 1);

    % 如果自动播放
    if params.AutoPlay
        onPlayPause(btn_play, fig);
    end
end

%% =========================================================================
%  构建无人机3D模型 (在 hgtransform 内部，局部坐标系)
%  机头朝向 +X，颜色：黄色机身 + RGB三轴
% =========================================================================
function buildDroneModel(parent_t, size)
    % 机身：扁平方块 (长2size, 宽1.2size, 高0.3size)
    L = size * 2;
    W = size * 1.2;
    H = size * 0.3;
    
    % 8个顶点
    v = [
        -L/2, -W/2, -H/2;   % 1
         L/2, -W/2, -H/2;   % 2
         L/2,  W/2, -H/2;   % 3
        -L/2,  W/2, -H/2;   % 4
        -L/2, -W/2,  H/2;   % 5
         L/2, -W/2,  H/2;   % 6
         L/2,  W/2,  H/2;   % 7
        -L/2,  W/2,  H/2;   % 8
    ];
    % 6个面
    f = [
        1 2 3 4;  % bottom
        5 6 7 8;  % top
        1 2 6 5;  % front
        3 4 8 7;  % back
        1 4 8 5;  % left
        2 3 7 6;  % right
    ];
    patch('Parent', parent_t, 'Vertices', v, 'Faces', f, ...
        'FaceColor', [0.9 0.8 0.2], 'FaceAlpha', 0.9, ...
        'EdgeColor', [0.6 0.5 0.1], 'LineWidth', 1);

    % 机头指示：+X方向红色三角
    nose_v = [L/2, 0, 0; L/2+size*0.5, -size*0.3, 0; L/2+size*0.5, size*0.3, 0];
    nose_f = [1 2 3];
    patch('Parent', parent_t, 'Vertices', nose_v, 'Faces', nose_f, ...
        'FaceColor', [1 0.2 0.2], 'EdgeColor', 'r');

    % 机体坐标系 Triad (RGB = XYZ)
    axLen = size * 2.5;
    % X轴 - 红
    line('Parent', parent_t, 'XData', [0, axLen], 'YData', [0,0], 'ZData', [0,0], ...
        'Color', 'r', 'LineWidth', 2.5);
    % Y轴 - 绿
    line('Parent', parent_t, 'XData', [0,0], 'YData', [0, axLen], 'ZData', [0,0], ...
        'Color', 'g', 'LineWidth', 2.5);
    % Z轴 - 蓝
    line('Parent', parent_t, 'XData', [0,0], 'YData', [0,0], 'ZData', [0, axLen], ...
        'Color', 'b', 'LineWidth', 2.5);
end

%% =========================================================================
%  构建3D箭头 (line箭身 + patch锥体头部)
%  返回结构体包含 shaft 和 head 句柄
% =========================================================================
function h = buildArrow3D(parent_t, color, name)
    h.shaft = line('Parent', parent_t, ...
        'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
        'Color', color, 'LineWidth', 2.5, ...
        'Visible', 'off', 'DisplayName', name);
    
    % 锥体头部：用 patch 画一个4面锥体
    h.head = patch('Parent', parent_t, ...
        'Vertices', NaN(5,3), 'Faces', NaN(4,3), ...
        'FaceColor', color, 'EdgeColor', color, ...
        'FaceAlpha', 0.9, 'Visible', 'off');
end

%% =========================================================================
%  更新箭头几何形状 (基于数据坐标)
% =========================================================================
function updateArrow3D(h, origin, direction, baseLen, headFrac)
    dir = direction(:);
    mag = norm(dir);
    if mag < 1e-10
        set(h.shaft, 'Visible', 'off');
        set(h.head, 'Visible', 'off');
        return;
    end
    dir = dir / mag;
    
    % 箭头总长度 = baseLen * mag (速度越大箭头越长)
    totalLen = baseLen * mag;
    tip = origin(:) + dir * totalLen;
    
    % 箭身终点 (箭头头部起点)
    headLen = totalLen * headFrac;
    shaftEnd = tip - dir * headLen;
    
    % 更新箭身
    set(h.shaft, ...
        'XData', [origin(1), shaftEnd(1)], ...
        'YData', [origin(2), shaftEnd(2)], ...
        'ZData', [origin(3), shaftEnd(3)], ...
        'Visible', 'on');
    
    % 更新锥体头部
    headRadius = headLen * 0.35;
    
    % 构造正交基
    if abs(dir(3)) < 0.9
        perp1 = cross(dir, [0;0;1]);
    else
        perp1 = cross(dir, [1;0;0]);
    end
    perp1 = perp1 / norm(perp1);
    perp2 = cross(dir, perp1);
    
    % 锥体底面中心
    baseC = tip - dir * headLen;
    
    % 底面4个点
    theta = [0, pi/2, pi, 3*pi/2];
    basePts = zeros(4, 3);
    for k = 1:4
        basePts(k,:) = baseC' + headRadius * (cos(theta(k))*perp1' + sin(theta(k))*perp2');
    end
    
    % 5个顶点 (4个底面点 + 1个尖端)
    verts = [basePts; tip(:)'];
    % 4个三角面
    faces = [1 2 5; 2 3 5; 3 4 5; 4 1 5];
    
    set(h.head, 'Vertices', verts, 'Faces', faces, 'Visible', 'on');
end

%% =========================================================================
%  核心更新函数：更新第 idx 帧的所有图形元素
% =========================================================================
function updateFrame(fig, idx)
    ud = fig.UserData;
    if idx < 1 || idx > ud.N, return; end
    
    % 1. 更新轨迹 (animatedline)
    % 清除并重新添加所有点 (为了支持滑动条跳转)
    clearpoints(ud.trail);
    startIdx = max(1, idx - 500);  % 最多显示最近500个点
    for i = startIdx:idx
        addpoints(ud.trail, ud.rx(i), ud.ry(i), ud.rz(i));
    end
    
    % 2. 更新无人机位姿 (hgtransform)
    origin = [ud.rx(idx), ud.ry(idx), ud.rz(idx)];
    
    if ud.hasQuat
        % 四元数转旋转矩阵
        q = [ud.qw(idx), ud.qx(idx), ud.qy(idx), ud.qz(idx)];
        R = quat2rotm(q);
        T_rot = eye(4);
        T_rot(1:3, 1:3) = R;
        T_pos = makehgtform('translate', origin);
        set(ud.t_uav, 'Matrix', T_pos * T_rot);
    else
        % 无四元数时只用平移 + yaw旋转
        T_pos = makehgtform('translate', origin);
        T_yaw = makehgtform('zrotate', ud.yaw_rad(idx));
        set(ud.t_uav, 'Matrix', T_pos * T_yaw);
    end
    
    % 3. 更新速度箭头
    baseLen = ud.arrowBaseLen;
    headFrac = ud.headFrac;
    
    % CMD箭头 (世界系)
    if ud.hasCmd
        vx = ud.cmd_x(idx); vy = ud.cmd_y(idx);
        updateArrow3D(ud.h_cmd, origin, [vx, vy, 0], baseLen, headFrac);
    end
    
    % RT箭头 (机头系 -> 世界系)
    if ud.hasRt && ud.hasQuat
        rt_bx = ud.rt_x(idx); rt_by = ud.rt_y(idx);
        cy = cos(ud.yaw_rad(idx)); sy = sin(ud.yaw_rad(idx));
        rt_wx = rt_bx * cy - rt_by * sy;
        rt_wy = rt_bx * sy + rt_by * cy;
        updateArrow3D(ud.h_rt, origin, [rt_wx, rt_wy, 0], baseLen, headFrac);
    end
    
    % FC箭头 (机头系 -> 世界系)
    if ud.hasFc && ud.hasQuat
        fc_bx = ud.fc_x(idx); fc_by = ud.fc_y(idx);
        cy = cos(ud.yaw_rad(idx)); sy = sin(ud.yaw_rad(idx));
        fc_wx = fc_bx * cy - fc_by * sy;
        fc_wy = fc_bx * sy + fc_by * cy;
        updateArrow3D(ud.h_fc, origin, [fc_wx, fc_wy, 0], baseLen, headFrac);
    end
    
    % YAW箭头 (固定长度)
    if ud.hasQuat
        yawLen = baseLen * 0.6;
        ang = ud.yaw_rad(idx);
        updateArrow3D(ud.h_yaw, origin, [cos(ang), sin(ang), 0], yawLen, headFrac);
    end
    
    % 4. 更新速度标注文本
    parts = {};
    if ud.hasCmd
        cmd_mag = sqrt(ud.cmd_x(idx)^2 + ud.cmd_y(idx)^2);
        cmd_signed = sign(ud.cmd_x(idx)) * cmd_mag;
        parts{end+1} = sprintf('\\color[rgb]{0.2,0.4,1}CMD:%+.2f', cmd_signed);
    end
    if ud.hasRt
        rt_mag = sqrt(ud.rt_x(idx)^2 + ud.rt_y(idx)^2);
        rt_signed = sign(ud.rt_x(idx)) * rt_mag;
        parts{end+1} = sprintf('\\color[rgb]{1,0.2,0.2}RT:%+.2f', rt_signed);
    end
    if ud.hasFc
        fc_mag = sqrt(ud.fc_x(idx)^2 + ud.fc_y(idx)^2);
        fc_signed = (ud.fc_x(idx) >= 0) * fc_mag - (ud.fc_x(idx) < 0) * fc_mag;
        parts{end+1} = sprintf('\\color[rgb]{1,0,0.6}FCS:%+.2f', fc_signed);
    end
    if ud.hasQuat
        parts{end+1} = sprintf('\\color[rgb]{0,0.8,0.5}YAW:%+.1f°', rad2deg(ud.yaw_rad(idx)));
    end
    
    if ~isempty(parts)
        dOff = ud.span * 0.05;
        set(ud.txt_info, ...
            'Position', [origin(1)+dOff, origin(2)+dOff, origin(3)+dOff], ...
            'String', strjoin(parts, '  |  '), ...
            'Visible', 'on');
    end
    
    % 5. 更新UI标签
    set(ud.lbl_frame, 'String', sprintf('Frame: %d/%d', idx, ud.N));
    
    drawnow limitrate;
end

%% =========================================================================
%  UI 回调函数
% =========================================================================
function onPlayPause(btn, fig)
    ud = fig.UserData;
    if ud.isPlaying
        % 暂停
        ud.isPlaying = false;
        fig.UserData = ud;
        set(btn, 'String', '▶ Play', 'BackgroundColor', [0.2 0.6 0.3]);
        if ~isempty(ud.timer) && isvalid(ud.timer)
            stop(ud.timer);
        end
    else
        % 播放
        ud.isPlaying = true;
        set(btn, 'String', '⏸ Pause', 'BackgroundColor', [0.8 0.3 0.2]);
        ud.timer = timer(...
            'ExecutionMode', 'fixedRate', ...
            'Period', 1/ud.fps, ...
            'TimerFcn', @(~,~) timerCallback(fig));
        fig.UserData = ud;
        start(ud.timer);
    end
end

function timerCallback(fig)
    ud = fig.UserData;
    if ~ud.isPlaying || ~isvalid(fig)
        return;
    end

    nextIdx = ud.currentIdx + 1;

    if nextIdx > ud.N
        % 播放结束
        ud.isPlaying = false;
        fig.UserData = ud;
        stop(ud.timer);
        return;
    end

    ud.currentIdx = nextIdx;
    fig.UserData = ud;
    updateFrame(fig, nextIdx);

    % 更新滑动条
    h_slider = findobj(fig, 'Style', 'slider');
    if ~isempty(h_slider)
        set(h_slider, 'Value', nextIdx);
    end
end

function onSliderChange(slider, fig)
    ud = fig.UserData;
    idx = round(get(slider, 'Value'));
    ud.currentIdx = idx;
    fig.UserData = ud;
    updateFrame(fig, idx);
end

function onStep(delta, slider, fig)
    ud = fig.UserData;
    newIdx = ud.currentIdx + delta;
    newIdx = max(1, min(ud.N, newIdx));
    ud.currentIdx = newIdx;
    fig.UserData = ud;
    set(slider, 'Value', newIdx);
    updateFrame(fig, newIdx);
end

function onSpeedChange(popup, fig, speed_map)
    ud = fig.UserData;
    idx = get(popup, 'Value');
    ud.playSpeed = speed_map(idx);
    fig.UserData = ud;
    if ~isempty(ud.timer) && isvalid(ud.timer)
        set(ud.timer, 'Period', 1/(ud.fps * ud.playSpeed));
    end
end