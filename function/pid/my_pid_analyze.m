function result = my_pid_analyze(csv_path,pid_params,tuning_mode)

    addpath('function\save\');
    if nargin < 1 || isempty(csv_path)
        error('csv_path 不能为空');
    end

    if nargin < 2 || isempty(pid_params)
        pid_params = struct();    %如果pid_params 是 空的话就将这玩意置0
    end

    if nargin < 3 || isempty(tuning_mode)
        tuning_mode = 'balanced';   %
    end

    validateattributes(tuning_mode,'char',{'nonempty'});  %matlab 的自带参数校验函数，必须是字符串类型，不能是空字符串

    if ~ismember(tuning_mode,{'balanced','aggressive','conservative'})
        error('tuning_mode 错误 必须是 balanced/aggressive/conservative')
    end

    %% 合并用户pid参数
    pid = merge_pid_params(pid_params);


    %% 加载CSV数据
    fprintf('\n[1/6] 读取数据:%s\n',csv_path);
    data_raw = load_csv_data(csv_path);
    radar_pos = data_raw.radar_pos;
    target_pos = data_raw.target_pos;
    cmd_vel    = data_raw.cmd_vel;
    row_raw_num = size(radar_pos,1);


    %% 准备结果结构
    result = struct();
    result.recommended = struct();
    result.identified_plant = struct();
    result.metrics = struct();
    result.model_fit.NRMSE = struct();
    result.pid_validation = struct();
    result.steady_state = struct();

    %% PART A PID 仿真对比

    
    fprintf('\n');
    fprintf('\nPID 仿真对比 (MATLAB 模拟 vs 实际 cmd_vel)...\n');
    fprintf('\n');

    dt_array = data_raw.dt_array;
    fprintf('实际平均采样率：%.1f HZ\n',1/data_raw.dt_median);

    rot_state = [];
    state_z = [];  %Z轴独立PID状态
    cmd_vel_sim = zeros(row_raw_num,3); %仿真输出XYZ速度(雷达系)
    cmd_vel_body_sim = zeros(row_raw_num,3); %仿真输出 经四元数旋转后的机体系速度
    length = zeros(row_raw_num,2);
    cmd_raw_xy_nums = zeros(row_raw_num,2);


    has_quat = isfield(data_raw, 'quat') && ~isempty(data_raw.quat);
    if has_quat
        quat = data_raw.quat;
    end

    for k = 1:row_raw_num
        setpoint = target_pos(k,:);
        measurement = radar_pos(k,:);

        if any(isnan(setpoint)) || any(isnan(measurement))
            if k > 1
                cmd_vel_sim(k,:) = cmd_vel_sim(k - 1,:);
                cmd_vel_body_sim(k,:) = cmd_vel_body_sim(k - 1,:);
            end
            continue;
        end

        dt_k = dt_array(k) * 1000; % C代码 xTaskGetTickCount() 单位是 tick(1tick=1ms)，需将秒转换为tick单位

        [cmd_raw_xy , length_xy,cmd_xy,rot_state] = pos_cmd_st(...
            pid.X,pid.Y,...
            measurement,setpoint,...
            rot_state,...
            dt_k ...
            );
        [cmd_z,state_z] = pid_simulate(pid.Z,setpoint(3),measurement(3),state_z,dt_k);

        cmd_vel_sim(k,1:2) = cmd_xy;
        cmd_vel_sim(k,3) = cmd_z;

        length(k,1:2) = length_xy;
        cmd_raw_xy_nums(k,1:2) = cmd_raw_xy;
        % === 四元数旋转: 雷达系速度 → 机体系速度 (复刻 s_cmd_vel_consume_v) ===
        if has_quat
            q = quat(k, :);  % [qw, qx, qy, qz]
            [vx_b, vy_b] = quat_rot_vel_xy(cmd_xy(1), cmd_xy(2), q);
            cmd_vel_body_sim(k, 1) = vx_b;
            cmd_vel_body_sim(k, 2) = vy_b;
        end
        cmd_vel_body_sim(k, 3) = cmd_z;  % Z 轴不做旋转
    end
       result.sim_cmd_vel.XY_radar = cmd_vel_sim(:,1:2);
       result.sim_cmd_vel.Z = cmd_vel_sim(:,3);
       result.sim_cmd_vel.full = cmd_vel_sim;
       if has_quat
           result.sim_cmd_vel.body_XY = cmd_vel_body_sim(:,1:2);
       end

       %% ========== 绘图 ==========
       t = data_raw.t;
       raw_vel = cmd_vel;
       sim_vel = cmd_vel_sim;

       % --- 图1: 雷达系速度对比 (cmd_vel vs sim) ---
       figure('Name','雷达系 三轴速度对比');
       ax_names = {'X','Y','Z'};
       for ax = 1:3
           subplot(3,1,ax);
           plot(t,raw_vel(:,ax),'b-',t,sim_vel(:,ax),'r-','LineWidth',1.1);
           grid on; legend('实际 cmd_vel','仿真', 'Location','best');
           title(sprintf('%s轴 雷达系速度', ax_names{ax})); ylabel('速度');
       end

       % --- 图2: 机体系 XY 速度对比 (rt_tar_vel vs sim_body) ---
       if has_quat && isfield(data_raw, 'rt_tar_vel') && ~isempty(data_raw.rt_tar_vel)
           rt_tar = data_raw.rt_tar_vel;
           figure('Name','机体系 XY 速度对比 (rt_tar_vel vs 仿真)');

           subplot(2,1,1);
           plot(t, rt_tar(:,1),'b-', t, cmd_vel_body_sim(:,1),'r--', ...
                t, cmd_vel_sim(:,1),'g:', 'LineWidth', 1.1);
           grid on;
           legend('实际 rt_tar_vel_x (机体系)', '仿真+旋转 (机体系)', '仿真 (雷达系)', 'Location','best');
           title('X轴: 雷达系 → 机体系 旋转对比'); ylabel('速度');

           subplot(2,1,2);
           plot(t, rt_tar(:,2),'b-', t, cmd_vel_body_sim(:,2),'r--', ...
                t, cmd_vel_sim(:,2),'g:', 'LineWidth', 1.1);
           grid on;
           legend('实际 rt_tar_vel_y (机体系)', '仿真+旋转 (机体系)', '仿真 (雷达系)', 'Location','best');
           title('Y轴: 雷达系 → 机体系 旋转对比'); ylabel('速度'); xlabel('时间');

           % 计算机体系仿真 vs 实际 rt_tar_vel 的 NRMSE
           for ax = 1:2
               ax_n = {'X','Y'};
               err = cmd_vel_body_sim(:,ax) - rt_tar(:,ax);
               rmse_val = sqrt(mean(err.^2));
               rng_val = max(rt_tar(:,ax)) - min(rt_tar(:,ax));
               if rng_val > 1e-6
                   nrmse = rmse_val / rng_val * 100;
               else
                   nrmse = NaN;
               end
               fprintf('  机体系 %s: RMSE=%.3f, NRMSE=%.1f%%\n', ax_n{ax}, rmse_val, nrmse);
           end
       end

       % --- 图3: 四元数导出的 Yaw 角时序 ---
       if has_quat
           % 复刻 fc_ctrl.c 的 yaw 计算公式 (四元数顺序: qx,qy,qz,qw)
           % yaw = -atan2(2*qX*qY + 2*qZ*qW, -2*qY^2 - 2*qZ^2 + 1) * 57.2957795
           qx_col = quat(:,1);
           qy_col = quat(:,2);
           qz_col = quat(:,3);
           qw_col = quat(:,4);
           yaw_deg = -atan2(2*qx_col.*qy_col + 2*qz_col.*qw_col, ...
                            -2*qy_col.^2 - 2*qz_col.^2 + 1) * 57.2957795;
           figure('Name','雷达四元数 Yaw 角');
           plot(t, yaw_deg, 'b-', 'LineWidth', 1.0);
           grid on;
           title('由四元数导出的 Yaw 角 (复刻 fc_ctrl.c 公式)');
           xlabel('时间'); ylabel('Yaw (deg)');

           result.yaw_deg = yaw_deg;
       end
 

       mat = [length,cmd_raw_xy_nums];
       col_names = {'length_x','length_y','cmd_raw_x','cmd_raw_y'};
       creat_file(mat,col_names,csv_path);
        

end


    %% 覆盖pid参数
function defaults = get_default_pid_params()
    % X 轴（与 Y 相同）
    xy.Kp_base = 0.18;
    xy.Ki_base = 0.2;
    xy.Kd_base = 0.5;
    xy.output_max = 23;
    xy.output_min = -23;
    xy.integral_max = 5;
    xy.I_Band = 25;
    xy.d_filter_alpha = 0.8;
    xy.error_threshold_high = 25;
    xy.error_threshold_low = 25;
    xy.Kp_high_ratio = 2.0;
    xy.Ki_high_ratio = 0.7;
    xy.Kd_high_ratio = 0.58;
    xy.kv = 1.0;
    xy.ka = 2.27;

    % Z 轴
    z = xy;
    z.Kp_base = 0.23;
    z.Ki_base = 0.03;
    z.Kd_base = 0.08;
    z.output_max = 20;
    z.output_min = -20;
    z.I_Band = 20;
    z.error_threshold_high = 10;
    z.error_threshold_low = 2;
    z.Kd_high_ratio = 0.7;
    z.ka = 2.22;

    defaults.X = xy;
    defaults.Y = xy;
    defaults.Z = z;
end



%%
function merged = merge_pid_params(pid_params_user)
    defaults = get_default_pid_params();
    merged = defaults;
    if isempty(pid_params_user)             %空 则 直接返回
        return;
    end
    axis_names = {'X','Y','Z'};             %轴的名字

    for i = 1 : 3
        ax = axis_names{i};
        if isfield(pid_params_user,ax)
            ax_user = pid_params_user.(ax);
            flds = fieldnames(ax_user);      % 提取用户当前轴所有修改过的参数字段名，例如 {'Kp','Ki','limit'}
            for k = 1:length(flds)
                f = flds{k};

                old_val = merged.(ax).(f);
                new_val = ax_user.(f);
                fprintf('[%s 轴]参数 %s : 默认值 = %g -> 用户传入 = %g\n',ax,f,old_val,new_val);
                if strcmp(f,'plant_init')
                    merged.(ax).plant_init = ax_user.(f);
                else
                    merged.(ax).(f) = ax_user.(f);
                end
            end
        end
    end
end

%%
    
%% 数据加载函数，获取数据，以及dt
function data_out = load_csv_data(csv_path)
    csv_path = char(csv_path); %确保char

    
    fid = fopen(csv_path,'r');

    if fid == -1
        error('无法打开文件: %s',csv_path);
    end


    % s = '   TICK , X , Y , Z  ';
    % s2 = strtrim(s);
    % s2 结果：'TICK , X , Y , Z'
    
    % first_line = fgetl(fid);
    % fclose(fid);
    % first_line = strtrim(first_line);

    opts = detectImportOptions(csv_path);       %自动分析 CSV 文件格式：分隔符、数值类型、行列规则，生成导入配置结构体 opts。
    opts.DataLines = [2,Inf];                   %跳过第 1 行的列名  %[2, Inf]：跳过第 1 行，从第 2 行读到文件最后一行
    data_raw = readmatrix(csv_path,opts);       %获取原始数据
    data_raw(all(isnan(data_raw),2),:) = [];    

    %列名找索引
    col_names = opts.VariableNames;
    radar_idx = find(contains(col_names,"RADAR_POS"));
    target_idx = find(contains(col_names,"TARGET_POS"));
    cmd_idx    = find(contains(col_names,"CMD_SPEED"));
    trel_idx =  find(contains(col_names,'T_REL'));    %需要找到T_REL 索引

    if length(radar_idx) < 3 || length(target_idx) < 3 || length(cmd_idx) < 3
        error('CSV 缺扫必要列 (RADAR_POS)/ TARGET_POS / CMD_SPEED');
    end

    data_out.radar_pos = data_raw(:,radar_idx(1:3));
    data_out.target_pos = data_raw(:,target_idx(1:3));
    data_out.cmd_vel   = data_raw(:,cmd_idx(1:3));

    % 加载四元数 (qx,qy,qz,qw) — 注意列顺序!
    qx_idx = find(contains(col_names, 'qx'));
    qy_idx = find(contains(col_names, 'qy'));
    qz_idx = find(contains(col_names, 'qz'));
    qw_idx = find(contains(col_names, 'qw'));
    if ~isempty(qw_idx) && ~isempty(qx_idx) && ...
       ~isempty(qy_idx) && ~isempty(qz_idx)
        data_out.quat = data_raw(:, [qx_idx(1), qy_idx(1), qz_idx(1), qw_idx(1)]);
        fprintf('  [四元数] 已加载 (顺序: qx,qy,qz,qw)\n');
    else
        data_out.quat = [];
        fprintf('  [四元数] 未找到 qx/qy/qz/qw 列，跳过\n');
    end

    % 加载机体系实时目标速度 rt_tar_vel_x/y — 可选
    rtx_idx = find(contains(col_names, 'rt_tar_vel_x'));
    rty_idx = find(contains(col_names, 'rt_tar_vel_y'));
    if ~isempty(rtx_idx) && ~isempty(rty_idx)
        data_out.rt_tar_vel = data_raw(:, [rtx_idx(1), rty_idx(1)]);
        fprintf('  [rt_tar_vel] 已加载 (机体系速度)\n');
    else
        data_out.rt_tar_vel = [];
        fprintf('  [rt_tar_vel] 未找到，跳过\n');
    end

    % 从 T_REL 计算 实际 dt

    if ~isempty(trel_idx)
        t = data_raw(:,trel_idx(1));
        data_out.t = t;

        %计算逐点时间间隔 dt(k) = t(k) - t(k - 1)
        dt_array = diff(t);

        %过滤异常值 ： 负间隔\ 接近0 的跳变值

        abnormal_idx = dt_array <= 0.001 | dt_array > 0.5; %大于0.5s 视为丢帧异常

        dt_array(abnormal_idx) = median(dt_array(~abnormal_idx)); %异常的点使用中位数进行填充

        %补齐到与数据等长：第1帧没有前一帧，用第1个dt填充
        dt_array = [dt_array(1); dt_array];

        %4. 输出： 逐点dt 数组 + 参考中位数dt
        data_out.dt_array = dt_array; %每一行对应一个真实的dt,PID 仿真使用这个
        data_out.dt_median = median(dt_array); %参考平均采样周期

    else
        %没有时间列的兜底
        data_out.t = (0:size(data_raw,1)-1)' * 0.1;
        data_out.dt_array = ones(size(data_raw,1),1)* 0.1;
        data_out.dt_median = 0.1;
    end
    
end



function [out,state] = pid_simulate(pid_ax,setpoint,measurement,state,dt)
    if isempty(state)
        state.integral = 0;
        state.prev_measurement = 0;           % 对应 C: prev_measurement_f = 0.0f
        state.prev_d_filtered = 0;            % 对应 C: prev_d_filtered_f = 0.0f
        state.pre_target_vel = 0;             % 对应 C: pre_target_vel_f = 0
        state.pre_target_position = 0;        % 对应 C: pre_target_position_f = 0（不是 measurement）
    end

    error = setpoint - measurement;
    error_abs = abs(error);

    if error_abs > pid_ax.error_threshold_high
        Kp = pid_ax.Kp_base * pid_ax.Kp_high_ratio;
        Ki = pid_ax.Ki_base * pid_ax.Ki_high_ratio;
        Kd = pid_ax.Kd_base * pid_ax.Kd_high_ratio;
    elseif error_abs > pid_ax.error_threshold_low
        ratio = (error_abs - pid_ax.error_threshold_low) / ...
                (pid_ax.error_threshold_high - pid_ax.error_threshold_low);
        Kp = pid_ax.Kp_base * (1 + ratio * (pid_ax.Kp_high_ratio - 1));
        Ki = pid_ax.Ki_base * (1 - ratio * (1 - pid_ax.Ki_high_ratio));
        Kd = pid_ax.Kd_base * (1 - ratio * (1 - pid_ax.Kd_high_ratio));
    else
        Kp = pid_ax.Kp_base;
        Ki = pid_ax.Ki_base;
        Kd = pid_ax.Kd_base;
    end

    % 积分分离 + 限幅
    if error_abs <= pid_ax.I_Band
        state.integral = state.integral + error * dt;
        state.integral = max(min(state.integral, pid_ax.integral_max), -pid_ax.integral_max);
    else
        state.integral = 0;
    end

    d_raw = (measurement - state.prev_measurement) / dt;
    d_filtered = pid_ax.d_filter_alpha * d_raw + ...
                 (1 - pid_ax.d_filter_alpha) * state.prev_d_filtered;

    out = Kp * error + Ki * state.integral - Kd * d_filtered;

    % 修正后的前馈（使用 measurement，而不是 C 代码中 bug 的 measurement[k] - setpoint[k-1]）
    target_vel = (measurement - state.pre_target_position) / dt;
    target_acc = (target_vel - state.pre_target_vel) / dt;
    out = out + pid_ax.kv * target_vel + pid_ax.ka * target_acc;

    % 输出限幅 + 抗饱和
    if out > pid_ax.output_max
        out = pid_ax.output_max;
        state.integral = (out - Kp * error + Kd * d_filtered) / Ki;
    elseif out < pid_ax.output_min
        out = pid_ax.output_min;
        state.integral = (out - Kp * error + Kd * d_filtered) / Ki;
    end

    % 更新状态
    state.prev_measurement = measurement;
    state.prev_d_filtered = d_filtered;
    state.pre_target_position = measurement;
    state.pre_target_vel = target_vel;
end

%% 传入的是单行的数据
function [cmd_raw_xy,length,cmd_vel_out,state_out] = pos_cmd_st(pid_x,pid_y,...
                            radar_pos,target_pos,rot_state,dt)

    radar_pos_xy = radar_pos(1:2);
    target_pos_xy = target_pos(1:2);

    if isempty(rot_state)
        rot_state.start_point = [0,0];
        rot_state.theta_1 = 0;
        rot_state.setpoint_modulus = 0;
        rot_state.last_target = 0;
        rot_state.state_X = [];
        rot_state.state_Y = [];
    end


    if any(rot_state.last_target ~= target_pos_xy )
        rot_state.start_point = rot_state.last_target;
        delta_xy = target_pos_xy - rot_state.start_point;

        if norm(delta_xy) < 1e-6
            rot_state.theta_1 = 0;
        else
            rot_state.theta_1 = atan2(delta_xy(2),delta_xy(1));
        end

        rot_state.setpoint_modulus = norm(delta_xy);
        rot_state.last_target   = target_pos_xy;

        % 复刻 C 代码 pid_reset_v: 只重置 integral 和 prev_measurement
        % 保留 prev_d_filtered、pre_target_position、pre_target_vel
        if ~isempty(rot_state.state_X)
            rot_state.state_X.integral = 0;
            rot_state.state_X.prev_measurement = 0;
        else
            rot_state.state_X = [];
        end
        if ~isempty(rot_state.state_Y)
            rot_state.state_Y.integral = 0;
            rot_state.state_Y.prev_measurement = 0;
        else
            rot_state.state_Y = [];
        end
    end

    un_trans_pos = radar_pos_xy - rot_state.start_point;
    pos_modulus = norm(un_trans_pos);

    if pos_modulus < 1e-6
        theta_2 = 0;
    else 
        theta_2 = atan2(un_trans_pos(2),un_trans_pos(1));
    end

    length_x = pos_modulus* cos(rot_state.theta_1 - theta_2);
    length_y = pos_modulus* sin(rot_state.theta_1 - theta_2);

    

    state_X = rot_state.state_X;
    state_Y = rot_state.state_Y;

    [cmd_x,state_X] = pid_simulate(pid_x,rot_state.setpoint_modulus,length_x,state_X,dt);
    [cmd_y,state_Y] = pid_simulate(pid_y,0,length_y,state_Y,dt);

    length = [rot_state.setpoint_modulus - length_x,-length_y];
    cmd_raw_xy = [cmd_x,cmd_y];
    
    cmd_vel_out = zeros(1,2);
    cmd_vel_out(1) = cmd_x * cos(rot_state.theta_1) + cmd_y * sin(rot_state.theta_1);
    cmd_vel_out(2) = cmd_x * sin(rot_state.theta_1) - cmd_y * cos(rot_state.theta_1);

    rot_state.state_X = state_X;
    rot_state.state_Y = state_Y;

    state_out = rot_state;
end


%% ========== 四元数旋转: 雷达系 → 机体系 ==========
function [vx_body, vy_body] = quat_rot_vel_xy(vx_radar, vy_radar, q)
% QUAT_ROT_VEL_XY 复刻 s_cmd_vel_consume_v 中的四元数旋转
%
% 输入:
%   vx_radar, vy_radar — 雷达系速度指令 (来自 PID)
%   q — 四元数行向量 [qx, qy, qz, qw] (注意顺序!)
%
% 输出:
%   vx_body, vy_body — 旋转后的机体系速度
%
% 原理:
%   C 代码中 qX_f=-qX, qY_f=-qY, qZ_f=-qZ (取负 = 四元数共轭),
%   等价于从雷达系到机体系的逆变换.

    % 提取四元数分量 (顺序: qx, qy, qz, qw)
    qx_raw = q(1);
    qy_raw = q(2);
    qz_raw = q(3);
    qw = q(4);

    % C 代码取共轭: qX_f=-qX, qY_f=-qY, qZ_f=-qZ
    qx = -qx_raw;
    qy = -qy_raw;
    qz = -qz_raw;
    % qw 保持不变

    % 旋转矩阵 XY 行
    r11 = 1 - 2*qy*qy - 2*qz*qz;
    r12 = 2*qx*qy - 2*qw*qz;
    r21 = 2*qx*qy + 2*qw*qz;
    r22 = 1 - 2*qx*qx - 2*qz*qz;

    vx_body = r11 * vx_radar + r12 * vy_radar;
    vy_body = r21 * vx_radar + r22 * vy_radar;
end

