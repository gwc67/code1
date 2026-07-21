function result = my_pid_analyze(csv_path,pid_params,tuning_mode)

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

    pid = merge_pid_params(pid_params);


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

%% 数据加载
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

    opts = detectImportOptions(csv_path);   %自动分析 CSV 文件格式：分隔符、数值类型、行列规则，生成导入配置结构体 opts。
    opts.data_lines = [2,Inf];              %跳过第 1 行的列名  %[2, Inf]：跳过第 1 行，从第 2 行读到文件最后一行
    data_raw = readmatrix(csv_path,opts);
    data_raw(all(isnan(data_raw),2),:) = [];
