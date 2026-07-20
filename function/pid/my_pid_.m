function result = my_pid_m(csvPath,pidParams,tuningMode)

    if nargin < 1 || isempty(csvPath)
        error('csvPath 不能为空');
    end
    if nargin < 2 || isempty(pidParams)
        pidParams = struct();
    end
    if nargin < 3 || isempty(tuningMode)
        tuningMode = 'balanced';
    end
    validateattributes(tuningMode, 'char', {'nonempty'});
    if ~ismember(tuningMode, {'balanced', 'aggressive', 'conservative'})
        error('tuningMode 必须是 balanced/aggressive/conservative');
    end


    function defaults = getDefaultPidParams()
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