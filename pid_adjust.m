
addpath(fullfile(pwd,"fresh\"));
% preprocess_raw_log("./excel_csv/still_circle.csv");
addpath(fullfile(pwd,"function\",'pid'));
addpath(fullfile(pwd,"function\",'track')); %添加路径
addpath("filiter\");
addpath("excel_csv\still_circle_waypoints\");
addpath("excel_csv\test9\");
addpath("function\pid\")
% opts =struct('output_dir','excel_csv/still_circle_2','output_name','mystill_circle_2');
% my_flitter("excel_csv/still_circle_2.csv",opts)

% pid_params = struct('Kp',0.1);
% pid_params.X.Kp_base = 0.1;
% pid_params.X.Ki_base = 0.2;
% pid_params.X.Kd_base = 0.6;
% my_pid_analyze("excel_csv/test9.csv",pid_params);
% opts = detectImportOptions("excel_csv/test9/test9_my.csv.csv");
% opts.VariableNamesLine = 1;
% opts.DataLines = [2,Inf];     

opts.X.Kp_base = 0.18;

my_pid_analyze("excel_csv\still_circle_2\mystill_circle_2.csv",opts);
analyze_overshoot("excel_csv\still_circle_2\mystill_circle_2.csv");
identify_and_tune("excel_csv\still_circle_2\mystill_circle_2.csv");
% col_names = opts.VariableNames;
% radar_idx = find(contains(col_names,"RADAR_POS"));
% target_idx = find(contains(col_names,"TARGET_POS"));
% cmd_idx    = find(contains(col_names,"CMD_SPEED"));
% trel_idx =  find(contains(col_names,'T_REL'));

% disp(radar_idx);
% disp(opts);

% opts.DataLines = [3,Inf];

% T = readtable("excel_csv/test9.csv",opts);
% col_names = T.Properties.VariableNames;
% data_mat = table2array(T);

% disp('列名');
% disp(col_names);
% disp('前5行')
% disp(data_mat(1:5,:));

% fprintf('names_line = %d \n',opts.VariableNamesLine);
% s = '   TICK , X , Y , Z  '
% s2 = strtrim(s)
    % s2 结果：'TICK , X , Y , Z'

