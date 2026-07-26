
addpath(fullfile(pwd,"fresh\"));
% preprocess_raw_log("./excel_csv/still_circle.csv");
addpath(fullfile(pwd,"function\",'pid'));
addpath(fullfile(pwd,"function\",'track')); %添加路径
addpath("filiter\");
addpath("excel_csv\still_circle_waypoints\");
addpath("excel_csv\test9\");
addpath("function\pid\")
opts =struct('output_dir','excel_csv\zhen','output_name','rt_tar_is_zhen');
my_flitter("excel_csv\question_2.csv",opts)

opts.X.Kp_base = 0.1;
opts.Y.Kp_base = 0.1;
opts.X.Ki_base = 0.06;
opts.Y.Ki_base = 0.06;
opts.X.Kd_base = 0.2;
opts.Y.Kd_base = 0.2;

% my_pid_analyze("excel_csv\question_2.csv",opts);
% analyze_overshoot("D:\Documents\MATLAB\code1\excel_csv\kkp=0.1--ki=0.06--kd=0.2_2\kp=0.1--ki=0.06--kd=0.2_2.csv");
% result = mathematical_pid_analysis('D:\Documents\MATLAB\code1\excel_csv\kkp=0.1--ki=0.06--kd=0.2_2\kp=0.1--ki=0.06--kd=0.2_2.csv');


