
addpath(fullfile(pwd,"fresh\"));
% preprocess_raw_log("./excel_csv/still_circle.csv");
addpath(fullfile(pwd,"function\",'pid'));
addpath(fullfile(pwd,"function\",'track')); %添加路径
addpath("filiter\");
addpath("excel_csv\still_circle_waypoints\");
addpath("excel_csv\test9\");
addpath("function\pid\")
% opts =struct('output_dir','excel_csv\ki-0.13-kd-0.34-radar','output_name','1');
my_flitter("excel_csv\7-27-21",opts);
% my_flitter("excel_csv\7-27-21",opts)

opts.X.Kp_base = 0.15;
opts.Y.Kp_base = 0.15;
opts.X.Ki_base = 0.1;
opts.Y.Ki_base = 0.1;
opts.X.Kd_base = 0.3;
opts.Y.Kd_base = 0.3;

% my_pid_analyze("excel_csv\zhen\3s.csv",opts);
plot_radar_3d('excel_csv\zhen\3s.csv');
% play_radar_3d_2('excel_csv\ki-0.13-kd-0.34-radar\1.csv');
% analyze_overshoot("D:\Documents\MATLAB\code1\excel_csv\kkp=0.1--ki=0.06--kd=0.2_2\kp=0.1--ki=0.06--kd=0.2_2.csv");
% result = mathematical_pid_analysis('D:\Documents\MATLAB\code1\excel_csv\kkp=0.1--ki=0.06--kd=0.2_2\kp=0.1--ki=0.06--kd=0.2_2.csv');


