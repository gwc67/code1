
addpath(fullfile(pwd,"fresh\"));
% preprocess_raw_log("./excel_csv/still_circle.csv");
addpath(fullfile(pwd,"function\",'pid'));
addpath(fullfile(pwd,"function\",'track')); %添加路径
addpath("filiter\");
addpath("excel_csv\still_circle_waypoints\");
addpath("excel_csv\test9\");
addpath("function\pid\")

% my_flitter("excel_csv\7-27-21",opts);

play_radar_3d_2('excel_csv\7-27-21\ki-0.13-kd-0.34\ki-0.13-kd-0.34.csv');


