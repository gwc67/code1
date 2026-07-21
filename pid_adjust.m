
addpath(fullfile(pwd,"fresh\"));
% preprocess_raw_log("./excel_csv/still_circle.csv");
addpath(fullfile(pwd,"function\",'pid'));
addpath(fullfile(pwd,"function\",'track')); %添加路径
addpath("filiter\");
addpath("excel_csv\still_circle_waypoints\");

% opts =struct('dt',0.05,'output_dir',' excel_csv\still_circle_waypoints\','output_name',' my.csv');
opts = struct('dt',0.1);
my_flitter("excel_csv/still_circle.csv",opts)

