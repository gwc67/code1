% analyze_radar_trajectory_2("./excel_csv/still_circle.csv");
% preprocess_raw_log("./excel_csv/still_circle.csv");
addpath(fullfile(pwd,"function\",'pid'));
addpath(fullfile(pwd,"function\",'track')); %添加路径
analyze_pid_performance("./excel_csv/still_circle_radar_sync.csv");
plot_radar_3d("./excel_csv/still_circle_radar_sync.csv");