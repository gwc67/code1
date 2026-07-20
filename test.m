% analyze_radar_trajectory_2("./excel_csv/still_circle.csv");
% addpath(fullfile(pwd,"fresh\"));
% preprocess_raw_log("./excel_csv/still_circle.csv");
% addpath(fullfile(pwd,"function\",'pid'));
% addpath(fullfile(pwd,"function\",'track')); %添加路径
addpath("filiter\")

pre_log("./excel_csv/still_circle.csv")
% analyze_pid_performance("./excel_csv/still_circle_radar_sync.csv");
% plot_radar_3d_optimized("./excel_csv/still_circle_radar_sync.csv");

% figure;
% subplot(2,1,1);plot(T_REL,ERROR_X);title("输入激励");
% subplot(2,1,2);plot(T_REL,CMD_SPEED_X);title("系统输出y");



% data_id = iddata(CMD_SPEED_X,ERROR_X,0.1);

% ident
% plant_tf = tfest(data_id,2,0)

% 传入tf模型 + 指定控制器类型PI（无人机位置不用D）
% pidTuner(plant_tf, "PID");