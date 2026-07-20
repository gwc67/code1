data = readmatrix("3_kd_0_5.csv");

data = readmatrix("speed_15.csv","Range",'A2:C106850');

T = readtable("3_kd_0_5.csv");

% time = T.Time;
% signal = T.voltage;

% uiimport("3_kd_0_5.csv") 导入波形

x = data(:,1);    %radar_x
y = data(:,2);    %radar_y
z = data(:,3);    %radar_z
figure;
plot3(x,y,z,'b-','LineWidth',1);

axis equal
xlabel('radar_x');
ylabel('radar_y');
zlabel('radar_z');
title('导入的excel波形');
grid on;
