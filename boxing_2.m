

data = readmatrix("speed_15.csv","Range",'A2:C106850');

% time = T.Time;
% signal = T.voltage;

% uiimport("3_kd_0_5.csv") 导入波形

x = data(:,1);    %radar_x
y = data(:,2);    %radar_y
z = data(:,3);    %radar_z
figure;

targets_raw = readmatrix("speed_15.csv","Range","D2:F106850");

validRows = ~any(isnan(targets_raw),2);
targets_clean = targets_raw(validRows,:);

targets = unique(targets_clean,"rows",'stable');

fprintf('原始目标点: %d 个 → 去重后: %d 个 (去除 %.1f%%)\n', ...
        size(targets_raw,1), size(targets,1), ...
        (1 - size(targets,1)/size(targets_raw,1))*100);
nTargets = size(targets,1);
minDist = zeros(nTargets,1);
nearestIdx = zeros(nTargets,1); %zeros 创建纯0数组

for i = 1 : nTargets
    %计算目标点到轨迹上所有点的欧式距离
    dists = sqrt((x - targets(i,1)).^2 + ...
            (y - targets(i,2)).^2 + ... 
             (z - targets(i,3).^2));
    [minDist(i),nearestIdx(i)] = min(dists);
end

figure('Color','w','Position',[100 100 1200 900]); %可绘制区域的大小

%绘制雷达的实际轨迹
plot3(x,y,z,'b-','LineWidth',1);

hold on;  %当时给一个界面下可以绘制多个曲线

scatter3(targets(:,1),targets(:,2),targets(:,3),...
    120,'red','filled','MarkerFaceAlpha',0.8);


%绘制目标点

for i = i: nTargets
    idx = nearestIdx(i);
    plot3([targets(i,1),x(idx)],...
        [targets(i,2),y(idx)],...
        [targets(i,3),z(idx)],...
        'r--','LineWidth',1.5);
 
    midX = (targets(i,1) + x(idx)) / 2;
    midY = (targets(i,2) + y(idx)) / 2;
    midZ = (targets(i,3) + z(idx)) / 2;
    text(midX, midY, midZ, sprintf('%.3f', minDist(i)), ...
         'FontSize', 9, 'Color', 'm', 'FontWeight', 'bold')
end


axis equal; grid on; box on;
 
xlabel('radar\x');
ylabel('radar\y');
zlabel('radar\z');
title('雷达3D轨迹 和 目标点差异分析');
legend('雷达实际轨迹','目标点','位置误差','Location','best');

hold off;

%% 5. 控制台输出量化差异
fprintf('\n===== 目标点差异分析结果 =====\n');
fprintf('%-8s %-12s %-12s\n', '目标编号', '最小距离(m)', '最近轨迹点索引');
fprintf('%s\n', repmat('-', 1, 35));
for i = 1:nTargets
    fprintf('%-8d %-12.4f %-12d\n', i, minDist(i), nearestIdx(i));
end
fprintf('平均偏差: %.4f m\n', mean(minDist));
fprintf('最大偏差: %.4f m\n', max(minDist));