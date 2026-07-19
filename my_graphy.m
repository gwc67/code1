x = linspace(0,2*pi);
y = sin(x);
plot(x,y,"r--")
xlabel("x")
ylabel("sin(x)")
title("正弦函数")

hold on  %添加绘图到目前窗口

y2 = cos(x);

plot(x,y2,":")