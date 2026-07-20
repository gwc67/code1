x = linspace(-2,2,20);
y = x';
z = x .* exp(-x.^2 - y.^2); %3D 绘图还是很nb的
surf(x,y,z) 

