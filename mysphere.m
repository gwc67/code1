[x,y,z] = sphere;
r = 2;
surf(x*r,y*r,z*r)
axis equal          %USE the same scale for eachaxis

A = 4*pi*r^2;  
v = (4/3)*pi*r^3;