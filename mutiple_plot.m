t = tiledlayout(2,2)
title(t,"trigonometric Functions")
x = linspace(0,30);

nexttile
plot(x,sin(x))
title("sine")

nexttile 
plot(x,cos(x))
title("cosine")

nexttile    
plot(x,tan(x))
title("tangent")

nexttile
plot(x,sec(x))
title("secant")