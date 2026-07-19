
n = 100
f(1) = 1;
f(2) = 1;

for a = 3 : n
    f(a) = f(a - 1) + f(a - 2)
end 

f(1:10)

num = randi(100)
if num < 34
    sz = 'low'
elseif num < 67
    sz = "medium"
else
    sz = 'high'
end 
