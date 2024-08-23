% WAP of Linear convolution - Part (a)
clc;
clear all;
close all;

n = 0:9;
x = [1,1,1,0,0];
v = [1,1,1,1,0,0];
y = conv(x,v);
figure;
stem(n,y,'filled');
xlabel('n values');
ylabel('y[n]');
title('Linear Convolution - Part (a)');
grid on;

% WAP of Linear convolution - Part (b)
clc;
clear all;
close all;

n = 0:9;
x = [2,1,0,0,0];
v = [1,1,1,1,0,0];
y = conv(x,v);
figure;
stem(n,y,'filled')
xlabel('n values');
ylabel('y[n]');
title('Linear Convolution - Part (b)');
grid on;

% WAP of Linear convolution - Part (c)
clc;
clear all;
close all;

n = 0:9;
x = [2,1,0,0,0];
v = [0,1,2,0,0,0];
y = conv(x,v);
figure;
stem(n,y,'filled')
xlabel('n values');
ylabel('y[n]');
title('Linear Convolution - Part (c)');
grid on;
