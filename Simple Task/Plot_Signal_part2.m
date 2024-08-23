% Tugas 1 Sistem Linier
% Nama    : Juhen Fashikha Wildan
% NIM     : 163221047

%% Soal 1.1 (bagian a)
% Sinyal Waktu Kontinyu
t = -3:0.01:3;
s1 = (t>=-2)-(t>=-1);
s2 = (t>=1)-(t>=2);
s3 = 2*(t>=-1)-2*(t>=0);
s4 = 2*(t>=0)-2*(t>=1);
figure;
plot(t, s1, t, s2, t, s3, t, s4)
ylim([0 3])
xlim([-3 3])
grid on

x = s1+s2+s3+s4;
figure;
plot(t, x, 'k', "LineWidth",2)
ylim([0 2.5])
xlabel('t')
ylabel('x(t)')
title('Plot Sinyal')
grid on

%% Soal 1.1 (bagian b)
% Sinyal Waktu Kontinyu
t = -5:0.01:5;
f1 = (t+4)/3;
f2 = (-t+4)/3;
s1 = (t>=-4)-(t>=-1);
s2 = (t>=1)-(t>=4);
s3 = (t>=-1)-(t>=0);
s4 = (t>=0)-(t>=1);
figure;
plot(t, f1, t, f2, t, s3, t, s4)
ylim([0 2])
xlim([-5 5])
grid on

f1 = f1.*s1;
f2 = f2.*s2;
figure;
plot(t, f1, t, f2, 'LineWidth',2)

x = f1+f2+s3+s4;
figure;
plot(t, x, 'k', "LineWidth",2)
ylim([0 1.5])
xlabel('t')
ylabel('x(t)')
title('Plot Sinyal')
grid on

%% Soal 1.1 (bagian c)
% Sinyal Waktu Kontinyu
t = -7:0.01:7;
f3 = (t/1.5+6);
f4 = (-t/1.5+6);
s1 = 2*(t>=-6)-2*(t>=-3);
s2 = 2*(t>=3)-2*(t>=6);
s3 = (t>=-3)-(t>=0);
s4 = (t>=0)-(t>=3);
figure;
plot(t, s1, t, s2, t, s3, t, s4, t, f3, t, f4)
ylim([0 7])
xlim([-7 7])
grid on

f3 = f3.*s3;
f4 = f4.*s4;
figure;
plot(t, f3, t, f4, 'LineWidth',2)

x = s1+s2+f3+f4;
figure;
plot(t, x, 'k', "LineWidth",2)
ylim([0 7])
xlabel('t')
ylabel('x(t)')
title('Plot Sinyal')
grid on

%% Soal 1.1 (bagian d)
% Sinyal Waktu Kontinyu
t = -4:0.01:4;
f1 = (-t+2);
f2 = (t+2);
s1 = (t>=-2)-(t>=0);
s2 = (t>=0)-(t>=2);
figure;
plot(t, s1, t, s2, t, f1, t, f2)
ylim([0 5])
xlim([-4 4])
grid on

f1 = f1.*s1;
f2 = f2.*s2;
figure;
plot(t, f1, t, f2, 'LineWidth',2)

x = f1+f2;
figure;
plot(t,x, 'k', "LineWidth",2)
ylim([0 5])
xlim([-4 4])
xlabel('t')
ylabel('x(t)')
title('Plot Sinyal')
grid on

%% Soal 1.1 (bagian e)
% Sinyal Waktu Kontinyu
t = -1:0.01:10;
s1 = (t>=0)-(t>=1);
s2 = (t>=2)-(t>=3);
s3 = (t>=4)-(t>=5);
figure;
plot(t, s1, t, s2, t, s3)
ylim([0 2])
xlim([-1 7])
grid on

x = s1+s2+s3;
figure;
plot(t,x, 'k', "LineWidth",2)
ylim([0 1.5])
xlim([-1 7])
xlabel('t')
ylabel('x(t)')
title('Plot Sinyal')
grid on

%% Soal 1.4 (bagian a)
% Sinyal Waktu Kontinyu
t = -2:0.01:4;

% Menghitung x(t) secara langsung menggunakan ekspresi matematika
x = (t >= -1) - 2.*(t >= 1) + (t >= 3);

% Plot sinyal
figure;
plot(t, x, 'b', 'LineWidth', 2);
xlabel('t');
ylabel('x(t)');
ylim([-1.2 1.2])
title('Plot Sinyal x(t) = u(t + 1) - 2u(t - 1) + u(t - 3)');
grid on;

%% Soal 1.4 (bagian b)
% Sinyal Waktu Kontinyu
t = -1:0.01:3;

% Menghitung x(t) secara langsung menggunakan ekspresi matematika
x = (t + 1).*((t >= 1)) - t.*(t >= 0) + (t >= 2);

% Plot Sinyal
figure;
plot(t, x, 'b', 'LineWidth', 2);
xlabel('t');
ylabel('x(t)');
ylim([-1.2 2.2])
title('Plot Sinyal x(t) = (t+1)u(t - 1) - t*u(t) + u(t - 2)');
grid on;

%% Soal 1.4 (bagian c)
% Sinyal Waktu Kontinyu
t = 0:0.01:5;

% Menghitung x(t) secara langsung menggunakan ekspresi matematika
x = 2*(t - 1).*((t >= 1)) - 2*(t-2).*((t >= 2)) + 2*(t-3).*((t >= 3));

% Plot Sinyal
figure;
plot(t, x, 'b', 'LineWidth', 2);
xlabel('t');
ylabel('x(t)');
ylim([-0.5 5])
title('Plot Sinyal x(t) = 2(t-1)u(t-1) - 2(t-2)u(t-2) + 2(t-3)u(t-3)');
grid on;

%% Soal 1.7 (bagian a)
% Sinyal Waktu Diskrit
n = -5:0.5:5;

% Menghitung nilai x[n] yang merupakan fungsi satuan step u[n]
x = (n >= 0); % step

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
ylim([0 1.2])
title('Plot Sinyal x[n] = u[n]');
grid on;

%% Soal 1.7 (bagian b)
% Sinyal Waktu Diskrit
n = -5:0.5:5;

% Menghitung nilai x[n] yang merupakan fungsi satuan ramp u[n]
x = n .* (n >= 0); % ramp

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
ylim([0 5.2])
title('Plot Sinyal x[n] = n * u[n]');
grid on;

%% Soal 1.7 (bagian c)
% Sinyal Waktu Diskrit
n = -2:0.5:10;

x = (0.5).^n.*(n>=0); % exponential decay

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
ylim([0 1.2])
title('Plot Sinyal x[n] = (0.5)^n u[n]');
grid on;

%% Soal 1.7 (bagian d)
% Sinyal Waktu Diskrit
n = -2:0.2:20;

x = (-0.5).^n.*(n>=0); % exponential alternating

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
xlim([-1 10])
ylim([-0.7 1.2])
title('Plot Sinyal x[n] = (-0.5)^n u[n]');
grid on;

%% Soal 1.7 (bagian e)
% Sinyal Waktu Diskrit
n = 80:1:100;

x = 2.^n.*(n>=0); % exponential growth

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
xlim auto
ylim([-5 1e30])
title('Plot Sinyal x[n] = 2^n u[n]');
grid on;

%% Soal 1.7 (bagian f)
% Sinyal Waktu Diskrit
n = 0:0.5:20;

x = sin(pi*n/4);

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
ylim([-1.2 1.2])
title('Plot sinyal x[n] = sin(\pi*n/4)');
grid on;

%% Soal 1.7 (bagian g)
% Sinyal Waktu Diskrit
n = 0:0.5:10;

x = sin(pi*n/2);

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
ylim([-1.2 1.2])
title('Plot sinyal x[n] = sin(\pi*n/2)');
grid on;

%% Soal 1.7 (bagian h)
% Sinyal Waktu Diskrit
n = -5:0.5:20;

x = (0.9).^n.*(sin(pi*n/4) + cos(pi*n/4));

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
ylim([-2.2 1.5])
title('Plot sinyal x[n] = (0.9)^n(sin(\pi*n/4) + cos(\pi*n/4))');
grid on;

%% Soal 1.7 (bagian i)
% Sinyal Waktu Diskrit
n = -5:0.5:6;

x = (n >= -4) & (n <= 4); % rectangular pulse

% Plot Sinyal
figure;
stem(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x[n]');
xlim([-5 6])
ylim([0 1.5])
title('Plot Sinyal -4 <= n <= 4');
grid on;

%% Soal 1.9 (bagian a)
% Sinyal Kontinyu dengan skala waktu kecil
t = 0:0.001:10;

x = cos(pi * t) + cos(4 * pi * t / 5);

% Plot Sinyal
figure;
plot(t, x, 'b', 'LineWidth', 2);
xlabel('t');
ylabel('x(t)');
title('Plot Sinyal x(t) = cos(\pi*t) + cos(4\pi*t/5)');
grid on;

%% Soal 1.9 (bagian b)
% Sinyal Kontinyu dengan skala waktu kecil
t = 0:0.001:10;

x = cos(2 * pi * (t-4)) + sin(5 * pi * t);

% Plot Sinyal
figure;
plot(t, x, 'b', 'LineWidth', 2);
xlabel('t');
ylabel('x(t)');
xlim([0 9])
title('Plot Sinyal x(t) = cos(2\pi*(t - 4)) + sin(5\pi*t)');
grid on;

%% Soal 1.9 (bagian c)
% Sinyal Kontinyu dengan skala waktu kecil
t = -5:0.001:10;

x = cos(2 * pi * t) + sin(10 * t);

% Plot Sinyal
figure;
plot(t, x, 'b', 'LineWidth', 2);
xlabel('t');
ylabel('x(t)');
xlim([-3 9])
title('Plot Sinyal x(t) = cos(2\pi*t) + sin(10*t)');
grid on;

%% Soal 1.9 (bagian d)
% Sinyal Kontinyu dengan skala waktu kecil
n = 0:0.001:10;

x = sin(10*n);

% Plot Sinyal
figure;
plot(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x(n)');
title('Plot Sinyal x(t) = sin(10*n)');
grid on;

%% Soal 1.9 (bagian e)
% Sinyal Kontinyu dengan skala waktu kecil
n = 0:0.001:10;

x = sin(10*pi*n/3);

% Plot Sinyal
figure;
plot(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x(n)');
title('Plot Sinyal x(t) = sin(10\pi*n/3)');
grid on;

%% Soal 1.9 (bagian f)
% Sinyal Kontinyu dengan skala waktu kecil
n = 0:0.001:10;

x = cos(pi*(n).^2);

% Plot Sinyal
figure;
plot(n, x, 'b', 'LineWidth', 2);
xlabel('n');
ylabel('x(n)');
title('Plot Sinyal x(t) = cos(\pi*n^2)');
grid on;
