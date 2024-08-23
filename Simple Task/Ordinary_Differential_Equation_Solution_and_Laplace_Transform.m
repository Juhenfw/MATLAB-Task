% Clear workspace and command window
clear all
close all
clc

%% Solusi 1st Order ODE
syms y(t)
ode = diff(y,t) == 1 + 2*y;
cond = y(0) == 1;
ySol(t) = dsolve(ode, cond);

% Plot solusi 1st Order ODE
t = 0:0.1:2;
y = (3/2) * exp(2*t) - (1/2);
figure;
plot(t, y, 'LineWidth', 1.5)
grid on
title('Solusi ODE 1st Order')
xlabel('t')
ylabel('y(t)')

%% Solusi 2nd Order ODE
syms y(t)
Dy = diff(y);
ode = diff(y,t,2) == -6*diff(y,t) - 8*y + 1;
cond1 = y(0) == 0;
cond2 = Dy(0) == 1;
conds = [cond1 cond2];
ySol(t) = dsolve(ode, conds);

% Plot solusi 2nd Order ODE
t = 0:0.01:5;
y = (1/4)*exp(-2*t) - (3/8)*exp(-4*t) + (1/8);
figure;
plot(t, y, 'LineWidth', 1.5)
grid on
title('Solusi ODE 2nd Order')
xlabel('t')
ylabel('y(t)')

%% Transformasi Laplace Menggunakan MATLAB
syms s H X Y
H = 2/(s^2 + 4*s + 4);
X = 1/s;
Y = H*X;
y = ilaplace(Y);

% Plot inverse Laplace transform
t = 0:0.01:10;
ysol = (1/2) - t.*exp(-2*t) - (1/2)*exp(-2*t);
figure;
plot(t, ysol, 'LineWidth', 1.5)
grid on
title('Inverse Transformasi Laplace')
xlabel('t')
ylabel('y(t)')
ylim([0 0.6])

% Simulasi input step menggunakan tf
num = [2];
den = [1 4 4];
H = tf(num, den);
t = 0:0.01:10;
y = step(H, t);
figure;
plot(t, y, 'LineWidth', 1.5)
grid on
title('Respons Step')
xlabel('t')
ylabel('y(t)')
ylim([0 0.6])

% Simulasi lsim dengan input step
t = 0:0.01:10;
u = 2*heaviside(t);
y = lsim(H, u, t);
figure;
plot(t, y, t, u, 'LineWidth', 1.5)
grid on
title('Respons lsim')
xlabel('t')
ylabel('y(t)')
ylim([0 2.2])

%% Pemodelan Motor DC menggunakan MATLAB
% Parameter Motor DC
J = 0.01; % kg.m^2
b = 0.1;  % N.m.s
Kt = 0.01; % N.m/A
Kv = 0.01; % V/(rad/s)
R = 1;    % Ohm
L = 0.5;  % Henry

% Transfer Function Motor DC
G = tf(Kt, [J*L, (J*R + b*L), (b*R + Kt*Kv)]);

% Generate input signal (square wave)
tau = 16;
Tf = 32;
Ts = 0.01;
[v, t] = gensig("square", tau, Tf, Ts);
v = 5*v;

% Plot input voltage signal
figure;
plot(t, v, "LineWidth", 1.5)
grid on
xlabel('(sec)')
ylabel('(Volt)')
ylim([-0.5 5.5])
xlim([0 32])

% Simulasi kecepatan putar motor
kec_putar = lsim(G, v, t);
figure;
plot(t, kec_putar, 'LineWidth', 1.5)
grid on
xlabel('(sec)')
ylabel('(rad/sec)')
ylim([-0.1 0.6])
xlim([0 32])

% Konversi kecepatan putar ke rpm
kec_putar = 9.5493 * kec_putar;
figure;
plot(t, kec_putar, 'LineWidth', 1.5)
grid on
xlabel('(sec)')
ylabel('(rpm)')
ylim([-0.5 5])
xlim([0 32])
