clear all
close all
clc

% Generate input signals
Fs      = 1000;                     % samples per second
dt      = 1/Fs;                     % seconds per sample
StopTime= 1;                        % seconds
t       = (0:dt:StopTime-dt)';      % seconds

% Generate input 1
A1      = 50;
Fin1    = 10;                       % hertz

% Generate input 2
A2      = 10;
Fin2    = 200;                      % hertz

% Generate input 3
A3      = 25;
Fin3    = 450;                      % hertz

x       = A1*sin(2*pi*Fin1*t) + A2*sin(2*pi*Fin2*t) + A3*sin(2*pi*Fin3*t);

% Plot input signals
figure;
plot(t, x);
title('Sinyal Input (Domain Waktu) - x(t)','FontSize',14,'FontWeight','bold');
xlabel('Waktu (detik)');
ylabel('Amplitudo');

% Fourier Transform (FFT)
L       = length(x);                    % identify data length
NFFT    = 2^nextpow2(L);                % count data step
fftx    = fft(x, NFFT)/L;               % Fourier transform of x
f       = Fs/2*linspace(0,1,NFFT/2+1);  % generate frequency data step

% Plot frequency spectrum of input signals
figure;
plot(f, 2*abs(fftx(1:NFFT/2+1)));
title('Spektrum Frekuensi Sinyal Input - X(f)','FontSize',14,'FontWeight','bold');
xlabel('Frekuensi (Hz)');
ylabel('Amplitudo');

% Generate filter coefficients (FIR)
Num = [-6.77077453069605e-06, 8.95706626106057e-05, -0.000593166801404474, ...
        0.00260505655219974, -0.00848591870104584, 0.0217484483139414, ...
        -0.0454287932312008, 0.0790997608709048, -0.116493933981783, ...
        0.146457163114763, 0.842017167684424, 0.146457163114763, ...
        -0.116493933981783, 0.0790997608709048, -0.0454287932312008, ...
        0.0217484483139414, -0.00848591870104584, 0.00260505655219974, ...
        -0.000593166801404474, 8.95706626106057e-05, -6.77077453069605e-06];
Den = 1;

% Filter Implementation
y = filter(Num, Den, x);

% Plot output signals
figure;
plot(t, y);
title('Sinyal Output (Domain Waktu) - y(t)','FontSize',14,'FontWeight','bold');
xlabel('Waktu (detik)');
ylabel('Amplitudo');

% Fourier Transform (FFT) of the output signal
ffty    = fft(y, NFFT)/L;               % Fourier transform of y

% Plot frequency spectrum of output signals
figure;
plot(f, 2*abs(ffty(1:NFFT/2+1)));
title('Spektrum Frekuensi Sinyal Output - Y(f)','FontSize',14,'FontWeight','bold');
xlabel('Frekuensi (Hz)');
ylabel('Amplitudo');
