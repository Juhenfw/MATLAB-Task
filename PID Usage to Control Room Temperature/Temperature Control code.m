%% *Temperature Control System For Commonly Room*
% 

% % Uncomment (Ctrl+T) if you want to make the block diagram in Simulink

% % Create a new Simulink model
% model = 'InvertingTempControl';
% open_system(new_system(model));
% 
% % Add blocks to the model
% add_block('simulink/Sources/Step', [model '/Step']);
% add_block('simulink/Commonly Used Blocks/Sum', [model '/Sum']);
% add_block('simulink/Continuous/PID Controller', [model '/PID Controller']);
% add_block('simulink/Continuous/Transfer Fcn', [model '/Temperature Control Plant']);
% add_block('simulink/Continuous/Transfer Fcn', [model '/Sensor']);
% add_block('simulink/Commonly Used Blocks/Scope', [model '/Scope']);
% 
% % Set block parameters
% set_param([model '/Step'], 'Time', '0', 'Before', '0', 'After', '1');
% set_param([model '/Sum'], 'Inputs', '+-');
% 
% % Define realistic parameters
% M = 200; % Thermal mass including air and occupants (kg)
% C = 1005; % Specific heat capacity of air (J/kg·K)
% m = 0.1; % Mass flow rate of air (kg/s)
% Cp = 1005; % Specific heat capacity of air (J/kg·K)
% k = 10; % Heat transfer coefficient (W/m²·K)
% A = 30; % Surface area for heat exchange (m²)
% X = 0.3; % Wall thickness (m)
% 
% % Define the numerator and denominator for the plant transfer function
% plant_num = 1;
% plant_den = [(M * C) (m * Cp + (k * A)/X)];
% set_param([model '/Temperature Control Plant'], 'Numerator', mat2str(plant_num), 'Denominator', mat2str(plant_den));
% 
% % Sensor parameters
% m_sen = 0.01; % Mass of sensor (kg)
% C_sen = 500; % Specific heat capacity of sensor (J/kg·K)
% h_conv = 25; % Convection heat transfer coefficient (W/m²·K)
% A_sen = 0.01; % Surface area of sensor (m²)
% 
% % Define the numerator and denominator for the sensor transfer function
% sensor_num = 1;
% sensor_den = [(m_sen * C_sen)/(h_conv * A_sen) 1];
% set_param([model '/Sensor'], 'Numerator', mat2str(sensor_num), 'Denominator', mat2str(sensor_den));
% 
% % Add lines to connect the blocks
% add_line(model, 'Step/1', 'Sum/1');
% add_line(model, 'Sum/1', 'PID Controller/1');
% add_line(model, 'PID Controller/1', 'Temperature Control Plant/1');
% add_line(model, 'Temperature Control Plant/1', 'Scope/1');
% add_line(model, 'Temperature Control Plant/1', 'Sensor/1');
% add_line(model, 'Sensor/1', 'Sum/2');
% 
% % Save and open the model
% save_system(model);
% open_system(model);

% % Batas Uncomment
%% |*-- Start Code for The System --*|

% Define parameters
M   = 200;        % Thermal mass including air and occupants (kg)
C   = 1005;       % Specific heat capacity of air (J/kg·K)
m   = 0.1;        % Mass flow rate of air (kg/s)
Cp  = 1005;       % Specific heat capacity of air (J/kg·K)
k   = 10;         % Heat transfer coefficient (W/m²·K)
A   = 30;         % Surface area for heat exchange (m²)
X   = 0.3;        % Wall thickness (m)

% Sensor parameters
m_sen = 0.01;     % Mass of sensor (kg)
C_sen = 500;      % Specific heat capacity of sensor (J/kg·K)
h_conv = 25;      % Convection heat transfer coefficient (W/m²·K)
A_sen = 0.01;     % Surface area of sensor (m²)

% Transfer function for Temperature Control Plant
num_plant = 1;
den_plant = [(M * C), (m * Cp + (k * A) / X)];
plant_tf = tf(num_plant, den_plant);

% Transfer function for Sensor
num_sensor = 1;
den_sensor = [(m_sen * C_sen) / (h_conv * A_sen), 1];
sensor_tf = tf(num_sensor, den_sensor);
% *Open Loop Plant*

% Open-loop transfer function (Plant only)
open_loop_tf = plant_tf
figure;
step(open_loop_tf);
title('Open Loop Plant Step Response');
hold off;
stepinfo(open_loop_tf)
figure;
rlocusplot(open_loop_tf);
grid on;
title('Root Locus Open Loop Plant');
hold off;
% *Closed Loop System Without Controller*

closed_loop_sys = feedback(open_loop_tf, sensor_tf)
figure;
step(closed_loop_sys);
title('Closed Loop Plant Step Response without Controller');
hold off;
stepinfo(closed_loop_sys)
figure;
rlocusplot(closed_loop_sys);
grid on;
title('Root Locus Closed Loop Plant without Controller');
hold off;
% *Close Loop System (First Step Tuning Method Ziegler Nichols)*

Kp = 45000; % Signal Oscilated
Ki = 0;
Kd = 0;

pid_controller = pid(Kp, Ki, Kd);

% Closed-loop transfer function
closed_loop_sys = feedback(series(pid_controller, open_loop_tf), sensor_tf);

% Step response
figure;
step(closed_loop_sys);
title('Initial PID Tuning - First Step Ziegler Nichols');
hold off;
stepinfo(closed_loop_sys)
figure;
rlocusplot(closed_loop_sys);
grid on;
title('Root Locus for First Step Ziegler Nichols');
hold off;
%% 
% 
% 
% *Dari hasil setting awal Kp, didapatkan Kcr = 45000 dan f = 15.661 mHz atau 
% Pcr = 63.852 detik; Sehingga:

% Measure Ku and Pu from the oscillations
Kcr = 45000;  
Pcr = 63.852; % second 

% Calculate PID parameters using Ziegler-Nichols or Tyreus-Luyben
Kp = 0.6 * Kcr
Ti = 0.5 * Pcr;
Td = 0.125 * Pcr;

% Convert to PID gains
Ki = Kp / Ti
Kd = Kp * Td

pid_controller = pid(Kp, Ki, Kd);

% Closed-loop transfer function
closed_loop_sys = feedback(series(pid_controller, open_loop_tf), sensor_tf)

figure;
step(closed_loop_sys);
title('Final Tuning by Ziegler-Nichols');
xlabel('Time (s)');
ylabel('Temperature (°C)');
hold off;

% Step response information
step_info = stepinfo(closed_loop_sys);
disp('Step Response Information:');
disp(step_info);
figure;
rlocusplot(closed_loop_sys);
grid on;
title('Root Locus Final Tuning by Ziegler-Nichols');
hold off;
%% 
% 
% 
% 
% 
% 
% *Set Damping Ratio to 0 < zeta < 1 (Underdamped Response)*

% Definisikan parameter sistem dan PID
Kp = 27000;
Ki = 845.7057;
Kd = 2.1550e+05;

% Transfer function dari PID controller
pid_controller = pid(Kp, Ki, Kd);
closed_loop_sys = feedback(series(pid_controller, open_loop_tf), sensor_tf);

% Poles dari closed-loop system
poles = pole(closed_loop_sys);

% Hitung damping ratio dan natural frequency
[zeta, omega_n] = damp(closed_loop_sys);

% Tampilkan hasil
disp('Poles of the closed-loop system:');
disp(poles);
disp('Damping Ratio:');
disp(zeta);
disp('Natural Frequency:');
disp(omega_n);

% Identifikasi mode dominan (poles dengan bagian real terkecil)
[~, idx] = min(abs(real(poles)));
dominant_pole = poles(idx);
dominant_zeta = zeta(idx);
dominant_omega_n = omega_n(idx);

% Tampilkan mode dominan
disp('Dominant Pole:');
disp(dominant_pole);
disp('Dominant Damping Ratio:');
disp(dominant_zeta);
disp('Dominant Natural Frequency:');
disp(dominant_omega_n);
%%
% Analyze initial response
info = stepinfo(closed_loop_sys);
disp('Initial Step Response Information:');
disp(info);

% Iteratively adjust PID parameters to achieve underdamped response
for i = 1:100
    % Analyze current response
    info = stepinfo(closed_loop_sys);
    damping_ratio = dominant_zeta;
    natural_frequency = dominant_omega_n;
    % Adjust Kp, Ki, and Kd to maintain underdamped response
    if damping_ratio >= 1
        Kp = Kp * 1.1; % Increase Kp
        Ki = Ki * 1.1; % Increase Ki
        Kd = Kd * 1.1; % Increase Kd
    elseif damping_ratio < 0.1
        Kp = Kp * 0.9; % Decrease Kp
        Ki = Ki * 0.9; % Decrease Ki
        Kd = Kd * 0.9; % Decrease Kd
    end
    
    % Create updated PID controller
    pid_controller = pid(Kp, Ki, Kd);
    
    % Closed-loop transfer function with updated PID
    closed_loop_sys = feedback(series(pid_controller, plant_tf), sensor_tf);
    
    % Step response with updated PID
    figure;
    step(closed_loop_sys);
    title(['Step Response with Updated PID (Iteration ', num2str(i), ')']);
    xlabel('Time (s)');
    ylabel('Temperature (°C)');
    grid on;
    
    % Analysis of updated response
    info = stepinfo(closed_loop_sys);
    disp(['Updated Step Response Information (Iteration ', num2str(i), '):']);
    disp(info);
    
    % Check if the system is underdamped with desired characteristics
    if damping_ratio > 0 && damping_ratio < 1
        disp('System is underdamped.');
        break;
    end
    
    % Pause to observe the response before next iteration
    pause(2);
end

Underdamped = closed_loop_sys;
% *Set Damping Ratio to 1 (Critically damped)*

% Definisikan parameter sistem dan PID
Kp = 27000;
Ki = 845.7057;
Kd = 2.1550e+05;

% Transfer function dari PID controller
pid_controller = pid(Kp, Ki, Kd);
closed_loop_sys = feedback(series(pid_controller, open_loop_tf), sensor_tf);

% Closed-loop transfer function
G_cl = closed_loop_sys;

% Poles dari closed-loop system
poles = pole(G_cl);

% Hitung damping ratio dan natural frequency
[zeta, omega_n] = damp(G_cl);

% Tampilkan hasil
disp('Poles of the closed-loop system:');
disp(poles);
disp('Damping Ratio:');
disp(zeta);
disp('Natural Frequency:');
disp(omega_n);

% Identifikasi mode dominan (poles dengan bagian real terkecil)
[~, idx] = min(abs(real(poles)));
dominant_pole = poles(idx);
dominant_zeta = zeta(idx);
dominant_omega_n = omega_n(idx);

% Tampilkan mode dominan
disp('Dominant Pole:');
disp(dominant_pole);
disp('Dominant Damping Ratio:');
disp(dominant_zeta);
disp('Dominant Natural Frequency:');
disp(dominant_omega_n);
%%
% Analyze initial response
info = stepinfo(closed_loop_sys);
disp('Initial Step Response Information:');
disp(info);

% Iteratively adjust PID parameters
for i = 1:100
    % Analyze current response
    info = stepinfo(closed_loop_sys);
    
    % Adjust Kp, Ki, and Kd to increase damping and reduce overshoot
    if info.Overshoot > 0
        Kp = Kp * 0.9; % Decrease Kp by 10%
        Ki = Ki * 0.9; % Decrease Ki by 10%
        Kd = Kd * 0.9; % Decrease Kd by 10%

    else
        break; % Break the loop if overshoot is zero
    end
    
    % Create updated PID controller
    pid_controller = pid(Kp, Ki, Kd);
    
    % Closed-loop transfer function with updated PID
    closed_loop_sys = feedback(series(pid_controller, plant_tf), sensor_tf);
    
    % Step response with updated PID
    figure;
    step(closed_loop_sys);
    title(['Step Response with Updated PID (Iteration ', num2str(i), ')']);
    xlabel('Time (s)');
    ylabel('Output');
    grid on;
    
    % Analysis of updated response
    info = stepinfo(closed_loop_sys);
    disp(['Updated Step Response Information (Iteration ', num2str(i), '):']);
    disp(info);
    
    % Check if the system is overdamped
    if info.Overshoot == 0 && info.PeakTime > 0
        disp('System is Critically damped.');
        break;
    end
    
    % Pause to observe the response before next iteration
    pause(2);
end

Criticallydamped = closed_loop_sys;
% *Set Damping Ratio more than 1 (Overdamped)*

% Definisikan parameter sistem dan PID
Kp = 27000;
Ki = 845.7057;
Kd = 2.1550e+05;

% Transfer function dari PID controller
pid_controller = pid(Kp, Ki, Kd);
closed_loop_sys = feedback(series(pid_controller, open_loop_tf), sensor_tf);

% rlocusplot(closed_loop_sys)
% grid on;
% title('Root Locus Overdamped Response');

% Closed-loop transfer function
G_cl = closed_loop_sys;

% Poles dari closed-loop system
poles = pole(G_cl);

% Hitung damping ratio dan natural frequency
[zeta, omega_n] = damp(G_cl);

% Tampilkan hasil
disp('Poles of the closed-loop system:');
disp(poles);
disp('Damping Ratio:');
disp(zeta);
disp('Natural Frequency:');
disp(omega_n);

% Identifikasi mode dominan (poles dengan bagian real terkecil)
[~, idx] = min(abs(real(poles)));
dominant_pole = poles(idx);
dominant_zeta = zeta(idx);
dominant_omega_n = omega_n(idx);

% Tampilkan mode dominan
disp('Dominant Pole:');
disp(dominant_pole);
disp('Dominant Damping Ratio:');
disp(dominant_zeta);
disp('Dominant Natural Frequency:');
disp(dominant_omega_n);
%%
% Analyze initial response
info = stepinfo(closed_loop_sys);
disp('Initial Step Response Information:');
disp(info);

% Iteratively adjust PID parameters
for i = 1:100
    % Analyze current response
    info = stepinfo(closed_loop_sys);
    
    % Adjust Kp, Ki, and Kd to increase damping and reduce overshoot
    if info.Overshoot > 0
        Kp = Kp * 0.7; % Decrease Kp by 30%
        Ki = Ki * 0.7; % Decrease Ki by 30%
        Kd = Kd * 0.9; % Decrease Kd by 10%

    else
        break; % Break the loop if overshoot is zero
    end
    
    % Create updated PID controller
    pid_controller = pid(Kp, Ki, Kd);
    
    % Closed-loop transfer function with updated PID
    closed_loop_sys = feedback(series(pid_controller, plant_tf), sensor_tf);
    
    % Step response with updated PID
    figure;
    step(closed_loop_sys);
    title(['Step Response with Updated PID (Iteration ', num2str(i), ')']);
    xlabel('Time (s)');
    ylabel('Output');
    grid on;
    
    % Analysis of updated response
    info = stepinfo(closed_loop_sys);
    disp(['Updated Step Response Information (Iteration ', num2str(i), '):']);
    disp(info);
    
    % Check if the system is overdamped
    if info.Overshoot == 0 && info.PeakTime > 0
        disp('System is overdamped.');
        break;
    end
    
    % Pause to observe the response before next iteration
    pause(2);
end

Overdamped = closed_loop_sys;
% Final Result

figure;
hold on;

Overdamped = 30.625*Overdamped;
Underdamped = 30.625*Underdamped;
Criticallydamped = 30.625*Criticallydamped;

% Overdamped Response
step(Overdamped);
% Underdamped Response
step(Underdamped);
% Critically Damped Response
step(Criticallydamped);

title('System Response with Different PID Tunings');
xlabel('Time (s)');
ylabel('Signal Analogy');
legend('Overdamped', 'Underdamped', 'Critically Damped');
hold off;

% Overdamped Response
step(Overdamped);
title('Overdamped Response');
xlabel('Time (s)');
ylabel('Signal Analogy');
hold off;
info = stepinfo(Overdamped);
disp(['Step Response Information:']);
disp(info);

rlocusplot(Overdamped)
grid on;
title('Root Locus Overdamped Response');


% Underdamped Response
step(Underdamped);
title('Underdamped Response');
xlabel('Time (s)');
ylabel('Signal Analogy');
hold off;
info = stepinfo(Underdamped);
disp(['Step Response Information:']);
disp(info);

rlocusplot(Underdamped)
grid on;
title('Root Locus Underdamped Response');

% Critically damped Response
step(Criticallydamped);
title('Critically damped Response');
xlabel('Time (s)');
ylabel('Signal Analogy');
hold off;
info = stepinfo(Criticallydamped);
disp(['Step Response Information:']);
disp(info);

rlocusplot(Criticallydamped)
grid on;
title('Root Locus Critically damped Response');

%%
% % Poles dari closed-loop system
% poles = pole(Criticallydamped);
% 
% % Hitung damping ratio dan natural frequency
% [zeta, omega_n] = damp(Criticallydamped);
% 
% % Tampilkan hasil
% disp('Poles of the closed-loop system:');
% disp(poles);
% disp('Damping Ratio:');
% disp(zeta);
% disp('Natural Frequency:');
% disp(omega_n);
% 
% % Identifikasi mode dominan (poles dengan bagian real terkecil)
% [~, idx] = min(abs(real(poles)));
% dominant_pole = poles(idx);
% dominant_zeta = zeta(idx);
% dominant_omega_n = omega_n(idx);
% 
% % Tampilkan mode dominan
% disp('Dominant Pole:');
% disp(dominant_pole);
% disp('Dominant Damping Ratio:');
% disp(dominant_zeta);
% disp('Dominant Natural Frequency:');
% disp(dominant_omega_n);
