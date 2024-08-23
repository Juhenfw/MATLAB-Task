% Define the time vector
t = -4:0.01:4;

% Define the linear functions
f1 = t + 2;
f2 = -t + 2;

% Define the step functions
s1 = (t >= -2) - (t >= 0);
s2 = (t >= 0) - (t >= 2);

% Plot the step functions
figure;
plot(t, s1, t, s2)
grid on

% Apply the step functions to the linear functions
f1 = f1 .* s1;
hold on
plot(t, f1, 'LineWidth', 2)

f2 = f2 .* s2;
plot(t, f2, 'LineWidth', 2)

% Sum the signals
x = f1 + f2;
plot(t, x, 'LineWidth', 2)
grid on

% Label the axes and add a title
xlabel('time')
title('Sinyal Segitiga')

% Display the figure
hold off
