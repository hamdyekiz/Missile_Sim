close all 
clear all
clc
s = tf('s');

% Block diagram for the boost autopilot. 
% Servo = 75 / (s + 75);
% Servo2 = (s + 2) / (s);
% Airframe_thetadot = (s + 0.418) / (s^2 + 0.644*s + 86.4);
% Airframe_thetadot_gain = -106.47;
% Rate_Gyro = 1;
% G_s = Servo * Servo2 * Airframe_thetadot * Airframe_thetadot_gain;
% C_s = Rate_Gyro;

Servo = 75 / (s + 75);

Airframe_thetadot = (s + 1.2) / (s^2 + 1.27*s + 72.25);
Airframe_thetadot_gain = -469.6;

sys = Servo * Airframe_thetadot * Airframe_thetadot_gain;
% Rate_Gyro = 0.15;
% C_s = Rate_Gyro;

% sys = feedback(G_s,C_s,+1);
% sys
% sys = zpk(sys)

% Plots 
% Step Response
figure;
step(sys);
title('Step Response');

% Bode Plot
figure;
bode(sys);
grid on;
title('Bode Plot');

% Root Locus
figure;
rlocus(sys);
title('Root Locus');

% Nyquist Plot
figure;
nyquist(sys);
title('Nyquist Plot');

% Subplots
figure;

% Step Response
subplot(2, 2, 1);
step(sys);
title('Step Response');

% Bode Plot
subplot(2, 2, 2);
bode(sys);
grid on;
title('Bode Plot');

% Root Locus
subplot(2, 2, 3);
rlocus(sys);
title('Root Locus');

% Nyquist Plot
subplot(2, 2, 4);
nyquist(sys);
title('Nyquist Plot');

% Poles of the System
disp('Poles of the System:');
pole(sys)

[Gm, Pm, Wcg, Wcp] = margin(sys);
disp(['Gain Margin: ', num2str(Gm)])
disp(['Phase Margin: ', num2str(Pm)])
