% task2_lqr_design.m
% This script performs all steps for Task 2:
% 1. Defines system matrices A, B, C.
% 2. Iteratively designs LQR controllers by adjusting weighting matrices Q and R.
% 3. Simulates and analyzes the performance of each design.

clear; clc; close all;

%% Step 1: System Definition
% --- User-specific parameters ---
a = 8; b = 4; c = 0; d = 1;
fprintf('Using parameters: a=%d, b=%d, c=%d, d=%d\n\n', a, b, c, d);

% --- Matrix Definitions (as per your correct version) ---
A = zeros(4, 4);
A(1,1) = -8.8487 + (a - b) / 5; A(1,2) = -0.0399; A(1,3) = -5.5500 + (c * d + 5) / 10; A(1,4) = 3.5846;
A(2,1) = -4.5740; A(2,2) = 2.5010 * (d + 5) / (c + 5); A(2,3) = -4.3662; A(2,4) = -1.1183 - (a - c) / 20;
A(3,1) = 3.7698; A(3,2) = 16.1212 - c / 5; A(3,3) = -18.2103 + (a + d) / (b + 4); A(3,4) = 4.4936;
A(4,1) = -8.5645 - (a - b) / (c + d + 2); A(4,2) = 8.3742; A(4,3) = -4.4331; A(4,4) = -7.7181 * (c + 5) / (b + 5);

B = zeros(4, 2);
B(1,1) = 0.0564 + b / (10 + c); B(1,2) = 0.0319;
B(2,1) = 0.0165 - (c + d - 5) / (1000 + 20 * a); B(2,2) = -0.02;
B(3,1) = 4.4939; B(3,2) = 1.5985 * (a + 10) / (b + 12);
B(4,1) = -1.4269; B(4,2) = -0.2730;

C = zeros(2, 4);
C(1,1) = -3.2988; C(1,2) = -2.1932 + (10 * c + d) / (100 + 5 * a); C(1,3) = 0.0370; C(1,4) = -0.0109;
C(2,1) = 0.2922 - (a * b) / 500; C(2,2) = -2.1506; C(2,3) = -0.0104; C(2,4) = 0.0163;

n = size(A, 1);
x0 = [0.5; -0.1; 0.3; -0.8];
t_sim = 0:0.01:20;

%% Step 2: LQR Design and Iteration
% We will test three different sets of Q and R to discuss their effects.

% --- Design 1: Baseline (Q = C'C, R = I) ---
fprintf('--- LQR Design 1: Baseline ---\n');
Q1 = C' * C;
R1 = eye(2);
K1 = lqr(A, B, Q1, R1);
sys_cl1 = ss(A - B * K1, B, C, 0);
fprintf('K1 (Q=C''C, R=I):\n');
disp(K1);

% --- Design 2: Aggressive Control (Increased Q/R ratio) ---
fprintf('\n--- LQR Design 2: Aggressive Control ---\n');
Q2 = 10 * (C' * C); % Penalize state error more
R2 = 0.1 * eye(2);  % Allow more control effort
K2 = lqr(A, B, Q2, R2);
sys_cl2 = ss(A - B * K2, B, C, 0);
fprintf('K2 (Q=10*C''C, R=0.1*I):\n');
disp(K2);

% --- Design 3: Conservative Control (Decreased Q/R ratio) ---
fprintf('\n--- LQR Design 3: Conservative Control ---\n');
Q3 = C' * C;
R3 = 10 * eye(2); % Penalize control effort more
K3 = lqr(A, B, Q3, R3);
sys_cl3 = ss(A - B * K3, B, C, 0);
fprintf('K3 (Q=C''C, R=10*I):\n');
disp(K3);

%% Step 3: Simulation and Performance Analysis

% --- Step Response Comparison ---
figure('Position', [100, 100, 700, 500]); % [left, bottom, width, height]
% --- Design 1 ---
subplot(3,1,1); 
step(sys_cl1, 10); 
grid on;
title(''); 
ylabel('Design 1 (Baseline)'); 
info1 = stepinfo(sys_cl1);
% --- Design 2 ---
subplot(3,1,2);
step(sys_cl2, 10); 
grid on;
title('');
ylabel('Design 2 (Aggressive)');
info2 = stepinfo(sys_cl2);
% --- Design 3 ---
subplot(3,1,3);
step(sys_cl3, 10); 
grid on;
title(''); 
ylabel('Design 3 (Conservative)');
info3 = stepinfo(sys_cl3);

% Adds a general title to the entire picture window
sgtitle('Step Response Comparison for Different Q/R Ratios');

% --- Display Performance Metrics ---
fprintf('\n--- Performance Metrics from Step Response ---\n');
fprintf('Design 1 (Baseline):\n');
fprintf('  Output 1: Overshoot = %.2f%%, SettlingTime = %.3f s\n', info1(1).Overshoot, info1(1).SettlingTime);
fprintf('  Output 2: Overshoot = %.2f%%, SettlingTime = %.3f s\n', info1(2).Overshoot, info1(2).SettlingTime);
fprintf('Design 2 (Aggressive):\n');
fprintf('  Output 1: Overshoot = %.2f%%, SettlingTime = %.3f s\n', info2(1).Overshoot, info2(1).SettlingTime);
fprintf('  Output 2: Overshoot = %.2f%%, SettlingTime = %.3f s\n', info2(2).Overshoot, info2(2).SettlingTime);
fprintf('Design 3 (Conservative):\n');
fprintf('  Output 1: Overshoot = %.2f%%, SettlingTime = %.3f s\n', info3(1).Overshoot, info3(1).SettlingTime);
fprintf('  Output 2: Overshoot = %.2f%%, SettlingTime = %.3f s\n', info3(2).Overshoot, info3(2).SettlingTime);

% --- Initial Condition Response & Control Effort Comparison ---
[~, ~, x_ic1] = initial(sys_cl1, x0, t_sim);
u_ic1 = -K1 * x_ic1';
[~, ~, x_ic2] = initial(sys_cl2, x0, t_sim);
u_ic2 = -K2 * x_ic2';
[~, ~, x_ic3] = initial(sys_cl3, x0, t_sim);
u_ic3 = -K3 * x_ic3';

figure;
% Plot state responses
subplot(3,2,1); plot(t_sim, x_ic1); title('State Response - Design 1'); legend('x1','x2','x3','x4'); grid on;
subplot(3,2,3); plot(t_sim, x_ic2); title('State Response - Design 2'); legend('x1','x2','x3','x4'); grid on;
subplot(3,2,5); plot(t_sim, x_ic3); title('State Response - Design 3'); xlabel('Time (s)'); legend('x1','x2','x3','x4'); grid on;

% Plot control signals
subplot(3,2,2); plot(t_sim, u_ic1); title('Control Signal - Design 1'); legend('u1','u2'); grid on;
subplot(3,2,4); plot(t_sim, u_ic2); title('Control Signal - Design 2'); legend('u1','u2'); grid on;
subplot(3,2,6); plot(t_sim, u_ic3); title('Control Signal - Design 3'); xlabel('Time (s)'); legend('u1','u2'); grid on;
sgtitle('Initial Condition Response and Control Effort Comparison');

fprintf('\nTask 2 simulation complete. Please analyze the figures.\n');
