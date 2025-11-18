% task3_observer_design.m
% This script performs all steps for Task 3:
% 1. Defines the system and the chosen LQR controller from Task 2.
% 2. Checks system observability.
% 3. Designs a full-state observer using pole placement.
% 4. Simulates the observer-based control system to analyze performance.

clear; clc; close all;

%% Step 1: System and Controller Definition
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

% --- Use the selected LQR controller from Task 2 (Baseline Design) ---
Q = C' * C;
R = eye(2);
K = lqr(A, B, Q, R);
fprintf('Using the LQR gain K from the baseline design in Task 2:\n');
disp(K);

%% Step 2: Observability Check
% Before designing an observer, we must check if the system is observable.
Wo = obsv(A, C); % Compute the observability matrix
rank_Wo = rank(Wo);

fprintf('\nRank of Observability Matrix: %d\n', rank_Wo);
if rank_Wo == n
    disp('System is observable. An observer can be designed.');
else
    disp('System is NOT observable. Cannot design a full-state observer.');
    return;
end

%% Step 3: Observer Design
% We need to place the observer poles to be faster than the controller poles.

% First, find the controller poles (eigenvalues of A-BK)
controller_poles = eig(A - B * K);
fprintf('\nController poles (from LQR design):\n');
disp(controller_poles);

% Select observer poles to be 3-5 times faster than the dominant controller pole.
% The dominant controller pole is the one with the largest real part (closest to zero).
dominant_pole_real_part = max(real(controller_poles));
observer_pole_real_part = 3 * dominant_pole_real_part; % Make them 3 times faster

% Place all four observer poles at this location for simplicity.
P_obs_desired = [observer_pole_real_part, observer_pole_real_part-0.1, observer_pole_real_part-0.2, observer_pole_real_part-0.3];
fprintf('\nDesired observer poles (placed to be faster than controller poles):\n');
disp(P_obs_desired');

% Calculate the observer gain L using the duality principle (place on A' and C')
% L = place(A', C', P_obs_desired)'
L = acker(A', C', P_obs_desired)'; % Using acker for robustness
fprintf('\nCalculated Observer Gain L:\n');
disp(L);

%% Step 4: Simulation of the Observer-Based Control System

% We construct an 8th-order augmented system to simulate the plant and observer together.
% The state of this augmented system is [x; e], where e = x - x_hat is the estimation error.
% The dynamics are:
% dx/dt = (A - BK)x + BK*e
% de/dt = (A - LC)e

A_aug = [A - B * K, B * K;
         zeros(n, n), A - L * C];

B_aug = [B;
         zeros(n, size(B,2))];

C_aug = [C, zeros(size(C,1), n)];

sys_aug = ss(A_aug, B_aug, C_aug, 0);

% Simulate the response from a non-zero initial state for the plant (x0)
% and zero initial error (e0 = x0 - x_hat(0), assuming x_hat(0) = 0).
x0_plant = [0.5; -0.1; 0.3; -0.8];
e0 = x0_plant; % Since x_hat(0) is zero
x0_aug = [x0_plant; e0];

t_sim = 0:0.01:10;
[y_obs, t_obs, x_aug] = initial(sys_aug, x0_aug, t_sim);

% Extract the plant states and estimation errors
x_plant = x_aug(:, 1:n);
e_error = x_aug(:, n+1:2*n);
x_hat = x_plant - e_error; % Reconstruct the estimated state

% --- Plotting the results ---
figure;

% 1. Plot the state estimation error
subplot(2,1,1);
plot(t_obs, e_error);
title('State Estimation Error (e = x - x_{hat})');
xlabel('Time (s)');
ylabel('Error');
legend('e1', 'e2', 'e3', 'e4');
grid on;

% 2. Plot the actual states vs. estimated states
subplot(2,1,2);
plot(t_obs, x_plant, '-', t_obs, x_hat, '--');
title('Actual States (solid) vs. Estimated States (dashed)');
xlabel('Time (s)');
ylabel('State Values');
legend('x1','x2','x3','x4', 'x1_{hat}','x2_{hat}','x3_{hat}','x4_{hat}');
grid on;
sgtitle('Observer Performance Analysis');

% 3. Compare output response with ideal state feedback vs. observer-based feedback
sys_ideal = ss(A - B * K, zeros(n,1), C, 0); % Ideal system with u=-Kx
[y_ideal, t_ideal] = initial(sys_ideal, x0_plant, t_sim);

figure;
plot(t_ideal, y_ideal, 'b', t_obs, y_obs, 'r--');
title('Output Response Comparison');
xlabel('Time (s)');
ylabel('Output y');
legend('y (Ideal State Feedback)', 'y (Observer-Based Feedback)');
grid on;

fprintf('\nTask 3 simulation complete. Please analyze the figures.\n');
