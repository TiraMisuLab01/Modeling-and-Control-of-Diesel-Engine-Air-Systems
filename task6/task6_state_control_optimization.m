% task6_state_regulation.m
% Task 6: State Regulation and Optimization
% 
% Objectives:
% 1. Check feasibility of regulating states to a specific setpoint x_sp.
% 2. If infeasible, find the optimal steady state x_s* that minimizes J(x_s).
% 3. Design a controller to stabilize the system at x_s*.
%
% Theoretical References:
% - Equilibrium Analysis: Chapter 7 (Introduction)
% - Optimization (Quadratic Cost): Related to LQR concepts in Chapter 8
% - Tracking/Regulation Control Law: Chapter 9

clear; clc; close all;

%% 1. System Definition and Parameters
% --- User-specific parameters (a=8, b=4, c=0, d=1) ---
a = 8; b = 4; c = 0; d = 1;
fprintf('Task 6: Using parameters a=%d, b=%d, c=%d, d=%d\n', a, b, c, d);

% --- Matrix Definitions ---
A = [-8.0487, -0.0399, -5.0500,  3.5846;
     -4.5740,  3.0012, -4.3662, -1.5183;
      3.7698, 16.1212, -17.0853, 4.4936;
     -9.8978,  8.3742, -4.4331, -4.2878];

B = [ 0.4564,  0.0319;
      0.0199, -0.0200;
      4.4939,  1.7983;
     -1.4269, -0.2730];

C = [-3.2988, -2.1861,  0.0370, -0.0109;
      0.2282, -2.1506, -0.0104,  0.0163];

% --- Target State Setpoint (x_sp) ---
x_sp = [0; 0.5; -0.4; 0.3];

% --- Weight Matrix W ---
% Formula: W = diag(a+1, b+1, c+1, d+1)
W = diag([a+1, b+1, c+1, d+1]);

fprintf('\n--- Target Setpoint x_sp ---\n');
disp(x_sp);
fprintf('--- Weight Matrix W ---\n');
disp(W);


%% 2. Task 6(a): Feasibility Analysis
% Theory: To maintain a steady state x_ss, we must satisfy: 0 = A*x_ss + B*u_ss
% Therefore, B*u_ss = -A*x_sp must hold for x_sp to be a feasible steady state.
% This is a linear equation Ax = b type problem.

fprintf('\n=== Task 6(a): Feasibility Check ===\n');

% Vector required to be in the column space of B
target_vector = -A * x_sp; 

% Check solvability using ranks
rank_B = rank(B);
rank_augmented = rank([B, target_vector]);

fprintf('Rank of B: %d\n', rank_B);
fprintf('Rank of Augmented [B, -A*x_sp]: %d\n', rank_augmented);

if rank_B == rank_augmented
    fprintf('Result: Feasible. The system CAN stay exactly at x_sp.\n');
    x_target = x_sp; % We can aim for the exact target
else
    fprintf('Result: Infeasible. The system CANNOT stay exactly at x_sp.\n');
    fprintf('Reason: -A*x_sp is not in the column space of B (Overdetermined system).\n');
end


%% 3. Task 6(b): Optimization (Find Optimal Steady State)
% Problem: Minimize J = 0.5 * (x_s - x_sp)' * W * (x_s - x_sp)
% Subject to: A*x_s + B*u_s = 0  =>  x_s = -inv(A)*B * u_s
% Let M = -inv(A)*B. Then x_s = M * u_s.
% Cost J(u_s) = 0.5 * (M*u_s - x_sp)' * W * (M*u_s - x_sp)
% This is a weighted least squares problem.

fprintf('\n=== Task 6(b): Optimization ===\n');

% Mapping matrix from input u to steady-state x
if det(A) == 0
    error('Matrix A is singular. Cannot invert A to solve steady state map.');
end
M = -inv(A) * B; 

% Solve for optimal input u_s*
% Analytical solution for min ||M*u - x_sp||_W:
% u_s* = inv(M' * W * M) * M' * W * x_sp
H_matrix = M' * W * M;
f_vector = M' * W * x_sp;
u_optimal = H_matrix \ f_vector;

% Calculate optimal steady state x_s*
x_optimal = M * u_optimal;

% Calculate final cost J
error_vec = x_optimal - x_sp;
J_min = 0.5 * error_vec' * W * error_vec;

fprintf('Optimal Steady-State Input u_s*:\n');
disp(u_optimal);
fprintf('Optimal Steady-State State x_s*:\n');
disp(x_optimal);
fprintf('Minimum Cost Function Value J(x_s*): %.6f\n', J_min);


%% 4. Controller Design
% We need a controller to drive the system to x_s*.
% Control Law: u(t) = u_optimal - K * (x(t) - x_optimal)
% This is State Feedback with Feedforward (integral action not needed if model is perfect).
% We can reuse the LQR gain from Task 2 or design a new one. Let's design a robust one.

Q_design = 10 * eye(4);
R_design = 1 * eye(2);
K = lqr(A, B, Q_design, R_design);

fprintf('\n=== Controller Design ===\n');
fprintf('Feedback Gain K (LQR design):\n');
disp(K);

%% 5. Simulation Verification
fprintf('\n=== Simulation ===\n');

% Initial Condition (from Project description)
x0 = [0.5; -0.1; 0.3; -0.8];

% Simulation settings
dt = 0.01;
T_final = 20;
time = 0:dt:T_final;
steps = length(time);

X = zeros(4, steps);
X(:,1) = x0;
U = zeros(2, steps);

% Simulation Loop
for k = 1:steps-1
    current_x = X(:,k);
    
    % Control Law: Regulator + Feedforward
    % u = u_ff - K*(x - x_ref)
    current_u = u_optimal - K * (current_x - x_optimal);
    U(:,k) = current_u;
    
    % System Dynamics (Euler Integration)
    dx = A * current_x + B * current_u;
    X(:,k+1) = current_x + dx * dt;
end
% Last input
U(:,end) = u_optimal - K * (X(:,end) - x_optimal);

%% 6. Plotting Results

figure('Name', 'Task 6: State Regulation Results', 'Color', 'w', 'Position', [100, 100, 1000, 600]);

% Plot States
for i = 1:4
    subplot(2, 2, i);
    plot(time, X(i,:), 'LineWidth', 2); hold on;
    yline(x_sp(i), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Original Target x_{sp}');
    yline(x_optimal(i), 'g-.', 'LineWidth', 1.5, 'DisplayName', 'Optimized Steady State x_s^*');
    
    title(['State x_', num2str(i)]);
    xlabel('Time (s)');
    ylabel('Amplitude');
    grid on;
    if i == 1
        legend('Response', 'Original Target', 'Optimized Target', 'Location', 'Best');
    end
end

sgtitle('Task 6: State Regulation Optimization Results');

% Plot Control Inputs
figure('Name', 'Task 6: Control Inputs', 'Color', 'w');
subplot(2,1,1);
plot(time, U(1,:), 'LineWidth', 1.5); hold on;
yline(u_optimal(1), 'g--', 'LineWidth', 1.5);
title('Control Input u_1');
ylabel('Amplitude'); grid on;
legend('u_1(t)', 'Optimal u_{s1}');

subplot(2,1,2);
plot(time, U(2,:), 'LineWidth', 1.5); hold on;
yline(u_optimal(2), 'g--', 'LineWidth', 1.5);
title('Control Input u_2');
ylabel('Amplitude'); grid on;
xlabel('Time (s)');
legend('u_2(t)', 'Optimal u_{s2}');

fprintf('\nTask 6 Complete. Check figures for convergence analysis.\n');