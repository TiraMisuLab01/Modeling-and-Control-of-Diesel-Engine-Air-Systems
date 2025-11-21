% EE5101/ME5401 Mini-project Task 5: Servo Control with Reduced-Order Observer
% Combines Servo Mechanism (Task 5) with Reduced-Order Observer (Task 3)

clear; clc; close all;
set(0, 'defaultAxesFontSize', 14); % Affects the scale and labels on the axes
set(0, 'defaultTextFontSize', 14);  % Affects text such as titles, axis labels, legends, etc.

%% 1. System Parameter Setup
% Replace with your Matriculation Number digits
a = 8; b = 4; c = 0; d = 1; 

% Helper calculations for system matrices
p1 = 2.5010 * (d + 5) / (c + 5);
p2 = 0.2922 - (a * b) / 500;
p3 = 0.0165 - (c + d - 5) / (1000 + 20 * a);

% Plant Matrices
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

[n, m_in] = size(B);
[m_out, ~] = size(C);

%% 2. Servo Controller Design (Augmented LQR)
% Augmented System: z = [x; v], v_dot = y_sp - y
A_aug = [A, zeros(n, m_out); 
         -C, zeros(m_out, m_out)];
B_aug = [B; zeros(m_out, m_in)];

% LQR Weights
Q_aug = diag([1, 1, 1, 1, 1000, 1000]); % Heavy penalty on integral states v
R_aug = 0.1 * eye(m_in);

% Calculate Gains
K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);
K_x = K_aug(:, 1:n);
K_v = K_aug(:, n+1:end);

fprintf('Servo Gains Calculated.\n');

%% 3. Reduced-Order Observer Design (Reused from Task 3)
% Reduced order
r_order = n - m_out; % 4 - 2 = 2

% Determine dominant pole of the closed-loop plant (A - B*K_x)
cl_poles_ctrl = eig(A - B*K_x);
dom_pole_ctrl = max(real(cl_poles_ctrl));
fprintf('Controller Dominant Pole: %.4f\n', dom_pole_ctrl);

% Observer Design Parameters (Recommended Case: 4x faster)
obs_poles = [dom_pole_ctrl * 4; dom_pole_ctrl * 4 - 0.5];
D_obs = diag(obs_poles);

% Solve Sylvester Equation: T*A - D*T = G*C
G_obs = ones(r_order, m_out); % Initial guess
T_obs = sylvester(A', -D_obs', C'*G_obs')';

% Check invertibility of [C; T]
if rcond([C; T_obs]) < 1e-10
    G_obs = eye(r_order, m_out); % Fallback
    T_obs = sylvester(A', -D_obs', C'*G_obs')';
end

% Calculate remaining matrices
M_obs = inv([C; T_obs]); % Reconstruction matrix
E_obs = T_obs * B;

fprintf('Reduced-Order Observer Designed (Poles: %.2f, %.2f)\n', obs_poles(1), obs_poles(2));

%% 4. Simulation
T_final = 15;
dt = 0.01;
time = 0:dt:T_final;
steps = length(time);

% Initial Conditions
x0 = [0.5; -0.1; 0.3; -0.8];
xi0 = [0; 0]; % Observer internal state initial condition
v0 = [0; 0];  % Integral state initial condition

% Set Points & Disturbance
y_sp = [0.4; 0.8];
t_dist = 10;
w_dist = [0.3; 0.2];

% Storage
X = zeros(n, steps); X(:,1) = x0;
Xi = zeros(r_order, steps); Xi(:,1) = xi0; % Reduced state
X_hat = zeros(n, steps); 
V = zeros(m_out, steps); V(:,1) = v0;
Y = zeros(m_out, steps);
U = zeros(m_in, steps);

% Initial Reconstruction
y_init = C * x0;
X_hat(:,1) = M_obs * [y_init; xi0];

for k = 1:steps-1
    % 1. Measurement
    y_meas = C * X(:,k);
    Y(:,k) = y_meas;
    
    % 2. Reconstruction of Full State
    % x_hat = M * [y; xi]
    x_hat_k = M_obs * [y_meas; Xi(:,k)];
    X_hat(:,k) = x_hat_k;
    
    % 3. Control Law (Servo + Observer)
    % u = -Kx * x_hat - Kv * v
    u_ctrl = -K_x * x_hat_k - K_v * V(:,k);
    U(:,k) = u_ctrl;
    
    % 4. Disturbance
    if time(k) >= t_dist
        w = w_dist;
    else
        w = zeros(m_in, 1);
    end
    u_plant = u_ctrl + w;
    
    % 5. Dynamics Integration (Euler)
    % Plant: dx = Ax + B(u+w)
    dX = A * X(:,k) + B * u_plant;
    X(:,k+1) = X(:,k) + dX * dt;
    
    % Observer: dxi = D*xi + E*u + G*y (Note: u is u_ctrl, not u_plant)
    dXi = D_obs * Xi(:,k) + E_obs * u_ctrl + G_obs * y_meas;
    Xi(:,k+1) = Xi(:,k) + dXi * dt;
    
    % Integrator: dv = y_sp - y
    dV = y_sp - y_meas;
    V(:,k+1) = V(:,k) + dV * dt;
end

% Final step values
Y(:,end) = C * X(:,end);
X_hat(:,end) = M_obs * [Y(:,end); Xi(:,end)];
U(:,end) = -K_x * X_hat(:,end) - K_v * V(:,end);

%% 5. Plotting
figure('Name', 'Task 5 (Reduced Observer) Results', 'Color', 'w');

% Outputs
subplot(3,1,1);
plot(time, Y(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, Y(2,:), 'r', 'LineWidth', 1.5);
yline(y_sp(1), 'b--', 'LineWidth', 1);
yline(y_sp(2), 'r--', 'LineWidth', 1);
xline(t_dist, 'k:', 'LineWidth', 1.2);
title('System Outputs vs Setpoints (Reduced Observer)');
legend('y_1', 'y_2', 'SP_1', 'SP_2', 'Disturbance');
grid on;

% Inputs
subplot(3,1,2);
plot(time, U(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, U(2,:), 'r', 'LineWidth', 1.5);
xline(t_dist, 'k:', 'LineWidth', 1.2);
title('Control Inputs');
legend('u_1', 'u_2');
grid on;

% Estimation Error
subplot(3,1,3);
err_norm = vecnorm(X - X_hat);
plot(time, err_norm, 'k', 'LineWidth', 1.5);
title('State Estimation Error Norm $||x - \hat{x}||$','Interpreter', 'latex');
grid on; xlabel('Time (s)');

fprintf('\nFinal Steady State Error (y - y_sp):\n');
disp(Y(:,end) - y_sp');