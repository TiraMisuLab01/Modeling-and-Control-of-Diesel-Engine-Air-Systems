% EE5101/ME5401 Mini-project Task 5: Servo Control Design
% Multivariable Integral Control with Disturbance Rejection
% Based on Chapter 9 (Servo Control) & Chapter 11 (State Estimation)

clear; clc; close all;

%% 1. Parameter Setup (Replace with your Matriculation Number digits)
% Example: A0162903M -> a=2, b=9, c=0, d=3
a = 2; b = 9; c = 0; d = 3; 

% Define System Matrices A, B, C based on Mini-project eq(2)
% Helper calculations
p1 = 2.5010 * (d + 5) / (c + 5);
p2 = 0.2922 - (a * b) / 500;
p3 = 0.0165 - (c + d - 5) / (1000 + 20 * a);

A = [ a-b,                -4.5740,              -0.0399,              -5.5500 + (c+d)/10;
     -8.8487 + b/5,       3.7698,               16.1212 - c/5,        -4.3662;
     -18.2103 + (a+d)/(b+4), a-c,               -1.1183 - d/20,        3.5846;
     -8.5645 - (a-b)/(c+d+2), 8.3742,           -4.4331,              -7.7181 * (c+5)/(b+5)];

B = [ 0.0564 + b/(10+c),  0.0319;
      p3,                 4.4939;
     -1.4269,            -0.02;
      1.5985 * (a+10)/(b+12), -0.2730];

C = [ -3.2988, -2.1932 + (10*c+d)/(100+5*a), 0.0370, -0.0109;
       p2,     -2.1506,                     -0.0104, 0.0163];

D = zeros(2, 2); % Assuming D is zero matrix

[n_states, n_inputs] = size(B);
[n_outputs, ~] = size(C);

%% 2. Design Servo Controller (Augmented System LQR) - Task 5
% Reference: Chapter 9, Slide 60-62 (Augmented System)
% Augmented State vector: z = [x; v], where v is integral of error

% Construct Augmented System Matrices
% x_dot = A*x + B*u
% v_dot = r - y = r - C*x = -C*x + r (Reference r is external input)
% Augmented form: [x_dot; v_dot] = [A 0; -C 0]*[x; v] + [B; 0]*u

A_aug = [A, zeros(n_states, n_outputs); 
         -C, zeros(n_outputs, n_outputs)];

B_aug = [B; zeros(n_outputs, n_inputs)];

% Check Controllability of Augmented System (Chapter 9, Slide 61)
Co_aug = ctrb(A_aug, B_aug);
if rank(Co_aug) < (n_states + n_outputs)
    error('Augmented system is not controllable!');
end

% LQR Weights for Augmented System
% We penalize the integral states (v) heavily to ensure zero steady-state error
Q_aug = diag([1, 1, 1, 1, 1000, 1000]); % [x1, x2, x3, x4, v1, v2]
R_aug = 0.1 * eye(n_inputs);           % Penalize control effort

% Calculate LQR Gain for Augmented System
K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

% Split K_aug into Kx (state feedback) and Kv (integral gain)
K_x = K_aug(:, 1:n_states);
K_v = K_aug(:, n_states+1:end);

fprintf('Servo Controller Gains:\n');
disp('K_x:'); disp(K_x);
disp('K_v:'); disp(K_v);

%% 3. Design Observer (Task 3 Component) - Task 5 Requirement
% Reference: Chapter 11, Slide 30-33
% We use the observer to estimate x since we only have output y
% Observer poles should be faster than controller poles

% Let's use a simple pole placement or LQR for observer
% Using LQR for observer design (duality) is robust
Q_obs = 100 * eye(n_states); % Process noise covariance proxy
R_obs = 1 * eye(n_outputs);  % Measurement noise covariance proxy
L_t = lqr(A', C', Q_obs, R_obs);
L = L_t';

fprintf('Observer Gain L:\n');
disp(L);

%% 4. Simulation with Disturbance
% Setup Simulation Parameters
T_final = 40;
dt = 0.01;
time = 0:dt:T_final;
steps = length(time);

% Initial Conditions
x0 = [0.5; -0.1; 0.3; -0.8]; % Given in mini-project
x_hat0 = [0; 0; 0; 0];       % Initial estimate (usually zero)
v0 = [0; 0];                 % Initial integral error

% Set Point
y_sp = [0.4; 0.8];

% Disturbance Parameters (Step disturbance at t = 10s)
t_dist = 10;
w_dist = [0.3; 0.2]; % Input disturbance

% Initialize Arrays for Storage
X = zeros(n_states, steps);
X_hat = zeros(n_states, steps);
Y = zeros(n_outputs, steps);
U = zeros(n_inputs, steps);
V = zeros(n_outputs, steps); % Integral state

X(:,1) = x0;
X_hat(:,1) = x_hat0;
V(:,1) = v0;

% Simulation Loop (Euler Integration)
for k = 1:steps-1
    % 1. Measurement
    y_meas = C * X(:,k); % Assuming perfect measurement for now
    Y(:,k) = y_meas;
    
    % 2. Calculate Control Input (using Estimated State x_hat and Integral State v)
    % u = -Kx*x_hat - Kv*v
    u_control = -K_x * X_hat(:,k) - K_v * V(:,k);
    U(:,k) = u_control;
    
    % 3. Determine Disturbance
    if time(k) >= t_dist
        w = w_dist;
    else
        w = zeros(n_inputs, 1);
    end
    
    % Total input to plant
    u_plant = u_control + w;
    
    % 4. Update Plant State (True System)
    % dx/dt = Ax + B(u + w)
    dx = A * X(:,k) + B * u_plant;
    X(:,k+1) = X(:,k) + dx * dt;
    
    % 5. Update Observer State (Estimated System)
    % d(x_hat)/dt = A*x_hat + B*u_control + L(y - C*x_hat)
    % Note: Observer uses u_control (known), not u_plant (unknown disturbance)
    y_hat = C * X_hat(:,k);
    dx_hat = A * X_hat(:,k) + B * u_control + L * (y_meas - y_hat);
    X_hat(:,k+1) = X_hat(:,k) + dx_hat * dt;
    
    % 6. Update Integral State
    % dv/dt = y_sp - y
    % (Note: Slide 60 defines e = r - y, so we integrate error)
    error_signal = y_meas - y_sp; % Wait, slide 60 says e = r-y, v_dot = e. 
                                  % If u = -Kx - Kv*v, then v should accumulate error.
                                  % Let's stick to convention: v_dot = y - y_sp or y_sp - y?
                                  % If v_dot = y - y_sp, then u = -Kv*v is negative feedback.
                                  % If v_dot = y_sp - y, then u = -Kv*v needs sign check.
                                  % Standard augmented system: [x_dot; v_dot] = [A 0; -C 0]... + [0; I]r
                                  % implies v_dot = r - Cx = error.
                                  % So v accumulates (r - y).
    
    dv = y_sp - y_meas; % r - y
    V(:,k+1) = V(:,k) + dv * dt;
end

% Fill last values
Y(:,end) = C * X(:,end);
U(:,end) = -K_x * X_hat(:,end) - K_v * V(:,end);

%% 5. Plotting Results
figure('Name', 'Task 5: Servo Control Results', 'Color', 'w');

% Plot Outputs vs Setpoints
subplot(3,1,1);
plot(time, Y(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, Y(2,:), 'r', 'LineWidth', 1.5);
yline(y_sp(1), 'b--', 'LineWidth', 1.2);
yline(y_sp(2), 'r--', 'LineWidth', 1.2);
xline(t_dist, 'k:', 'LineWidth', 1);
title('System Outputs y_1, y_2 vs Setpoints');
legend('y_1 (AFR)', 'y_2 (EGR)', 'SP_1', 'SP_2', 'Disturbance Onset');
grid on;
ylabel('Outputs');

% Plot Control Inputs
subplot(3,1,2);
plot(time, U(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, U(2,:), 'r', 'LineWidth', 1.5);
xline(t_dist, 'k:', 'LineWidth', 1);
title('Control Inputs u_1, u_2');
legend('u_1 (VGT)', 'u_2 (EGR Valve)');
grid on;
ylabel('Inputs');

% Plot Estimation Error (Optional but good for verification)
subplot(3,1,3);
est_error = X - X_hat;
plot(time, vecnorm(est_error), 'k', 'LineWidth', 1.5);
title('Norm of State Estimation Error ||x - \hat{x}||');
grid on;
xlabel('Time (s)');
ylabel('Error Norm');

% Display Performance Metrics
fprintf('\nSteady State Values (Last 1 sec average):\n');
fprintf('y1: %.4f (Target: %.4f)\n', mean(Y(1, end-100:end)), y_sp(1));
fprintf('y2: %.4f (Target: %.4f)\n', mean(Y(2, end-100:end)), y_sp(2));