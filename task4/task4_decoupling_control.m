%% Linear System Mini-project Task 4: Decoupling Control Analysis
% Student ID Parameters: a=8, b=4, c=0, d=1
clear; clc; close all;
set(0, 'defaultAxesFontSize', 14); % Affects the scale and labels on the axes
set(0, 'defaultTextFontSize', 14);  % Affects text such as titles, axis labels, legends, etc.

%% 1. System Definition 
% Using the matrices provided by the user
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

D = zeros(2, 2);

% Initial condition given in the project
x0 = [0.5; -0.1; 0.3; -0.8];

%% 2. Check Relative Degree & Decouplability (检查相对阶和解耦性)
[p, n] = size(C); % p = outputs, n = states
[~, m] = size(B); % m = inputs

sigma = zeros(p, 1); % Store relative degree for each output

% Calculating relative degree sigma_i for each row i
for i = 1:p
    c_i = C(i, :);
    % We check if the row vector c_i * A^k * B is non-zero
    if norm(c_i * B) > 1e-10 
        sigma(i) = 1;
    elseif norm(c_i * A * B) > 1e-10
        sigma(i) = 2;
    elseif norm(c_i * A^2 * B) > 1e-10
        sigma(i) = 3;
    else
        sigma(i) = 4; 
    end
end

fprintf('Relative degrees (相对阶): sigma_1 = %d, sigma_2 = %d\n', sigma(1), sigma(2));

% Constructing the Decoupling Matrix B_star
B_star = zeros(p, m);
for i = 1:p
    B_star(i, :) = C(i, :) * A^(sigma(i)-1) * B;
end

% Check singularity
if abs(det(B_star)) < 1e-10
    error('B* is singular. Decoupling is not possible by static state feedback.');
else
    fprintf('Det(B*) = %f. System is decouplable.\n', det(B_star));
end

%% 3. Controller Design (Theorem 2) (控制器设计)
% Design stable polynomials phi_i(s) for each channel.
% Desired poles for decoupling (Expected settling time < 20s)
% 期望的解耦极点。为了保证稳定且快速，选择 -2 和 -3。
poles_ch1 = [-2];       % For sigma_1 = 1 (Order 1)
poles_ch2 = [-3];       % For sigma_2 = 1 (Order 1)

% Define polynomial matrices phi(A)
% phi(s) = s + lambda -> phi(A) = A + lambda*I
Phi_A_1 = A - poles_ch1(1) * eye(n); 
Phi_A_2 = A - poles_ch2(1) * eye(n); 

% Construct C_star_star (Matrix containing phi(A))
C_star_star = zeros(p, n);
C_star_star(1, :) = C(1, :) * Phi_A_1;
C_star_star(2, :) = C(2, :) * Phi_A_2;

% Calculate Gains F and K
inv_B_star = inv(B_star);
F = inv_B_star;               % Feedforward Gain
K = inv_B_star * C_star_star; % Feedback Gain

fprintf('\nController Gains:\n');
disp('F = '); disp(F);
disp('K = '); disp(K);

%% 4. Simulation & Verification (仿真与验证)
% Closed-loop system matrices
A_cl = A - B*K;
B_cl = B*F;
C_cl = C;
D_cl = zeros(2,2);

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

% 4.1 Step Response (Verifying Decoupling) - Manual Plotting
% 4.1 阶跃响应 (手动绘图以自定义标签)
figure('Name', 'Task 4: Decoupling Step Response');

% Get step response data: Y is (Time x Outputs x Inputs)
[Y_step, T_step] = step(sys_cl);

% Subplot 1: Input 1 -> Output 1
subplot(2, 2, 1);
plot(T_step, Y_step(:, 1, 1), 'LineWidth', 1.5);
title('From r_1 to y_1');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% Subplot 2: Input 2 -> Output 1 (Should be 0)
subplot(2, 2, 2);
plot(T_step, Y_step(:, 1, 2), 'LineWidth', 1.5);
title('From r_2 to y_1 (Decoupling Check)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% Subplot 3: Input 1 -> Output 2 (Should be 0)
subplot(2, 2, 3);
plot(T_step, Y_step(:, 2, 1), 'LineWidth', 1.5);
title('From r_1 to y_2 (Decoupling Check)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% Subplot 4: Input 2 -> Output 2
subplot(2, 2, 4);
plot(T_step, Y_step(:, 2, 2), 'LineWidth', 1.5);
title('From r_2 to y_2');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% 4.2 Internal Stability Check (Initial Response)
figure('Name', 'Task 4: Internal Stability (Initial Response)');
[Y_init, T_init, X_init] = initial(sys_cl, x0);
subplot(2,1,1);
plot(T_init, Y_init, 'LineWidth', 1.5);
title('Output Response to Initial Condition (y)');
legend('y_1', 'y_2');
xlabel('Time (s)');
ylabel('Output Amplitude');
grid on;

subplot(2,1,2);
plot(T_init, X_init, 'LineWidth', 1.5);
title('State Response to Initial Condition (x) - Check for Divergence');
legend('x_1', 'x_2', 'x_3', 'x_4');
xlabel('Time (s)');
ylabel('State Amplitude');
grid on;

%% 5. Stability & Observability Analysis (稳定性和可观测性分析)
cl_eigenvalues = eig(A_cl);
fprintf('\nClosed-loop Eigenvalues (闭环特征值):\n');
disp(cl_eigenvalues);

% Check observability of the unstable modes
% 检查不稳定模态的可观测性
[U, S, V] = svd(obsv(A_cl, C_cl));
rank_obsv = rank(obsv(A_cl, C_cl));
fprintf('Rank of Observability Matrix: %d (Full rank is %d)\n', rank_obsv, n);

if all(real(cl_eigenvalues) < -1e-5)
    fprintf('Result: The decoupled system is INTERNALLY STABLE.\n');
else
    fprintf('Result: The decoupled system is UNSTABLE.\n');
    fprintf('Reason: Unstable Pole-Zero Cancellation (Hidden Unstable Modes).\n');
    unstable_poles = cl_eigenvalues(real(cl_eigenvalues) >= 0);
    fprintf('Unstable Pole(s): \n');
    disp(unstable_poles);
end