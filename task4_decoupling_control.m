%% Linear System Mini-project Task 4: Decoupling Control
% Student ID Parameters: a=8, b=4, c=0, d=1
clear; clc; close all;

%% 1. System Definition (系统定义)
% Using the matrices provided by the user
% 使用用户提供的矩阵数值
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
% 项目给定的初始条件
x0 = [0.5; -0.1; 0.3; -0.8];

%% 2. Check Relative Degree & Decouplability (检查相对阶和解耦性)
[p, m] = size(C); % p outputs, m inputs (p=2, m=2)
sigma = zeros(p, 1); % Store relative degree for each output (存储相对阶)

% Calculating relative degree sigma_i for each row i
% 计算每一行的相对阶 sigma_i
for i = 1:p
    c_i = C(i, :);
    if norm(c_i * B) > 1e-10 % Check if c_i*B is non-zero (检查是否非零)
        sigma(i) = 1;
    elseif norm(c_i * A * B) > 1e-10
        sigma(i) = 2;
    elseif norm(c_i * A^2 * B) > 1e-10
        sigma(i) = 3;
    else
        sigma(i) = 4; % Should not happen for this valid system
    end
end

fprintf('Relative degrees (相对阶): sigma_1 = %d, sigma_2 = %d\n', sigma(1), sigma(2));

% Constructing the Decoupling Matrix B_star
% 构建解耦矩阵 B*
B_star = zeros(p, m);
for i = 1:p
    B_star(i, :) = C(i, :) * A^(sigma(i)-1) * B;
end

% Check singularity
% 检查奇异性
if det(B_star) == 0
    error('B* is singular. Decoupling is not possible by static state feedback.');
else
    fprintf('Det(B*) = %f. System is decouplable.\n', det(B_star));
end

%% 3. Controller Design (Theorem 2) (控制器设计)
% We need to choose stable polynomials phi_i(s) for each channel.
% Stability is required. Roots should be in LHP.
% 我们需要为每个通道选择稳定的多项式 phi_i(s)。必须保证稳定性，根在左半平面。

% Desired poles for decoupling (Expected settling time < 20s)
% 期望的解耦极点 (期望调节时间 < 20s)
% Let's choose poles at -2, -3, etc. to be safe and fast enough.
poles_ch1 = [-2];       % For sigma_1 = 1 (Order 1)
poles_ch2 = [-3];       % For sigma_2 = 1 (Order 1)
% Note: If sigma was 2, we would need 2 poles, e.g., [-2, -3].
% Since both sigmas are likely 1 here, we just need 1 pole each.
% 注意：如果 sigma 是 2，我们需要 2 个极点。这里大概率 sigma 都是 1。

% Define polynomial matrices phi(A)
% 定义多项式矩阵 phi(A)
% For sigma=1, phi(s) = s + lambda -> phi(A) = A + lambda*I
Phi_A_1 = A - poles_ch1(1) * eye(4); % Corresponds to (s - (-2)) = s+2
Phi_A_2 = A - poles_ch2(1) * eye(4); % Corresponds to (s - (-3)) = s+3

% Construct C_star_star (Matrix containing phi(A))
% 构建 C** 矩阵
C_star_star = zeros(p, 4);
C_star_star(1, :) = C(1, :) * Phi_A_1;
C_star_star(2, :) = C(2, :) * Phi_A_2;

% Calculate Gains F and K
% 计算增益 F 和 K
inv_B_star = inv(B_star);
F = inv_B_star;            % Feedforward Gain (前馈增益)
K = inv_B_star * C_star_star; % Feedback Gain (反馈增益)

fprintf('\nController Gains:\n');
disp('F = '); disp(F);
disp('K = '); disp(K);

%% 4. Simulation & Verification (仿真与验证)
% Closed-loop system matrices
% 闭环系统矩阵
A_cl = A - B*K;
B_cl = B*F;
C_cl = C;
D_cl = zeros(2,2);

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

% 4.1 Step Response (Verifying Decoupling)
% 4.1 阶跃响应 (验证解耦)
figure('Name', 'Task 4: Decoupling Step Response');
step(sys_cl);
title('Step Response of Decoupled System');
% The off-diagonal plots (Input 1 -> Output 2, Input 2 -> Output 1) should be zero.
% 非对角线的图 (输入1->输出2, 输入2->输出1) 应该为零。

% 4.2 Internal Stability Check (Initial Response)
% 4.2 内部稳定性检查 (零输入响应)
figure('Name', 'Task 4: Internal Stability (Initial Response)');
initial(sys_cl, x0);
title(['Initial Response (x0 = [', num2str(x0'), '])']);
grid on;
% All states should converge to zero.
% 所有状态应收敛至零。

%% 5. Stability Analysis (稳定性分析)
cl_eigenvalues = eig(A_cl);
fprintf('\nClosed-loop Eigenvalues (闭环特征值):\n');
disp(cl_eigenvalues);

if all(real(cl_eigenvalues) < 0)
    fprintf('Result: The decoupled system is INTERNALLY STABLE.\n');
else
    fprintf('Result: The decoupled system is UNSTABLE (Hidden unstable modes).\n');
end