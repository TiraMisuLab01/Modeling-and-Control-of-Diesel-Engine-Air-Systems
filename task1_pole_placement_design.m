% task1_pole_placement_design.m
% This script performs all steps for Task 1:
% 1. Defines system matrices A, B, C based on matriculation number.
% 2. Checks system controllability.
% 3. Determines desired closed-loop pole locations.
% 4. Calculates state feedback gain matrix K using the Unity Rank method.
% 5. Verifies pole placement.

clear; clc; close all; % 清除工作区、命令行窗口并关闭所有图形窗口

%% 步骤一：系统矩阵的确定与可控性检验
% --- 用户自定义参数 ---
% 学号的最后四位数字: a, b, c, d
a = 8;
b = 4;
c = 0;
d = 1;

fprintf('使用参数: a=%d, b=%d, c=%d, d=%d\n\n', a, b, c, d);

% 1. 确定系统矩阵 (A, B, C)：
% 根据学号参数 a, b, c, d，将其代入项目文件（Page 3，公式2）中给出的 A, B, C 矩阵的表达式。

% --- 矩阵 A 的定义 ---
A = zeros(4, 4);
A(1,1) = -8.8487 + (a - b) / 5;
A(1,2) = -0.0399;
A(1,3) = -5.5500 + (c * d + 5) / 10; % 使用您提供的公式
A(1,4) = 3.5846;

A(2,1) = -4.5740;
A(2,2) = 2.5010 * (d + 5) / (c + 5);
A(2,3) = -4.3662;
A(2,4) = -1.1183 - (a - c) / 20;

A(3,1) = 3.7698;
A(3,2) = 16.1212 - c / 5;
A(3,3) = -18.2103 + (a + d) / (b + 4);
A(3,4) = 4.4936;

A(4,1) = -8.5645 - (a - b) / (c + d + 2);
A(4,2) = 8.3742;
A(4,3) = -4.4331;
A(4,4) = -7.7181 * (c + 5) / (b + 5);

% --- 矩阵 B 的定义 ---
B = zeros(4, 2);
B(1,1) = 0.0564 + b / (10 + c); % 使用您提供的公式
B(1,2) = 0.0319;

B(2,1) = 0.0165 - (c + d - 5) / (1000 + 20 * a);
B(2,2) = -0.02;

B(3,1) = 4.4939;
B(3,2) = 1.5985 * (a + 10) / (b + 12);

B(4,1) = -1.4269;
B(4,2) = -0.2730;

% --- 矩阵 C 的定义 ---
C = zeros(2, 4);
C(1,1) = -3.2988;
C(1,2) = -2.1932 + (10 * c + d) / (100 + 5 * a);
C(1,3) = 0.0370;
C(1,4) = -0.0109;

C(2,1) = 0.2922 - (a * b) / 500;
C(2,2) = -2.1506;
C(2,3) = -0.0104;
C(2,4) = 0.0163;

% --- 显示矩阵 ---
disp('矩阵 A:');
disp(A);
disp('矩阵 B:');
disp(B);
disp('矩阵 C:');
disp(C);

% --- 创建状态空间模型 ---
sys = ss(A, B, C, 0);
disp('State-space model ''sys'' created successfully.');

% 2. 检验系统的可控性：构建系统的可控性矩阵 W_c，并计算其秩。
% 根据第7章（Page 21，定理1 / Slide Page），系统能够通过状态反馈任意配置极点的充要条件是系统是可控的。
% 如果系统不可控，那么我们将无法将所有极点移动到我们期望的位置，任务目标就无法实现。

n = size(A, 1); % 状态变量的数量 n = 4（四阶系统）
m = size(B, 2); % 输入的数量  m = 2（两个输入：u1: VGT（可变几何涡轮）的叶片位置，u2: EGR（废气再循环）的阀门位置）

Wc = ctrb(A, B); % 计算可控性矩阵 （[B, AB, A^2B, A^3B]）
rank_Wc = rank(Wc); % 计算可控性矩阵的秩

fprintf('\n可控性矩阵的秩: %d\n', rank_Wc);
if rank_Wc == n
    disp('系统是可控的。');
else
    disp('系统是不可控的。无法通过极点配置实现所有期望极点。');
    return; % 如果不可控，则退出脚本
end

%% --- 步骤二：确定期望的闭环极点位置 ---
% 将性能指标转换为S平面区域：利用二阶系统近似公式，将超调量和稳定时间的要求转换为S平面上极点必须满足的区域。
% 第7章（Page 11）提供了这些公式，它们是连接时域性能和频域（极点位置）特性的桥梁。

% 性能指标：
Mp_target = 0.10; % 目标超调量 < 10%
ts_target = 5;    % 目标2%稳定时间 < 5秒

% 1. 根据超调量计算最小阻尼比 (zeta_min)
zeta_min = -log(Mp_target) / sqrt(pi^2 + log(Mp_target)^2);  % 0.591
fprintf('根据超调量 Mp < %.1f%%, 所需最小阻尼比 zeta > %.3f\n', Mp_target*100, zeta_min);

% 2. 根据稳定时间计算最小极点实部 (sigma_min = zeta*omega_n)
sigma_min = 4 / ts_target;  % 0.800
fprintf('根据稳定时间 ts < %d 秒, 所需最小极点实部 |Re(lambda)| > %.3f\n', ts_target, sigma_min);

% 3. 选择期望的闭环极点
% 考虑到需要一定的超调，选择一对复数共轭主导极点
dominant_real_part = -1.5; % 满足 |Re(lambda)| > sigma_min (1.5 > 0.8)
dominant_imag_part = 1.5;  % 配合实部，使阻尼比满足要求

% 验证主导极点的阻尼比和自然频率
omega_n_dominant = sqrt(dominant_real_part^2 + dominant_imag_part^2);
zeta_dominant = abs(dominant_real_part) / omega_n_dominant;
fprintf('选择主导极点 Re(lambda) = %.1f, Im(lambda) = %.1f\n', dominant_real_part, dominant_imag_part);
fprintf('  -> 对应自然频率 omega_n = %.3f, 阻尼比 zeta = %.3f\n', omega_n_dominant, zeta_dominant);
if zeta_dominant > zeta_min
   fprintf('  -> 阻尼比满足 Mp 要求 (%.3f > %.3f)\n', zeta_dominant, zeta_min);
else
   fprintf('  -> 阻尼比不满足 Mp 要求 (%.3f <= %.3f), 可能需要调整极点位置\n', zeta_dominant, zeta_min);
end

% 选择非主导极点 (通常比主导极点快3-5倍)
% 主导极点实部为 -1.5，选择 -5.0 和 -6.0 作为非主导极点
non_dominant_1 = -5.0;
non_dominant_2 = -6.0;
fprintf('选择非主导极点: %.1f, %.1f (比主导极点快约3-4倍)\n', non_dominant_1, non_dominant_2);

P_desired = [dominant_real_part + dominant_imag_part*1j, ...
             dominant_real_part - dominant_imag_part*1j, ...
             non_dominant_1, ...
             non_dominant_2];  % [-1.5 + 1.5j, -1.5 - 1.5j, -5.0, -6.0]
fprintf('\n最终期望的闭环极点 P_desired:\n');
disp(P_desired);


%% --- 步骤三：计算反馈增益矩阵 K (使用单位秩方法) ---

% 3.1 选择权重向量 q:选择一个 m x 1（即 2 x 1）的非零向量 q，使得 (A, Bq) 对是可控的
% 第7章 §7.4.1 Unity Rank K (Page 50-56) 介绍了单位秩方法。它通过将多输入系统 (A, B) 转换为一个等效的单输入系统 (A,Bq)，
% 从而可以使用单输入系统的极点配置方法来求解。选择 q 的目的是确保这个等效的单输入系统仍然可控。

% 尝试 q = [1; 1] 以确保两个执行器都被利用
q = [1; 1]; 
fprintf('\n尝试权重向量 q = [%d; %d]\n', q(1), q(2));

% 计算 Bq
Bq = B * q;

% 检验 (A, Bq) 的可控性
Wc_siso = ctrb(A, Bq);
rank_Wc_siso = rank(Wc_siso);

fprintf('(A, Bq) 的可控性矩阵的秩: %d\n', rank_Wc_siso);
if rank_Wc_siso == n
    disp('(A, Bq) 是可控的。继续进行极点配置。');
    
    % 3.2 计算等效单输入反馈增益 k_siso
    % 使用 'acker' 函数进行极点配置
    k_siso = acker(A, Bq, P_desired);
    
    % 3.3 构建最终的MIMO反馈增益矩阵 K
    K = q * k_siso; % K = q * k_siso; 这里的 k_siso 是行向量，q 是列向量，所以 K 是列向量 * 行向量 = 矩阵
    
    fprintf('\n计算出的反馈增益矩阵 K:\n');
disp(K);
    
    % 验证极点配置是否成功 (检查 A-BK 的特征值)
    A_cl_check = A - B * K;
    fprintf('\nA-BK 的特征值 (闭环极点):\n');
    disp(eig(A_cl_check));
    
else
    disp('(A, Bq) 是不可控的。可能需要尝试不同的 q 向量。');
    % 如果 q = [1; 1] 失败，可以尝试 q = [1; 0] 或 q = [0; 1]
    K = []; 
end

%% 步骤四 ：仿真与性能分析
% 4.1 构建闭环系统模型
A_cl = A - B * K;
B_cl = B; % 假设参考输入 r 通过 B 矩阵进入系统 (u = -Kx + r)
C_cl = C;
D_cl = zeros(size(C,1), size(B,2)); % D矩阵为零

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);
fprintf('\n闭环系统模型 sys_cl 创建成功。\n');

% 4.2 阶跃响应仿真
fprintf('\n--- 阶跃响应仿真 ---\n');
figure;

% --- 错误修正：MIMO 的 'step' 函数语法 ---
% 我们一次性计算所有输入的响应，然后分别提取它们。
T_final_step = 20; % 仿真时间
[y_all, t_all, x_all] = step(sys_cl, T_final_step);

% 提取对输入1 (r = [1; 0]) 的响应
y_step1 = y_all(:, :, 1); % 维度: [time, 2 outputs]
t_step1 = t_all;

% 提取对输入2 (r = [0; 1]) 的响应
y_step2 = y_all(:, :, 2); % 维度: [time, 2 outputs]
t_step2 = t_all;
% (我们在这里不需要 x_step1 和 x_step2，所以省略了提取)
% --- 修正结束 ---


% 仿真第一个输入通道的阶跃响应 (r = [1; 0])
subplot(2,1,1);
% --- 错误修正：'plot' 函数的语法 ---
% 将 "t_step1" 字符串改为了 t_step1 变量
plot(t_step1, y_step1(:,1), "b", t_step1, y_step1(:,2), "r");
% 或者使用更简洁的： plot(t_step1, y_step1);
title('输出阶跃响应 (r = [1; 0])');
xlabel('时间 (秒)');
ylabel('输出');
legend('y1', 'y2');
grid on;

% 仿真第二个输入通道的阶跃响应 (r = [0; 1])
subplot(2,1,2);
% --- 错误修正：'plot' 函数的语法 ---
plot(t_step2, y_step2(:,1), "b", t_step2, y_step2(:,2), "r");
% 或者使用更简洁的： plot(t_step2, y_step2);
title('输出阶跃响应 (r = [0; 1])');
xlabel('时间 (秒)');
ylabel('输出');
legend('y1', 'y2');
grid on;

% 4.3 非零初始状态响应仿真 (这部分代码是正确的)
fprintf('\n--- 非零初始状态响应仿真 ---\n');
x0 = [0.5; -0.1; 0.3; -0.8]; % 项目指定的初始状态
t_sim = 0:0.01:20; % 仿真时间

% 使用 initial 函数仿真，外部输入为零
[y_ic, t_ic, x_ic] = initial(sys_cl, x0, t_sim); % 这里的 sys_cl 已经包含了 A_cl = A-BK

figure;
subplot(2,1,1);
plot(t_ic, x_ic);
title('状态变量响应 (非零初始状态, 零外部输入)');
xlabel('时间 (秒)');
ylabel('状态');
legend('x1', 'x2', 'x3', 'x4');
grid on;

% 4.4 控制信号幅值监测 (这部分代码是正确的)
% u = -Kx
u_ic = -K * x_ic'; % 计算每个时间点的控制信号
subplot(2,1,2);
plot(t_ic, u_ic);
title('控制信号 (非零初始状态, 零外部输入)');
xlabel('时间 (秒)');
ylabel('控制量');
legend('u1', 'u2');
grid on;

fprintf('\n仿真完成。请检查生成的图形以分析性能。\n');
