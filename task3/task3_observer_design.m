% task3_reduced_observer.m
% Task 3: Reduced-Order Observer Design and Investigation
% Based on Chapter 11 Slides 49-67
%
% Theoretical Basis:
% 1. System: dx/dt = Ax + Bu, y = Cx
% 2. Reduced State: xi = Tx (order n-m = 2)
% 3. Dynamics: d(xi)/dt = D*xi + E*u + G*y
% 4. Constraints: TA - DT = GC, E = TB
% 5. Reconstruction: x_hat = inv([C; T]) * [y; xi]

clear; clc; close all;
set(0, 'defaultAxesFontSize', 14); % Affects the scale and labels on the axes
set(0, 'defaultTextFontSize', 14);  % Affects text such as titles, axis labels, legends, etc.

%% 1. System Definition
% --- Parameters ---
a = 8; b = 4; c = 0; d = 1;
fprintf('Parameter: a=%d, b=%d, c=%d, d=%d\n', a, b, c, d);

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

n = 4; % State dimension
m = 2; % Output dimension
r_order = n - m; % Reduced observer order (4-2=2)

% --- LQR Controller (Baseline from Task 2) ---
Q_lqr = C' * C;
R_lqr = eye(2);
K = lqr(A, B, Q_lqr, R_lqr);

% Get Dominant Closed-loop Poles for Reference
cl_poles = eig(A - B*K);
dom_pole = max(real(cl_poles)); % Pole closest to imaginary axis
fprintf('LQR Dominant Pole Real Part: %.4f\n', dom_pole);

%% 2. Investigation Loop & Data Storage
% We first run all simulations and store data to facilitate complex plotting
multipliers = [1, 4, 15]; 
case_names = {'Slow (1x)', 'Recommended (4x)', 'Fast (15x)'};
colors = {'r', 'g', 'b'};

% Struct to store results for plotting later
results = struct(); 

x0 = [0.5; -0.1; 0.3; -0.8]; % Plant Initial Condition
xi0 = [0; 0];                % Observer Initial Condition
t_span = 0:0.01:8;           % Simulate long enough, but we will zoom in plots

for i = 1:length(multipliers)
    mult = multipliers(i);
    fprintf('Simulating Case %d: %s...\n', i, case_names{i});
    
    % --- Design (Same as before) ---
    p_obs = [dom_pole * mult; dom_pole * mult - 0.5]; 
    D = diag(p_obs); 
    G = ones(r_order, m); 
    T = sylvester(A', -D', C'*G')';
    
    % Robustness check for G
    if rcond([C; T]) < 1e-10
        G = eye(r_order, m);
        T = sylvester(A', -D', C'*G')';
    end
    
    M = inv([C; T]);
    N1 = M(:, 1:m); N2 = M(:, m+1:n);
    E = T * B;
    
    % --- Build System ---
    A_aug = [A - B*K*N1*C, -B*K*N2; 
             G*C - E*K*N1*C, D - E*K*N2];
    sys_cl = ss(A_aug, zeros(6,1), eye(6), 0);
    
    % --- Simulate ---
    [~, t, z] = initial(sys_cl, [x0; xi0], t_span);
    
    x_real = z(:, 1:4);
    xi_sim = z(:, 5:6);
    x_hat = zeros(length(t), 4);
    for k = 1:length(t)
        y_meas = C * x_real(k, :)';
        x_hat(k, :) = (M * [y_meas; xi_sim(k, :)'])';
    end
    
    % --- Store Results ---
    results(i).t = t;
    results(i).x_real = x_real;
    results(i).x_hat = x_hat;
    results(i).error_norm = vecnorm(x_real - x_hat, 2, 2);
    results(i).u = (-K * x_hat')';
    results(i).name = case_names{i};
    results(i).color = colors{i};
end

%% 3. Advanced Plotting (Optimized Visualization)

% --- Figure 1: Estimation Error (With Zoom-in Inset) ---
fig1 = figure('Name', 'Estimation Error with Zoom', 'Color', 'w');

% 1.1 Main Plot (0 to 4s)
ax1 = axes(fig1); 
hold(ax1, 'on');
for i = 1:3
    plot(ax1, results(i).t, results(i).error_norm, ...
        'Color', results(i).color, 'LineWidth', 1.5, 'DisplayName', results(i).name);
end
title(ax1, 'State Estimation Error Norm $||e(t)||$', 'Interpreter', 'latex', 'FontSize', 12);
xlabel(ax1, 'Time (s)'); ylabel(ax1, 'Error Norm');
xlim(ax1, [0, 4]); % Restrict main view to 0-4s as requested
grid(ax1, 'on');
legend(ax1, 'Location', 'Northeast');

% 1.2 Inset Plot (Magnify 0 to 0.6s)
% Position: [left bottom width height] (normalized 0-1)
ax_inset = axes('Position', [0.45 0.4 0.4 0.35]); 
box(ax_inset, 'on'); hold(ax_inset, 'on');
for i = 1:3
    plot(ax_inset, results(i).t, results(i).error_norm, ...
        'Color', results(i).color, 'LineWidth', 1.5);
end
xlim(ax_inset, [0, 0.6]); % Focus specifically on the initial peak
ylim(ax_inset, [0, 9]);   % Ensure the fast peak is visible
title(ax_inset, 'Zoom: 0 - 0.6s');
grid(ax_inset, 'on');
set(ax_inset, 'FontSize', 8);


% --- Figure 2: State Tracking (Separated Subplots) ---
fig2 = figure('Name', 'Separated State Tracking', 'Color', 'w');
sgtitle('State Tracking Performance (x_2) - Comparison', 'FontSize', 14);

for i = 1:3
    subplot(3, 1, i); hold on;
    
    % Plot Real vs Estimated
    plot(results(i).t, results(i).x_real(:,2), ...
        'Color', 'k', 'LineWidth', 1.5, 'DisplayName', 'Real x_2'); % Real is always Black
    plot(results(i).t, results(i).x_hat(:,2), ...
        'Color', results(i).color, 'LineStyle', '--', 'LineWidth', 2.0, ...
        'DisplayName', ['Est (' results(i).name ')']);
    
    % Styling
    ylabel('Amplitude');
    xlim([0, 3]); % Focus on the convergence phase (0-3s)
    grid on;
    legend('Location', 'Southeast');
    title(['Case ' num2str(i) ': ' results(i).name]);
    
    if i == 3
        xlabel('Time (s)');
    end
end


% --- Figure 3: Control Effort (Optional but recommended) ---
fig3 = figure('Name', 'Control Effort Comparison', 'Color', 'w');
hold on;
for i = 1:3
    plot(results(i).t, results(i).u(:,1), ...
        'Color', results(i).color, 'LineWidth', 1.5, 'DisplayName', results(i).name);
end
title('Control Input u_1 (Peaking Phenomenon Analysis)');
xlabel('Time (s)'); ylabel('u_1');
xlim([0, 3.5]); % Zoom in on the initial transient
grid on;
legend('Location', 'Best');

fprintf('\nVisualization updated based on your requirements.\n');