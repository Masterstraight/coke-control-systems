clear; clc; close all;

% =========================================================
% Problem Title  : Conveyor Belt Speed PID Control
% Plant          : DC-motor-driven bottle conveyor
%                  (24,000 bph Coca-Cola filling line)
% Controller     : PI  (Kd = 0), IMC lambda-tuning
% Difficulty     : EASY
% Plant TF       : G(s) = K / (tau*s + 1)
%                  K   = 2.5   (m/s)/V   steady-state gain
%                  tau = 0.8   s         mechanical time constant
% Requirements   : Setpoint      = 1.2 m/s
%                  Settling time < 3 s  (2%)
%                  Overshoot     < 10 %
%                  Steady-state error = 0
%                  Reject 0.3 V input-disturbance step within ~2 s
% =========================================================

% ---- plots/ folder auto-creation ----------------------------------------
script_dir = fileparts(mfilename('fullpath'));
plot_dir   = fullfile(script_dir, 'plots');
if ~exist(plot_dir, 'dir'); mkdir(plot_dir); end

% =========================================================================
% Plant definition
% =========================================================================
K   = 2.5;               % (m/s)/V  steady-state gain
tau = 0.8;               % s        mechanical time constant
G   = tf(K, [tau, 1]);

% =========================================================================
% IMC (lambda) tuning
%   Closed-loop time constant: lambda = 0.3 s
%   Kp = tau / (K * lambda)
%   Ki = Kp / tau
% =========================================================================
lambda = 0.3;            % s  desired closed-loop time constant
Kp     = tau / (K * lambda);
Ki     = Kp / tau;
Kd     = 0;
C      = pid(Kp, Ki);   % PI controller (no derivative)

% =========================================================================
% Closed-loop transfer functions
% =========================================================================
T_yr = feedback(C*G, 1);      % reference -> output
T_yd = feedback(G,   C);      % input disturbance -> output

% =========================================================================
% Simulation
% =========================================================================
SP  = 1.2;               % setpoint  (m/s)
D   = 0.3;               % disturbance amplitude (V)
t   = 0:0.01:8;          % time vector (s)
y   = step(SP * T_yr, t);
yd  = step(D  * T_yd, t);

% =========================================================================
% Step-info metrics (computed on the full-amplitude closed-loop response)
% =========================================================================
info = stepinfo(SP * T_yr);

% =========================================================================
% Colour palette  (CLAUDE.md conventions)
% =========================================================================
coke_red  = [0.86 0.08 0.24];   % primary signal
proc_blue = [0.0  0.45 0.74];   % comparison signal
band_gray = [0.3  0.3  0.3 ];   % +/- 2% settling band

% =========================================================================
% Figure 1 -- Step Response
% =========================================================================
fig1 = figure('Position', [100 100 900 500], 'Color', 'w');

h1 = plot(t, y, 'Color', coke_red, 'LineWidth', 2);
hold on;  grid on;

% Setpoint line with text label on the line
h2 = yline(SP, '--k', 'Setpoint');

% +/- 2 % settling band (dotted, neutral gray)
h3    = yline(SP * 1.02, ':');
h3_lo = yline(SP * 0.98, ':');
h3.Color             = band_gray;
h3_lo.Color          = band_gray;
h3_lo.HandleVisibility = 'off';   % single legend entry for both band lines

xlabel('Time (s)');
ylabel('Belt Speed (m/s)');
title('Conveyor Speed -- PID Step Response to 1.2 m/s');
legend([h1, h2, h3], {'Speed', 'Setpoint', '+/- 2% band'}, ...
       'Location', 'southeast');

saveas(fig1, fullfile(plot_dir, 'problem1_step_response.png'));

% =========================================================================
% Figure 2 -- Disturbance Rejection
% =========================================================================
fig2 = figure('Position', [100 100 900 500], 'Color', 'w');

plot(t, yd, 'Color', proc_blue, 'LineWidth', 2);
hold on;  grid on;
yline(0, '--k');

xlabel('Time (s)');
ylabel('Speed Deviation (m/s)');
title('Load Disturbance Rejection (sudden bottle accumulation)');

saveas(fig2, fullfile(plot_dir, 'problem1_disturbance.png'));

% =========================================================================
% Console output
% =========================================================================
fprintf('\n--- Problem 1: Conveyor speed PID ---\n');
fprintf('Kp=%.4f  Ki=%.4f  Kd=%.4f\n', Kp, Ki, Kd);
fprintf('Rise time     : %.3f s\n',  info.RiseTime);
fprintf('Settling time : %.3f s\n',  info.SettlingTime);
fprintf('Overshoot     : %.2f %%\n', info.Overshoot);
fprintf('Plots saved to: %s\n\n',    plot_dir);
