clear; clc; close all;

% =========================================================
% Problem Title  : Carbonator Vessel Pressure PI Control
% Plant          : Pneumatic CO2 injection valve into chilled
%                  syrup-water mix (~4 degC) on a CSD line.
%                  Combined valve actuation + pipe transport +
%                  sensor lag modelled as FOPDT.
% Controller     : PI  (Kd = 0), Cohen-Coon FOPDT tuning
% Difficulty     : EASY-MEDIUM
% Plant TF       : G(s) = K * exp(-L*s) / (tau*s + 1)
%                  K   = 1.8   bar/(% valve)  steady-state gain
%                  tau = 4.0   s              process time constant
%                  L   = 0.5   s              dead time
% Requirements   : Setpoint      = 4.5 bar  (~3.7 vol CO2 at 4 degC,
%                                             Coca-Cola Classic spec)
%                  Overshoot     < 6 %  (safety relief valve lifts above)
%                  Settling time < 20 s  (2%)
%                  Reject a CO2 header supply-pressure disturbance
%                  equivalent to +0.4 bar at the plant input
% =========================================================

% ---- plots/ folder auto-creation ----------------------------------------
script_dir = fileparts(mfilename('fullpath'));
plot_dir   = fullfile(script_dir, 'plots');
if ~exist(plot_dir, 'dir'); mkdir(plot_dir); end

% =========================================================================
% Plant definition  (FOPDT with input dead time)
% =========================================================================
K   = 1.8;               % bar / (% valve)  steady-state gain
tau = 4.0;               % s                process time constant
L   = 0.5;               % s                dead time (L/tau = 0.125)

G      = tf(K, [tau, 1], 'InputDelay', L);
G_pade = pade(G, 2);     % 2nd-order Pade approximation of exp(-L*s)

% =========================================================================
% Cohen-Coon tuning  (FOPDT formulae, PI form)
%   Kp = (1/K) * (tau/L) * (0.9 + L/(12*tau))
%   Ti = L * (30 + 3*L/tau) / (9 + 20*L/tau)
%   Ki = Kp / Ti
% =========================================================================
Kp = (1/K) * (tau/L) * (0.9 + L/(12*tau));
Ti = L * (30 + 3*(L/tau)) / (9 + 20*(L/tau));
Ki = Kp / Ti;
Kd = 0;
C  = pid(Kp, Ki);        % PI controller (no derivative -- TX noise)

% =========================================================================
% Closed-loop transfer functions  (built on Pade-approximated plant)
% =========================================================================
T_yr = feedback(C * G_pade, 1);   % reference -> output
T_yd = feedback(G_pade, C);       % input disturbance -> output

% =========================================================================
% Simulation
% =========================================================================
SP  = 4.5;               % setpoint  (bar)
D   = 0.4;               % disturbance amplitude (bar equivalent at input)
t   = 0:0.05:30;         % time vector (s)
y   = step(SP * T_yr, t);
yd  = step(D  * T_yd, t);

% =========================================================================
% Step-info metrics
% =========================================================================
info = stepinfo(SP * T_yr);

% =========================================================================
% Colour palette  (CLAUDE.md conventions)
% =========================================================================
coke_red  = [0.86 0.08 0.24];   % primary signal
proc_blue = [0.0  0.45 0.74];   % comparison signal

% =========================================================================
% Figure 1 -- Step Response
% =========================================================================
fig1 = figure('Position', [100 100 900 500], 'Color', 'w');

h1 = plot(t, y, 'Color', coke_red, 'LineWidth', 2);
hold on;  grid on;

h2 = yline(SP,        '--k', 'Setpoint');   % nominal setpoint
h3 = yline(SP * 1.06, ':r',  '+6% safety'); % safety-relief limit

xlabel('Time (s)');
ylabel('Pressure (bar)');
title('Carbonator Pressure -- PI Step Response to 4.5 bar');
legend([h1, h2, h3], {'Pressure', 'Setpoint', 'Safety limit'}, ...
       'Location', 'southeast');

saveas(fig1, fullfile(plot_dir, 'problem2_step_response.png'));

% =========================================================================
% Figure 2 -- Disturbance Rejection
% =========================================================================
fig2 = figure('Position', [100 100 900 500], 'Color', 'w');

plot(t, yd, 'Color', proc_blue, 'LineWidth', 2);
hold on;  grid on;
yline(0, '--k');

xlabel('Time (s)');
ylabel('Pressure Deviation (bar)');
title('CO_2 Header Pressure Disturbance Rejection');

saveas(fig2, fullfile(plot_dir, 'problem2_disturbance.png'));

% =========================================================================
% Console output
% =========================================================================
fprintf('\n--- Problem 2: Carbonator pressure PI ---\n');
fprintf('Kp=%.4f  Ki=%.4f  (Ti=%.4f s)\n', Kp, Ki, Ti);
fprintf('Settling time : %.3f s\n',  info.SettlingTime);
fprintf('Overshoot     : %.2f %%\n', info.Overshoot);
fprintf('Plots saved to: %s\n\n',    plot_dir);
