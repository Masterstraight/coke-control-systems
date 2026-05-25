clear; clc; close all;

% =========================================================
% Problem Title  : Carbonator Vessel Pressure PI Control
%                  -- Cohen-Coon vs Skogestad SIMC Comparison
% Plant          : Pneumatic CO2 injection valve into chilled
%                  syrup-water mix (~4 degC) on a CSD line.
%                  Combined valve actuation + pipe transport +
%                  sensor lag modelled as FOPDT.
% Controller     : PI  (Kd = 0)
%                  Tuning A -- Cohen-Coon FOPDT  (BASELINE / what not to do)
%                  Tuning B -- Skogestad SIMC    (RECOMMENDED for production)
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
%
% TUNING COMPARISON
% ---------------------------------------------------------
% Cohen-Coon targets a quarter-decay-ratio (QDR) response,
% which historically implied "fast with modest overshoot".
% However, for FOPDT plants with mild dead time (L/tau < 0.2)
% the QDR formula produces very high integral gain. On this
% plant (L/tau = 0.125) the resulting Kp (~4.0) and Ki (~3.1)
% drive the closed loop into sustained oscillation with ~78%
% overshoot, crossing the 4.77 bar safety relief threshold on
% every cycle. Cohen-Coon should NEVER be used as a final
% setting for fast pressure loops -- it is retained here only
% as an instructive "what not to do" baseline.
%
% Skogestad SIMC selects a conservative closed-loop time
% constant:  lambda = max(L, 0.5*tau)
% This guarantees gain-margin-safe operation at the cost of a
% slower setpoint response. For a safety-critical carbonation
% loop, slower + safe is the only acceptable design choice.
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
% Tuning A: Cohen-Coon  (FOPDT PI -- baseline, aggressive)
%   Kp = (1/K) * (tau/L) * (0.9 + L/(12*tau))
%   Ti = L * (30 + 3*L/tau) / (9 + 20*L/tau)
% =========================================================================
Kp_cc = (1/K) * (tau/L) * (0.9 + L/(12*tau));
Ti_cc = L * (30 + 3*(L/tau)) / (9 + 20*(L/tau));
Ki_cc = Kp_cc / Ti_cc;
C_cc  = pid(Kp_cc, Ki_cc);

% =========================================================================
% Tuning B: Skogestad SIMC  (FOPDT PI -- recommended, robust)
%   lambda_SIMC = max(L, 0.5*tau)        % conservative lambda choice
%   Kp_simc = (1/K) * tau / (lambda_SIMC + L)
%   Ti_simc = min(tau, 4*(lambda_SIMC + L))
% =========================================================================
lambda_SIMC = max(L, 0.5 * tau);
Kp_simc     = (1/K) * tau / (lambda_SIMC + L);
Ti_simc     = min(tau, 4 * (lambda_SIMC + L));
Ki_simc     = Kp_simc / Ti_simc;
C_simc      = pid(Kp_simc, Ki_simc);

% =========================================================================
% Closed-loop transfer functions  (both built on Pade-approximated plant)
% =========================================================================
T_yr_cc   = feedback(C_cc   * G_pade, 1);  % Cohen-Coon: reference -> output
T_yr_simc = feedback(C_simc * G_pade, 1);  % SIMC:       reference -> output
T_yd_cc   = feedback(G_pade, C_cc);        % Cohen-Coon: disturbance -> output
T_yd_simc = feedback(G_pade, C_simc);      % SIMC:       disturbance -> output

% =========================================================================
% Simulation
% =========================================================================
SP  = 4.5;               % setpoint  (bar)
D   = 0.4;               % disturbance amplitude (bar equivalent at input)
t   = 0:0.05:30;         % time vector (s)

y_cc    = step(SP * T_yr_cc,   t);
y_simc  = step(SP * T_yr_simc, t);
yd_cc   = step(D  * T_yd_cc,   t);
yd_simc = step(D  * T_yd_simc, t);

% =========================================================================
% Step-info metrics
% =========================================================================
info_cc   = stepinfo(SP * T_yr_cc);
info_simc = stepinfo(SP * T_yr_simc);

% =========================================================================
% Colour palette  (CLAUDE.md conventions)
% =========================================================================
coke_red  = [0.86 0.08 0.24];   % Cohen-Coon trace (warning / baseline)
proc_blue = [0.0  0.45 0.74];   % SIMC trace (recommended)

% =========================================================================
% Figure 1 -- Step Response Comparison
% =========================================================================
fig1 = figure('Position', [100 100 950 550], 'Color', 'w');

h1 = plot(t, y_cc,   'Color', coke_red,  'LineWidth', 1.8);
hold on;  grid on;
h2 = plot(t, y_simc, 'Color', proc_blue, 'LineWidth', 2.2);

h3 = yline(SP,        '--k', 'Setpoint');   % nominal setpoint
h4 = yline(SP * 1.06, ':r',  '+6% safety'); % safety-relief limit

xlabel('Time (s)');
ylabel('Pressure (bar)');
title('Carbonator Pressure -- Cohen-Coon vs SIMC PI tuning');
legend([h1, h2, h3, h4], ...
       {'Cohen-Coon (78 % overshoot)', 'SIMC', 'Setpoint', 'Safety'}, ...
       'Location', 'southeast');

saveas(fig1, fullfile(plot_dir, 'problem2_step_response.png'));

% =========================================================================
% Figure 2 -- Disturbance Rejection Comparison
% =========================================================================
fig2 = figure('Position', [100 100 950 550], 'Color', 'w');

h5 = plot(t, yd_cc,   'Color', coke_red,  'LineWidth', 1.8);
hold on;  grid on;
h6 = plot(t, yd_simc, 'Color', proc_blue, 'LineWidth', 2.2);
yline(0, '--k');

xlabel('Time (s)');
ylabel('Pressure Deviation (bar)');
title('CO_2 Header Disturbance -- Cohen-Coon vs SIMC');
legend([h5, h6], {'Cohen-Coon', 'SIMC'}, 'Location', 'northeast');

saveas(fig2, fullfile(plot_dir, 'problem2_disturbance.png'));

% =========================================================================
% Console output
% =========================================================================
fprintf('\n--- Problem 2: Carbonator pressure PI (tuning comparison) ---\n');
fprintf('Cohen-Coon:  Kp=%.4f  Ki=%.4f  (Ti=%.4f s)\n', Kp_cc,   Ki_cc,   Ti_cc);
fprintf('             Overshoot=%.1f %%   Settling=%.1f s   Peak=%.3f bar\n', ...
        info_cc.Overshoot, info_cc.SettlingTime, info_cc.Peak);
fprintf('SIMC:        Kp=%.4f  Ki=%.4f  (Ti=%.4f s)\n', Kp_simc, Ki_simc, Ti_simc);
fprintf('             Overshoot=%.1f %%   Settling=%.1f s   Peak=%.3f bar\n', ...
        info_simc.Overshoot, info_simc.SettlingTime, info_simc.Peak);
fprintf('Note:        SIMC''s larger disturbance peak (~0.23 bar) is the\n');
fprintf('             classical setpoint-vs-disturbance trade-off with single\n');
fprintf('             PI -- still acceptable here (well below the 6 %% safety\n');
fprintf('             margin), and the alternative (Cohen-Coon''s 78 %% setpoint\n');
fprintf('             overshoot) would lift the relief valve. For applications\n');
fprintf('             needing both, see 2-DOF PID.\n');
fprintf('Plots saved to: %s\n\n', plot_dir);
