clear; clc; close all;

% =========================================================
% Problem Title  : CIP Tank Temperature PID Control
% Plant          : Jacketed caustic recirculation tank (~5000 L),
%                  heated by saturated steam through pneumatic valve.
%                  RTD at tank outlet provides measurement.
%                  Two real thermal lags (bulk fluid + jacket) plus
%                  transport and sensor delay.
% Controller     : PID with N=5 derivative filter
%                  Tuning: Skogestad SIMC for SOPDT, lambda = L = 15 s
% Difficulty     : MEDIUM
% Plant TF       : G(s) = K * exp(-L*s) / ((tau1*s+1)*(tau2*s+1))
%                  K    = 0.85   degC/(% steam valve)
%                  tau1 = 120    s   bulk fluid thermal time constant
%                  tau2 = 30     s   jacket dynamics
%                  L    = 15     s   transport + RTD + transmitter lag
% Requirements   : Setpoint      = 85 degC (caustic CIP wash temperature)
%                  Overshoot     < 3 degC absolute (ceramic valve seal limit)
%                  Settling time < 6 min from 20 degC ambient start
%                  Wash hold:    +/- 1 degC during wash phase
%                  Reject cold makeup-water inrush (-8 degC effective
%                  output disturbance) at t = 350 s into wash cycle
%
%  DISTURBANCE-REJECTION OBSERVATION
%  ---------------------------------
%  The simulated -8 degC cold-water inrush produces a peak deviation
%  of ~8 degC -- well outside the +/- 1 degC hold spec. This is NOT
%  a tuning failure; it is a fundamental limit of single-loop PID on
%  a plant with 15 s dead time + 120 s dominant lag fighting an
%  instantaneous output disturbance. The control round-trip
%  (RTD detection -> valve action -> jacket transfer -> bulk fluid
%  response) is ~3 minutes; during that window, no PID gain can
%  prevent the drop. Real plant mitigations:
%    1. Feed-forward on inlet water temperature (see FF problem 1)
%    2. Cascade with jacket-temperature inner loop (see Cascade
%       problem 4 -- CIP heat exchanger)
%    3. Slower makeup-water valve to spread the disturbance over
%       minutes rather than seconds.
%  The heat-up phase nonetheless meets all specs cleanly.
% =========================================================

% ---- plots/ folder auto-creation ----------------------------------------
script_dir = fileparts(mfilename('fullpath'));
plot_dir   = fullfile(script_dir, 'plots');
if ~exist(plot_dir, 'dir'); mkdir(plot_dir); end

% =========================================================================
% Plant definition  (SOPDT with input dead time)
% =========================================================================
K    = 0.85;             % degC / (% steam valve)
tau1 = 120;              % s  bulk fluid thermal time constant (dominant)
tau2 = 30;               % s  jacket dynamics
L    = 15;               % s  transport + RTD + transmitter delay

G      = tf(K, conv([tau1, 1], [tau2, 1]), 'InputDelay', L);
G_pade = pade(G, 3);     % 3rd-order Pade approximation of exp(-L*s)

% =========================================================================
% Skogestad SIMC tuning for SOPDT, lambda = L = 15 s
%   L_eff   = L + tau2/2          (30 s)
%   tau_eff = tau1 + tau2/2       (135 s)
%   Kp = (1/K) * tau_eff / (lambda + L_eff)
%   Ti = min(tau_eff, 4*(lambda + L_eff))
%   Td = (tau1 * tau2) / tau_eff
% =========================================================================
lambda  = L;                          % s  SIMC rule: lambda = theta
L_eff   = L   + tau2/2;               % 30 s
tau_eff = tau1 + tau2/2;              % 135 s

Kp = (1/K) * tau_eff / (lambda + L_eff);
Ti = min(tau_eff, 4*(lambda + L_eff));
Td = (tau1 * tau2) / tau_eff;
Ki = Kp / Ti;
Kd = Kp * Td;

C = pid(Kp, Ki, Kd, 5);              % PID, derivative filter N = 5

% =========================================================================
% Closed-loop transfer functions
% =========================================================================
T_yr = feedback(C * G_pade, 1);      % reference -> output
T_yd = feedback(1, C * G_pade);      % output disturbance -> output  (S)

% =========================================================================
% Simulation
% =========================================================================
sp = 85;                 % setpoint  (degC)
y0 = 20;                 % initial temperature (degC, ambient start)
t  = (0:1:600)';         % 10 minutes, 1 s resolution (column vector for lsim)

% Heat-up: ramp from 20 degC to 85 degC setpoint
y_heatup = y0 + step((sp - y0) * T_yr, t);

% Disturbance: cold makeup-water inrush (-8 degC step) at t = 350 s
d  = -8 * (t >= 350);    % output disturbance signal (degC)
yd = lsim(T_yd, d, t);   % temperature deviation at plant output

% =========================================================================
% Performance metrics
% =========================================================================
info_heatup = stepinfo(y_heatup, t, sp, y0);
max_dev     = max(abs(yd));           % degC -- worst-case wash deviation

% =========================================================================
% Colour palette  (CLAUDE.md conventions)
% =========================================================================
coke_red  = [0.86 0.08 0.24];   % primary signal
proc_blue = [0.0  0.45 0.74];   % comparison signal
neut_gray = [0.4  0.4  0.4 ];   % band lines

% =========================================================================
% Figure 1 -- Heat-up Response
% =========================================================================
fig1 = figure('Position', [100 100 950 550], 'Color', 'w');

h1 = plot(t/60, y_heatup, 'Color', coke_red, 'LineWidth', 2);
hold on;  grid on;

h2    = yline(sp,   '--k', '85 \circC setpoint');
h3    = yline(sp+1, ':');   % upper +1 degC hold band
h3_lo = yline(sp-1, ':');   % lower -1 degC hold band
h3.Color              = neut_gray;
h3_lo.Color           = neut_gray;
h3_lo.HandleVisibility = 'off';     % single legend entry for both band lines
h4    = yline(sp+3, ':r', 'Seal limit');   % 88 degC ceramic seal limit

xlabel('Time (min)');
ylabel('Tank Temperature (\circC)');
title('CIP Tank Heat-up -- PID from 20 \circC to 85 \circC');
legend([h1, h2, h3, h4], ...
       {'Temperature', 'Setpoint', '\pm1 \circC band', 'Seal limit'}, ...
       'Location', 'southeast');

saveas(fig1, fullfile(plot_dir, 'problem3_heatup.png'));

% =========================================================================
% Figure 2 -- Disturbance Rejection During Wash
% =========================================================================
fig2 = figure('Position', [100 100 950 550], 'Color', 'w');

h5    = plot(t/60, sp + yd, 'Color', proc_blue, 'LineWidth', 2);
hold on;  grid on;

h6    = yline(sp,   '--k', 'Setpoint');
h7    = yline(sp+1, ':');
h7_lo = yline(sp-1, ':');
h7.Color              = neut_gray;
h7_lo.Color           = neut_gray;
h7_lo.HandleVisibility = 'off';

xlabel('Time (min)');
ylabel('Tank Temperature (\circC)');
title('Cold Make-up Water Disturbance During Wash Cycle');
legend([h5, h6, h7], ...
       {'Temperature', 'Setpoint', '\pm 1 \circC hold band'}, ...
       'Location', 'southeast');

saveas(fig2, fullfile(plot_dir, 'problem3_disturbance.png'));

% =========================================================================
% Console output
% =========================================================================
fprintf('\n--- Problem 3: CIP tank temperature PID ---\n');
fprintf('Plant:     K=%.2f  tau1=%g s  tau2=%g s  L=%g s\n', K, tau1, tau2, L);
fprintf('Method:    Skogestad SIMC for SOPDT, lambda = L = %g s\n', lambda);
fprintf('Gains:     Kp=%.4f  Ki=%.5f  Kd=%.4f   (Ti=%.1f s, Td=%.2f s)\n', ...
        Kp, Ki, Kd, Ti, Td);
fprintf('Heat-up:   Settling=%.2f min  Peak=%.2f degC  (limit 88 degC)\n', ...
        info_heatup.SettlingTime/60, info_heatup.Peak);
fprintf('Wash hold: Max deviation=%.3f degC after disturbance\n', max_dev);
fprintf('Plots saved to: %s\n\n', plot_dir);
