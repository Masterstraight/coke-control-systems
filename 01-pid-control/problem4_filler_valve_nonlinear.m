clear; clc; close all;

% =========================================================
% Problem Title  : Filler Valve Nonlinear PID Control
%                  (Stiction + Anti-stiction Dither Comparison)
% Plant          : Pneumatic filler valve on a 24,000 bph Coca-Cola
%                  PET filling line.  Dispenses 500 mL per bottle in
%                  a 0.4 s window. Linear first-order valve dynamics
%                  with a stiction (static friction) nonlinearity in
%                  the valve stem.
% Controller     : PID -- TWO VARIANTS compared in this script:
%                  Variant A -- Naive PID + conditional anti-windup,
%                               no dither
%                  Variant B -- Same PID + conditional anti-windup +
%                               50 Hz anti-stiction dither
%                  NOTE: No closed-form SIMC / IMC tuning formula
%                  applies to this nonlinear plant. Gains are hand-
%                  tuned to stabilise the loop while making the
%                  stiction behaviour clearly visible.
% Difficulty     : HARD
% Plant TF       : G_lin(s) = K / (tau*s + 1)   [linear part]
%                  K            = 1.2   mL/s per % valve opening
%                  tau          = 0.15  s         valve dynamics
%                  Stiction band   = 2.0 %        stem sticks until
%                                    |u_cmd - u_stem| exceeds band
%                  Slip overshoot  = 1.0 %        stem overshoots on
%                                    break-away
% Requirements   : Fill 500 mL per bottle within +/- 2 mL (+/- 0.4 %)
%                  Filling window: 0.4 s active dispense (SP steps
%                    to 500/0.4 mL/s at t = 0.05 s)
%                  Eliminate stiction limit cycles that cause
%                    bottle-to-bottle volume variance
%                  Total settle time within 1.2 s
% =========================================================

% ---- plots/ folder auto-creation ----------------------------------------
script_dir = fileparts(mfilename('fullpath'));
plot_dir   = fullfile(script_dir, 'plots');
if ~exist(plot_dir, 'dir'); mkdir(plot_dir); end

% =========================================================================
% Plant parameters
% =========================================================================
K              = 1.2;    % mL/s per % valve opening
tau            = 0.15;   % s  valve first-order time constant
stiction_band  = 2.0;    % %  stem motion dead-band
slip_overshoot = 1.0;    % %  overshoot on break-away from stiction

% =========================================================================
% Controller gains  (hand-tuned -- identical for both variants)
% =========================================================================
Kp = 0.08;
Ki = 0.35;
Kd = 0.008;

% =========================================================================
% Time vector and setpoint
% =========================================================================
dt = 0.001;              % s
t  = 0:dt:3;             % 3 s simulation
N  = numel(t);

sp           = ones(1, N) .* (500/0.4);   % 1250 mL/s flow target
sp(t < 0.05) = 0;                          % valve closed before bottle trigger

% =========================================================================
% Manual time-stepping simulation  -- both variants
% =========================================================================
y_all    = zeros(2, N);    % flow rate traces
stem_all = zeros(2, N);    % valve stem position traces

for variant = 1:2
    y        = zeros(1, N);
    stem_log = zeros(1, N);
    integ    = 0;
    e_prev   = 0;
    u_cmd    = 0;
    u_stem   = 0;

    for k = 2:N
        e = sp(k) - y(k-1);

        % Conditional anti-windup: freeze integrator when saturated
        if (u_cmd > 0 && u_cmd < 100)
            integ = integ + e * dt;
        end

        deriv = (e - e_prev) / dt;
        u_cmd = Kp*e + Ki*integ + Kd*deriv;

        % Anti-stiction dither (Variant B only) -- 50 Hz square wave
        if variant == 2
            u_cmd = u_cmd + 0.5 * sign(sin(2*pi*50*t(k)));
        end

        u_cmd = max(0, min(100, u_cmd));   % clamp to [0, 100] %

        % Stiction nonlinearity
        if abs(u_cmd - u_stem) > stiction_band
            u_stem = u_cmd + slip_overshoot * sign(u_cmd - u_stem);
            u_stem = max(0, min(100, u_stem));
        end

        % Valve linear dynamics  (forward Euler)
        y(k) = y(k-1) + (dt/tau) * (K * u_stem - y(k-1));

        stem_log(k) = u_stem;
        e_prev      = e;
    end

    y_all(variant, :)    = y;
    stem_all(variant, :) = stem_log;
end

% Extract individual variant results
y_naive    = y_all(1, :);
y_anti     = y_all(2, :);
stem_naive = stem_all(1, :);
stem_anti  = stem_all(2, :);

% =========================================================================
% Cumulative dispensed volume
% =========================================================================
vol_naive = cumtrapz(t, y_naive);
vol_anti  = cumtrapz(t, y_anti);

% =========================================================================
% Colour palette  (CLAUDE.md conventions)
% =========================================================================
coke_red  = [0.86 0.08 0.24];
proc_blue = [0.0  0.45 0.74];
neut_gray = [0.4  0.4  0.4];

% =========================================================================
% Figure 1 -- Flow rate and valve stem position  (2x1 subplot)
% =========================================================================
fig1 = figure('Position', [100 100 1000 600], 'Color', 'w');

subplot(2, 1, 1);
h1 = plot(t, sp,       '--k',       'LineWidth', 1.2);
hold on;  grid on;
h2 = plot(t, y_naive,  'Color', coke_red,  'LineWidth', 1.5);
h3 = plot(t, y_anti,   'Color', proc_blue, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Flow (mL/s)');
title('Filler Valve Flow -- naive PID vs anti-stiction PID with dither');
legend([h1, h2, h3], {'Setpoint', 'Naive PID', 'Anti-stiction'}, ...
       'Location', 'southeast');

subplot(2, 1, 2);
h4 = plot(t, stem_naive, 'Color', coke_red,  'LineWidth', 1.4);
hold on;  grid on;
h5 = plot(t, stem_anti,  'Color', proc_blue, 'LineWidth', 1.4);
xlabel('Time (s)');
ylabel('Stem position (%)');
title('Valve stem position');
legend([h4, h5], {'Naive', 'Anti-stiction'}, 'Location', 'southeast');

saveas(fig1, fullfile(plot_dir, 'problem4_flow_and_stem.png'));

% =========================================================================
% Figure 2 -- Cumulative dispensed volume
% =========================================================================
fig2 = figure('Position', [100 100 900 500], 'Color', 'w');

h6 = plot(t, vol_naive, 'Color', coke_red,  'LineWidth', 2);
hold on;  grid on;
h7 = plot(t, vol_anti,  'Color', proc_blue, 'LineWidth', 2);

h8    = yline(500, '--k', '500 mL target');
h8.LineWidth = 1.2;
h9    = yline(502, ':');
h9_lo = yline(498, ':');
h9.Color              = neut_gray;
h9_lo.Color           = neut_gray;
h9_lo.HandleVisibility = 'off';    % single legend entry for both band lines

xlabel('Time (s)');
ylabel('Dispensed volume (mL)');
title('Cumulative dispensed volume per bottle');
legend([h6, h7, h8, h9], ...
       {'Naive PID', 'Anti-stiction PID', 'Target', '\pm2 mL band'}, ...
       'Location', 'southeast');

saveas(fig2, fullfile(plot_dir, 'problem4_volume.png'));

% =========================================================================
% Console output
% =========================================================================
fprintf('\n--- Problem 4: Filler valve nonlinear PID ---\n');
fprintf('Plant:        K=%.1f  tau=%.2f s  (linear part)\n', K, tau);
fprintf('              Stiction band=%.1f %%  Slip jump=%.1f %%  (nonlinear part)\n', ...
        stiction_band, slip_overshoot);
fprintf('Method:       Hand-tuned PID, both variants share Kp=%.3f, Ki=%.3f, Kd=%.4f\n', ...
        Kp, Ki, Kd);
fprintf('              Variant B adds 50 Hz, +/-0.5 %% anti-stiction dither.\n');
fprintf('Naive PID:    Final volume = %.2f mL   (target 500, tolerance +/- 2)\n', ...
        vol_naive(end));
fprintf('Anti-stiction:Final volume = %.2f mL\n', vol_anti(end));
fprintf('Note:         Anti-stiction keeps the stem moving through the\n');
fprintf('              dead-band; the resulting limit-cycle elimination is\n');
fprintf('              the lesson of this problem.\n');
fprintf('Plots saved to: %s\n\n', plot_dir);
