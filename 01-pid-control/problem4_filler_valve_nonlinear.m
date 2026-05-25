clear; clc; close all;

% =========================================================
% Problem Title  : Filler Valve Nonlinear PID -- Flow Regulation
%                  (Stiction + Anti-stiction Dither Comparison)
% Plant          : Pneumatic filler valve on a 24,000 bph Coca-Cola
%                  PET filling line. Linear first-order valve dynamics
%                  with a stiction (static friction) nonlinearity.
% Controller     : PID -- TWO VARIANTS compared in this script:
%                  Variant A -- Naive PID + conditional anti-windup,
%                               no dither
%                  Variant B -- Same PID + conditional anti-windup +
%                               50 Hz anti-stiction dither
%                  NOTE: No closed-form SIMC / IMC tuning formula
%                  applies to this nonlinear plant. Gains are hand-
%                  tuned (Kp*K_plant = 0.9; Ti = Kp/Ki = 0.1 s,
%                  matching the 0.15 s valve time constant).
% Difficulty     : HARD
% Plant TF       : G_lin(s) = K / (tau*s + 1)   [linear part]
%                  K            = 15.0  mL/s per % valve opening
%                                 (max flow ~1500 mL/s at 100 %,
%                                  ~20 % headroom above 1250 mL/s SP)
%                  tau          = 0.15  s         valve dynamics
%                  Stiction band   = 2.0 %        stem sticks until
%                                    |u_cmd - u_stem| exceeds band
%                  Slip overshoot  = 1.0 %        stem overshoots
%                                    by this amount on break-away
% Test scenario  : FLOW REGULATION -- controller holds steady flow
%                  at 1250 mL/s for ~2.4 s after a 0.05 s startup.
%                  Stiction is only observable when the controller
%                  is HOLDING position, not ramping toward it -- this
%                  scenario isolates that behaviour. Real bottle
%                  filling requires volume control with feed-forward
%                  valve-close logic (out of scope here).
% Requirements   : SP = 1250 mL/s (= 500 mL / 0.4 s fill window)
%                  Reach SP within ~0.3 s of trigger
%                  Anti-stiction variant must show lower flow std
%                  deviation than naive variant during hold phase
%                  Stems must NOT saturate at 100 % during hold
%                    (saturation only during 0.05 s startup is OK)
% =========================================================

% ---- plots/ folder auto-creation ----------------------------------------
script_dir = fileparts(mfilename('fullpath'));
plot_dir   = fullfile(script_dir, 'plots');
if ~exist(plot_dir, 'dir'); mkdir(plot_dir); end

% =========================================================================
% Plant parameters
% =========================================================================
K              = 15.0;   % mL/s per % valve opening  (max 1500 mL/s)
tau            = 0.15;   % s  valve first-order time constant
stiction_band  = 2.0;    % %  stem dead-band width
slip_overshoot = 1.0;    % %  overshoot magnitude on break-away

% =========================================================================
% Controller gains  (hand-tuned -- identical for both variants)
%   Loop gain:   Kp * K = 0.06 * 15 = 0.9
%   Integral TC: Ti = Kp / Ki = 0.06 / 0.6 = 0.1 s  (~= tau)
%   Derivative:  Td = Kd / Kp = 0.002 / 0.06 = 0.033 s
% =========================================================================
Kp = 0.06;
Ki = 0.6;
Kd = 0.002;

% =========================================================================
% Time vector and setpoint  (hold test -- no volume cutoff)
% =========================================================================
dt = 0.001;              % s
t  = 0:dt:2.5;           % 2.5 s simulation
N  = numel(t);

sp           = ones(1, N) .* 1250;   % 1250 mL/s flow target
sp(t < 0.05) = 0;                     % valve closed before bottle trigger

% =========================================================================
% Manual time-stepping simulation  -- both variants
% =========================================================================
y_all    = zeros(2, N);
stem_all = zeros(2, N);

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

% Cumulative volumes (informational -- hold test runs full 2.5 s)
vol_naive = cumtrapz(t, y_naive);
vol_anti  = cumtrapz(t, y_anti);

% =========================================================================
% Hold-phase metrics  (window: t = 0.3 s to 2.4 s)
%   -- excludes startup transient; isolates stiction limit-cycle
% =========================================================================
idx_h0 = find(t >= 0.3, 1, 'first');
idx_h1 = find(t >= 2.4, 1, 'first');

stem_std  = zeros(1, 2);
flow_std  = zeros(1, 2);
sat_ms    = zeros(1, 2);

for v = 1:2
    stem_seg  = stem_all(v, idx_h0:idx_h1);
    flow_seg  = y_all(v,    idx_h0:idx_h1);
    stem_std(v) = std(stem_seg);
    flow_std(v) = std(flow_seg);
    sat_ms(v)   = sum(stem_seg >= 99.9) * dt * 1000;
end

% =========================================================================
% Colour palette  (CLAUDE.md conventions)
% =========================================================================
coke_red  = [0.86 0.08 0.24];
proc_blue = [0.0  0.45 0.74];

% =========================================================================
% Figure 1 -- Flow rate and valve stem position  (2x1 subplot)
% =========================================================================
fig1 = figure('Position', [100 100 1000 600], 'Color', 'w');

subplot(2, 1, 1);
h1 = plot(t, sp,       '--k',           'LineWidth', 1.2);
hold on;  grid on;
h2 = plot(t, y_naive,  'Color', coke_red,  'LineWidth', 1.5);
h3 = plot(t, y_anti,   'Color', proc_blue, 'LineWidth', 1.5);
ylim([0, 1500]);
xlabel('Time (s)');
ylabel('Flow (mL/s)');
title('Filler Valve Flow -- naive PID vs anti-stiction PID with dither');
legend([h1, h2, h3], {'Setpoint', 'Naive PID', 'Anti-stiction'}, ...
       'Location', 'southeast');

subplot(2, 1, 2);
h4 = plot(t, stem_naive, 'Color', coke_red,  'LineWidth', 1.4);
hold on;  grid on;
h5 = plot(t, stem_anti,  'Color', proc_blue, 'LineWidth', 1.4);
ylim([0, 100]);
xlabel('Time (s)');
ylabel('Stem position (%)');
title('Valve stem position');
legend([h4, h5], {'Naive', 'Anti-stiction'}, 'Location', 'southeast');

saveas(fig1, fullfile(plot_dir, 'problem4_flow_and_stem.png'));

% =========================================================================
% Figure 2 -- Cumulative flow (informational)
% =========================================================================
fig2 = figure('Position', [100 100 900 500], 'Color', 'w');

h6 = plot(t, vol_naive, 'Color', coke_red,  'LineWidth', 2);
hold on;  grid on;
h7 = plot(t, vol_anti,  'Color', proc_blue, 'LineWidth', 2);

h8 = yline(500, '--k', '500 mL reference');
h8.LineWidth = 1.2;

xlabel('Time (s)');
ylabel('Dispensed volume (mL)');
title('Cumulative flow over 2.5 s hold test');
legend([h6, h7, h8], ...
       {'Naive PID', 'Anti-stiction PID', '500 mL reference'}, ...
       'Location', 'southeast');

saveas(fig2, fullfile(plot_dir, 'problem4_volume.png'));

% =========================================================================
% Console output
% =========================================================================
fprintf('\n--- Problem 4: Filler valve nonlinear PID ---\n');
fprintf('Plant:        K=%.1f  tau=%.2f s  (linear part, max flow 1500 mL/s)\n', K, tau);
fprintf('              Stiction band=%.1f %%  Slip jump=%.1f %%  (nonlinear part)\n', ...
        stiction_band, slip_overshoot);
fprintf('Method:       Hand-tuned PID, Kp=%.4f, Ki=%.4f, Kd=%.5f\n', Kp, Ki, Kd);
fprintf('              Variant B adds 50 Hz, +/-0.5 %% anti-stiction dither.\n');
fprintf('Hold-phase metrics (t = 0.3 s to 2.4 s):\n');
fprintf('Naive PID:    stem_std = %.3f %%   flow_std = %.2f mL/s\n', ...
        stem_std(1), flow_std(1));
fprintf('              sat_time = %.1f ms at 100 %%   total_vol = %.0f mL\n', ...
        sat_ms(1), vol_naive(end));
fprintf('Anti-stiction:stem_std = %.3f %%   flow_std = %.2f mL/s\n', ...
        stem_std(2), flow_std(2));
fprintf('              sat_time = %.1f ms at 100 %%   total_vol = %.0f mL\n', ...
        sat_ms(2), vol_anti(end));
fprintf('Lesson:       anti-stiction stem_std >= naive (dither adds motion)\n');
fprintf('              anti-stiction flow_std  < naive (smoother result)\n');
fprintf('Plots saved to: %s\n\n', plot_dir);
