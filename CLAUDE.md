# CLAUDE.md — Coke Control Systems Portfolio

This file is the **single source of truth** for project conventions.
Every `.m` file, every commit, and every later prompt implicitly references it.

---

## 1. Project Goal

A MATLAB portfolio of the **four most common controllers** used in
Coca-Cola beverage production:

| # | Controller | Folder |
|---|------------|--------|
| 1 | PID | `01-pid-control/` |
| 2 | Cascade | `02-cascade-control/` |
| 3 | Ratio | `03-ratio-control/` |
| 4 | Feedforward | `04-feedforward-control/` |

Each folder contains **5 problems** that escalate in difficulty from EASY
to VERY HARD. All 20 problems together form the complete portfolio.

---

## 2. Build Sequence

Problems are added **one at a time** in this exact order:

```
01-pid-control/problem1_conveyor_speed.m
01-pid-control/problem2_carbonator_pressure.m
01-pid-control/problem3_cip_tank_temperature.m
01-pid-control/problem4_filler_valve_nonlinear.m
01-pid-control/problem5_coupled_tanks_mimo.m
02-cascade-control/problem1_pasteurizer_basic.m
02-cascade-control/problem2_syrup_tank_level.m
02-cascade-control/problem3_carbonator_cascade.m
02-cascade-control/problem4_cip_heat_exchanger.m
02-cascade-control/problem5_pasteurizer_regen.m
03-ratio-control/problem1_basic_syrup_water.m
03-ratio-control/problem2_ratio_with_brix_trim.m
03-ratio-control/problem3_three_component_blend.m
03-ratio-control/problem4_product_changeover.m
03-ratio-control/problem5_adaptive_ratio.m
04-feedforward-control/problem1_static_ff_cip.m
04-feedforward-control/problem2_production_rate_ff.m
04-feedforward-control/problem3_ff_fb_pasteurizer.m
04-feedforward-control/problem4_dynamic_ff_leadlag.m
04-feedforward-control/problem5_multivariable_ff_blender.m
```

---

## 3. Status Tracker

- [x] `01-pid-control/problem1_conveyor_speed.m`
- [x] `01-pid-control/problem2_carbonator_pressure.m`
- [ ] `01-pid-control/problem3_cip_tank_temperature.m`
- [ ] `01-pid-control/problem4_filler_valve_nonlinear.m`
- [ ] `01-pid-control/problem5_coupled_tanks_mimo.m`
- [ ] `02-cascade-control/problem1_pasteurizer_basic.m`
- [ ] `02-cascade-control/problem2_syrup_tank_level.m`
- [ ] `02-cascade-control/problem3_carbonator_cascade.m`
- [ ] `02-cascade-control/problem4_cip_heat_exchanger.m`
- [ ] `02-cascade-control/problem5_pasteurizer_regen.m`
- [ ] `03-ratio-control/problem1_basic_syrup_water.m`
- [ ] `03-ratio-control/problem2_ratio_with_brix_trim.m`
- [ ] `03-ratio-control/problem3_three_component_blend.m`
- [ ] `03-ratio-control/problem4_product_changeover.m`
- [ ] `03-ratio-control/problem5_adaptive_ratio.m`
- [ ] `04-feedforward-control/problem1_static_ff_cip.m`
- [ ] `04-feedforward-control/problem2_production_rate_ff.m`
- [ ] `04-feedforward-control/problem3_ff_fb_pasteurizer.m`
- [ ] `04-feedforward-control/problem4_dynamic_ff_leadlag.m`
- [ ] `04-feedforward-control/problem5_multivariable_ff_blender.m`

**Progress: 2 / 20**

---

## 4. Conventions Every `.m` File MUST Follow

### 4.1 Preamble

Every script begins with:

```matlab
clear; clc; close all;
```

### 4.2 Top-of-File Comment Header Block

```matlab
% =========================================================
% Problem Title  : <full descriptive title>
% Plant          : <plant equipment name>
% Controller     : <controller type>
% Difficulty     : EASY | MEDIUM | HARD | VERY HARD
% Plant TF       : G(s) = K * exp(-theta*s) / (tau*s + 1)
%                  (ASCII math -- fill in actual numeric values)
% Requirements   : Ts < X s, OS < Y%, ess = 0
% =========================================================
```

### 4.3 `plots/` Folder Auto-Creation

Place this block immediately after the header comment:

```matlab
script_dir = fileparts(mfilename('fullpath'));
plot_dir   = fullfile(script_dir, 'plots');
if ~exist(plot_dir, 'dir'); mkdir(plot_dir); end
```

### 4.4 Saving Figures

```matlab
saveas(fig, fullfile(plot_dir, 'descriptive_name.png'));
```

### 4.5 Console Output (end of script)

Print the following to the command window:
- Controller gains (Kp, Ki, Kd, or equivalent)
- Key `stepinfo` metrics:
  - Rise time (s)
  - Settling time at 2% (s)
  - Overshoot (%)
  - Steady-state error

### 4.6 Plot Conventions

| Property | Value |
|----------|-------|
| Figure position — single plot | `[100 100 900 500]` |
| Figure position — multi-subplot | `[100 100 1000 700]` or larger |
| Figure background | `'w'` (white) |
| **Coke red** — primary signals | `[0.86 0.08 0.24]` |
| **Process blue** — comparison signals | `[0.0  0.45 0.74]` |
| **Neutral gray** — setpoints / disturbances | `[0.4  0.4  0.4]` |
| Grid | `grid on` |
| Axes | Fully labelled (`xlabel`, `ylabel`, `title`) |
| Legend | `'Location', 'southeast'` unless inappropriate |
| Setpoint lines | `yline(SP, '--k', 'Setpoint')` |

---

## 5. Tuning Preferences (Default per Problem)

Override only when a specific prompt explicitly says otherwise.

| Problem | Tuning Method |
|---------|---------------|
| PID 1 | IMC (lambda tuning) |
| PID 2 | Cohen-Coon (FOPDT) |
| PID 3 | Skogestad SIMC (SOPDT) |
| PID 4 | Hand-tuned + anti-stiction dither |
| PID 5 | RGA + static decoupler + diagonal PI |
| Cascade 1–3 | IMC on inner loop, IMC on outer loop |
| Cascade 4 | Inner positioner PI; outer SIMC PID with N = 5 |
| Cascade 5 | Discrete-time sim with input buffers |
| Ratio 1–3 | PI on flow loop; ratio block computes SP |
| Ratio 4 | Linear ramp + integral tracking (bumpless transfer) |
| Ratio 5 | RLS with λ_RLS = 0.98 |
| FF 1–3 | Static gain: Kff = −Kd_ss / Kp_ss |
| FF 4 | Lead-lag: Kff · (τ_z·s + 1) / (τ_p·s + 1) |
| FF 5 | Static MIMO: Kff = −inv(Kp_ss) · Kd_ss |

---

## 6. Toolbox Dependency

| Item | Policy |
|------|--------|
| **Required** | Control System Toolbox |
| **Forbidden** | System Identification Toolbox, Simulink, Optimization Toolbox |
| **MATLAB version** | R2019b or later |
| **Portability** | Must run from *any* working directory — all paths derived via `mfilename('fullpath')` |

---

## 7. Workflow Rule

> When asked to create a problem file, **ONLY** create that single `.m` file.
>
> - Do **not** pre-emptively create the other 19 problems.
> - Do **not** modify any `README.md` files unless explicitly asked.
> - **Do** update the status checkbox in Section 3 of this file for the
>   problem just completed — change `- [ ]` to `- [x]` and update the
>   progress counter.
