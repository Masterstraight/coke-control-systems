# Coca-Cola Control Systems Portfolio

> A MATLAB portfolio of the four most common feedback and feedforward
> controllers found in Coca-Cola beverage production, built problem-by-problem
> from easy to very hard.

---

## Table of Contents

1. [Project Layout](#project-layout)
2. [Controllers Covered](#controllers-covered)
3. [Problem Index](#problem-index)
   - [01 — PID Control](#01--pid-control)
   - [02 — Cascade Control](#02--cascade-control)
   - [03 — Ratio Control](#03--ratio-control)
   - [04 — Feedforward Control](#04--feedforward-control)
4. [Running the Code](#running-the-code)
5. [Plant Numbers & Where They Come From](#plant-numbers--where-they-come-from)
6. [Tuning Methods Used](#tuning-methods-used)
7. [Roadmap](#roadmap)
8. [License](#license)

---

## Project Layout

```
coke-control-systems/
├── CLAUDE.md                          ← project conventions (single source of truth)
├── README.md                          ← this file
├── LICENSE
├── .gitignore
├── 01-pid-control/
│   ├── README.md
│   ├── plots/                         ← PNG outputs (generated on run)
│   └── problem1_conveyor_speed.m      ← (pending)
│   └── ...
├── 02-cascade-control/
│   ├── README.md
│   ├── plots/
│   └── problem1_pasteurizer_basic.m   ← (pending)
│   └── ...
├── 03-ratio-control/
│   ├── README.md
│   ├── plots/
│   └── problem1_basic_syrup_water.m   ← (pending)
│   └── ...
└── 04-feedforward-control/
    ├── README.md
    ├── plots/
    └── problem1_static_ff_cip.m       ← (pending)
    └── ...
```

---

## Controllers Covered

| # | Controller | Where It Lives in a Coke Plant |
|---|------------|-------------------------------|
| 1 | **PID** | Conveyor speed, carbonator pressure, CIP tank temperature, filler valve position, coupled tank levels |
| 2 | **Cascade** | Pasteuriser temperature (inner flow / outer temp), syrup tank level, CIP heat-exchanger |
| 3 | **Ratio** | Syrup-to-water blending at the specified 5.7:1 ratio, Brix trim, product changeover |
| 4 | **Feedforward** | CIP disturbance rejection, production-rate changes, pasteuriser FB+FF combined |

---

## Problem Index

### 01 — PID Control

| # | Problem | Plant | Difficulty |
|---|---------|-------|-----------|
| 1 | `problem1_conveyor_speed.m` | Conveyor belt motor | ⭐ *(pending)* |
| 2 | `problem2_carbonator_pressure.m` | CO₂ carbonator vessel | ⭐⭐ *(pending)* |
| 3 | `problem3_cip_tank_temperature.m` | CIP wash tank | ⭐⭐⭐ *(pending)* |
| 4 | `problem4_filler_valve_nonlinear.m` | Filler valve with stiction | ⭐⭐⭐⭐ *(pending)* |
| 5 | `problem5_coupled_tanks_mimo.m` | Coupled syrup/water tanks (MIMO) | ⭐⭐⭐⭐⭐ *(pending)* |

### 02 — Cascade Control

| # | Problem | Plant | Difficulty |
|---|---------|-------|-----------|
| 1 | `problem1_pasteurizer_basic.m` | HTST pasteuriser | ⭐ *(pending)* |
| 2 | `problem2_syrup_tank_level.m` | Syrup storage tank | ⭐⭐ *(pending)* |
| 3 | `problem3_carbonator_cascade.m` | Carbonation vessel | ⭐⭐⭐ *(pending)* |
| 4 | `problem4_cip_heat_exchanger.m` | CIP heat exchanger | ⭐⭐⭐⭐ *(pending)* |
| 5 | `problem5_pasteurizer_regen.m` | Pasteuriser with regeneration | ⭐⭐⭐⭐⭐ *(pending)* |

### 03 — Ratio Control

| # | Problem | Plant | Difficulty |
|---|---------|-------|-----------|
| 1 | `problem1_basic_syrup_water.m` | Syrup-to-water blender | ⭐ *(pending)* |
| 2 | `problem2_ratio_with_brix_trim.m` | Blender + Brix analyser trim | ⭐⭐ *(pending)* |
| 3 | `problem3_three_component_blend.m` | Three-stream blend (syrup, water, CO₂) | ⭐⭐⭐ *(pending)* |
| 4 | `problem4_product_changeover.m` | Product changeover ramp | ⭐⭐⭐⭐ *(pending)* |
| 5 | `problem5_adaptive_ratio.m` | Adaptive ratio via RLS | ⭐⭐⭐⭐⭐ *(pending)* |

### 04 — Feedforward Control

| # | Problem | Plant | Difficulty |
|---|---------|-------|-----------|
| 1 | `problem1_static_ff_cip.m` | CIP static disturbance FF | ⭐ *(pending)* |
| 2 | `problem2_production_rate_ff.m` | Production-rate change FF | ⭐⭐ *(pending)* |
| 3 | `problem3_ff_fb_pasteurizer.m` | Pasteuriser FB + FF combined | ⭐⭐⭐ *(pending)* |
| 4 | `problem4_dynamic_ff_leadlag.m` | Dynamic FF with lead-lag compensator | ⭐⭐⭐⭐ *(pending)* |
| 5 | `problem5_multivariable_ff_blender.m` | Multivariable static FF blender | ⭐⭐⭐⭐⭐ *(pending)* |

---

## Running the Code

**Requirement:** MATLAB R2019b or later with the **Control System Toolbox**.
No Simulink, no System Identification Toolbox, no Optimization Toolbox.

### Run a single problem

```matlab
% In the MATLAB command window, navigate to the project root and run:
cd('01-pid-control')
run('problem1_conveyor_speed.m')
```

Or use the MATLAB editor Run button. The script auto-creates its `plots/`
subfolder and saves all figures as PNG files there.

### Batch run all implemented problems

```matlab
% From the project root
problems = dir(fullfile('**', 'problem*.m'));
for k = 1:numel(problems)
    run(fullfile(problems(k).folder, problems(k).name));
end
```

---

## Plant Numbers & Where They Come From

- **72 °C pasteurisation** — HTST (High-Temperature Short-Time) hold temperature
  required by FDA 21 CFR Part 131 and Coca-Cola's own QSE standard.
- **4.5 bar carbonation** — gauge pressure in the carbonator at standard
  cola carbonation levels (~3.7 volumes CO₂).
- **5.7 : 1 ratio** — nominal syrup-to-water volumetric ratio for
  Coca-Cola Classic concentrate blending.
- **85 °C CIP** — caustic wash temperature for Clean-In-Place circuits
  (typical 2–4% NaOH at 80–90 °C per EHEDG guidelines).
- **±2 mL filler accuracy** — volumetric fill tolerance for a 355 mL
  can line operating at ~1 500 cans/min.
- **10.5 °Brix finished product** — refractometer reading for finished
  Coca-Cola; consumer perception threshold is ~±0.05 °Brix.

---

## Tuning Methods Used

| Problem | Method | Key Formula / Rule |
|---------|--------|-------------------|
| PID 1 | IMC (λ-tuning) | τ_c = λ chosen as λ ≥ θ |
| PID 2 | Cohen-Coon (FOPDT) | Uses K, τ, θ from step test |
| PID 3 | Skogestad SIMC (SOPDT) | τ_c = max(θ, τ₂) |
| PID 4 | Hand-tuned + dither | Anti-stiction signal injection |
| PID 5 | RGA + decoupler + diagonal PI | Λ = K ⊙ (K⁻¹)ᵀ |
| Cascade 1–3 | IMC inner + IMC outer | 5× bandwidth separation rule |
| Cascade 4 | Positioner PI + SIMC PID (N=5) | Derivative filter included |
| Cascade 5 | Discrete-time with input buffers | ZOH discretisation |
| Ratio 1–3 | PI on flow + ratio block | SP_controlled = R · PV_wild |
| Ratio 4 | Ramp + integral tracking | Bumpless transfer on changeover |
| Ratio 5 | RLS (λ = 0.98) | Online ratio estimation |
| FF 1–3 | Static gain | Kff = −Kd_ss / Kp_ss |
| FF 4 | Lead-lag FF | Kff · (τ_z·s+1)/(τ_p·s+1) |
| FF 5 | Static MIMO FF | Kff = −inv(Kp_ss) · Kd_ss |

---

## Roadmap

Future extensions planned after the 20-problem core is complete:

- **Override / selector control** — low-pressure override on carbonator
- **Split-range control** — heating/cooling valve pair on CIP exchanger
- **Model Predictive Control (MPC)** — coordinated pasteuriser + filler
- **Gain-scheduled PID** — viscosity-varying syrup lines
- **Smith Predictor** — long-dead-time CIP pipework

---

## License

MIT — see [LICENSE](LICENSE).
