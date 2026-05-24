# 04 — Feedforward Control

Feedforward (FF) control measures a **disturbance** before it reaches the
process and applies a pre-computed correction to cancel its effect. The ideal
static feedforward gain is derived directly from steady-state process gains:

```
Gff(s) = -Gd(s) / Gp(s)
```

where `Gd` is the disturbance-to-output transfer function and `Gp` is the
manipulated-input-to-output transfer function. In practice, a static gain
`Kff = -Kd_ss / Kp_ss` is a good first approximation, and a dynamic lead-lag
element is added when the disturbance and manipulated-variable paths have
different time constants or dead times.

**Realisability condition:** The feedforward compensator `Gff(s)` is
physically realisable only if its relative degree is ≥ 0. A pure differentiator
(relative degree −1) cannot be implemented and must be approximated with a
lead-lag filter.

**FF is a complement to FB — never a replacement.**
Feedforward corrects for *measured* disturbances but cannot handle
*unmeasured* disturbances, model errors, or setpoint changes. A pure
feedforward system has no guarantee of zero steady-state error. In every
problem in this folder a feedback loop remains active alongside the
feedforward path. The combination gives both fast disturbance rejection (FF)
and accurate steady-state regulation (FB integral action).

---

## Problems

| # | File | Difficulty | Status |
|---|------|-----------|--------|
| 1 | `problem1_static_ff_cip.m` | ⭐ EASY | *(pending)* |
| 2 | `problem2_production_rate_ff.m` | ⭐⭐ MEDIUM | *(pending)* |
| 3 | `problem3_ff_fb_pasteurizer.m` | ⭐⭐⭐ HARD | *(pending)* |
| 4 | `problem4_dynamic_ff_leadlag.m` | ⭐⭐⭐⭐ HARD | *(pending)* |
| 5 | `problem5_multivariable_ff_blender.m` | ⭐⭐⭐⭐⭐ VERY HARD | *(pending)* |

---

## What Each Problem Will Show

**Problem 1 — Static FF on CIP Circuit**
A step supply-temperature disturbance enters the CIP tank. A static
feedforward gain `Kff = -Kd_ss / Kp_ss` is added to the existing PID output.
The problem compares three responses: no FF (slowest), static FF alone
(fast but offset remains), and PID + static FF (fast and zero steady-state
error). This is the clearest illustration of why FF and FB must coexist.

**Problem 2 — Production-Rate Feedforward**
As line speed increases, the heat load on the pasteuriser grows
proportionally. A production-rate signal (measured conveyor encoder) is used
as the feedforward disturbance. The static FF gain is computed from the
steady-state energy balance. The problem demonstrates how a measured
throughput signal can pre-load the temperature controller before any
temperature deviation appears.

**Problem 3 — FB + FF on the Pasteuriser**
A combined feedback PI (tuned via SIMC) and static feedforward path are
implemented on the full HTST pasteuriser model. Two disturbances are applied
simultaneously: a supply-temperature step and a flow-rate step. The problem
shows disturbance rejection from each path and quantifies the improvement
in integrated absolute error (IAE) versus the feedback-only baseline.

**Problem 4 — Dynamic FF with Lead-Lag**
The disturbance path has a longer dead time than the manipulated-variable
path, so a static `Kff` over-corrects in the short term. A lead-lag
compensator `Kff·(τ_z·s+1)/(τ_p·s+1)` is designed to match the dynamic
mismatch. The problem shows the residual error with static FF, then the
dramatic improvement when the lead-lag is correctly parameterised.

**Problem 5 — Multivariable Static FF Blender**
A 2×2 blending system has two manipulated variables (syrup flow and water
flow) and two controlled variables (Brix and total flow). Two measured
disturbances enter (upstream pressure on each supply line). The static MIMO
feedforward matrix `Kff = -inv(Kp_ss) · Kd_ss` is computed and combined
with a diagonal PI feedback controller. The problem shows how MIMO
feedforward decouples the disturbance responses simultaneously.

---

## Output

| Script | Output file | Note |
|--------|-------------|------|
| `problem1_static_ff_cip.m` | `plots/p1_ff_comparison.png` | *(generated on run)* |
| `problem2_production_rate_ff.m` | `plots/p2_production_ff.png` | *(generated on run)* |
| `problem3_ff_fb_pasteurizer.m` | `plots/p3_combined_ff_fb.png` | *(generated on run)* |
| `problem4_dynamic_ff_leadlag.m` | `plots/p4_leadlag_ff.png` | *(generated on run)* |
| `problem5_multivariable_ff_blender.m` | `plots/p5_mimo_ff.png` | *(generated on run)* |

---

## Reading the Console Output

```
=============================================
  Problem 1 — Static FF on CIP Circuit
=============================================
  Kff (static)      =  -0.714
  PID (feedback)
    Kp              =   1.250
    Ki              =   0.417  (Ti = 3.00 s)
    Kd              =   0.000
---------------------------------------------
  FB only  IAE      =  47.23
  FF only  IAE      =  12.08  (no SS correction)
  FF+FB    IAE      =   6.41
  FF+FB  settle (s) =   8.9
=============================================
```
