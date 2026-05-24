# 02 — Cascade Control

Cascade control places a fast **inner (secondary) loop** inside a slower
**outer (primary) loop**. The outer controller's output becomes the
setpoint for the inner controller, so disturbances that enter between the
two measurement points are corrected before they can upset the primary
variable.

**The 5× bandwidth rule:** the inner loop must be at least five times faster
than the outer loop (ω_inner ≥ 5·ω_outer). If this condition is not met, the
two loops interact destructively and performance degrades below what a single
well-tuned PID would achieve.

**When cascade beats single-loop:**
- A measurable intermediate variable exists (e.g., jacket flow rate inside a
  temperature loop).
- The inner process is significantly faster than the outer process.
- The primary disturbance enters between the two sensors.

In Coca-Cola plants cascade is most visible on the HTST pasteuriser: the
outer loop controls product temperature; the inner loop controls hot-water
jacket flow. A single-loop temperature controller reacts only after the
product temperature changes; cascade corrects a supply-pressure upset at the
jacket level, before the thermal lag of the product channel even begins.

---

## Problems

| # | File | Difficulty | Status |
|---|------|-----------|--------|
| 1 | `problem1_pasteurizer_basic.m` | ⭐ EASY | *(pending)* |
| 2 | `problem2_syrup_tank_level.m` | ⭐⭐ MEDIUM | *(pending)* |
| 3 | `problem3_carbonator_cascade.m` | ⭐⭐⭐ HARD | *(pending)* |
| 4 | `problem4_cip_heat_exchanger.m` | ⭐⭐⭐⭐ HARD | *(pending)* |
| 5 | `problem5_pasteurizer_regen.m` | ⭐⭐⭐⭐⭐ VERY HARD | *(pending)* |

---

## What Each Problem Will Show

**Problem 1 — Pasteuriser Basic Cascade**
The simplest cascade configuration: a first-order outer temperature loop and
a first-order inner flow loop. IMC is applied to both, with the inner
closed-loop bandwidth set 5× higher than the outer. The problem demonstrates
how a step disturbance on jacket pressure is rejected at the inner loop
without reaching the product temperature sensor.

**Problem 2 — Syrup Tank Level**
The outer loop controls syrup tank level (integrating process); the inner
loop controls inlet flow rate (fast first-order). IMC is tuned separately for
each loop. The problem shows that integral-mode windup in the outer loop must
be clamped to the inner-loop setpoint limits, and illustrates a practical
anti-windup implementation.

**Problem 3 — Carbonator Cascade**
The carbonator vessel pressure is the outer controlled variable; CO₂
injection flow rate is the inner variable. Both have different dynamics and
the inner loop has dead time. IMC tuning is applied to each. The problem
explores what happens to setpoint tracking when the bandwidth separation rule
is violated (purposely mistuned inner loop as a comparison case).

**Problem 4 — CIP Heat Exchanger**
A more industrial configuration: the inner loop uses a valve-positioner PI
(fast, integrating actuator) and the outer loop uses Skogestad SIMC PID with
derivative filter coefficient N = 5. The problem demonstrates why adding
derivative to the outer loop can improve disturbance rejection without
exciting high-frequency noise on the positioner.

**Problem 5 — Pasteuriser with Regeneration (Discrete-Time)**
The most complex problem: the regeneration section recirculates heat from hot
product back to cold product, creating a thermal feedback path. The cascade
structure is simulated in discrete time using input buffers to model the
transport lag through the regeneration plate. The problem highlights
interaction between the regeneration feedback and the cascade outer loop.

---

## Output

| Script | Output file | Note |
|--------|-------------|------|
| `problem1_pasteurizer_basic.m` | `plots/p1_cascade_response.png` | *(generated on run)* |
| `problem2_syrup_tank_level.m` | `plots/p2_level_cascade.png` | *(generated on run)* |
| `problem3_carbonator_cascade.m` | `plots/p3_bandwidth_comparison.png` | *(generated on run)* |
| `problem4_cip_heat_exchanger.m` | `plots/p4_positioner_pid.png` | *(generated on run)* |
| `problem5_pasteurizer_regen.m` | `plots/p5_discrete_regen.png` | *(generated on run)* |

---

## Reading the Console Output

```
=============================================
  Problem 1 — Pasteuriser Basic Cascade
=============================================
  INNER LOOP (flow)
    Kp_inner    =   2.150
    Ki_inner    =   1.433  (Ti = 1.50 s)
  OUTER LOOP (temperature)
    Kp_outer    =   0.842
    Ki_outer    =   0.140  (Ti = 6.01 s)
---------------------------------------------
  Outer rise time   =  11.4 s
  Outer settle time =  29.2 s  (2%)
  Outer overshoot   =   3.6 %
  Outer SS error    =   0.00 %
=============================================
```
