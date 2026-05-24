# 01 — PID Control

The Proportional-Integral-Derivative (PID) controller is the workhorse of
industrial process control. A single PID loop computes a corrective output
`u(t) = Kp·e + Ki·∫e dt + Kd·ė` and drives the plant toward a setpoint.
In Coca-Cola production lines PID loops govern everything from conveyor belt
speed to vessel pressures and wash-water temperatures — wherever a single
measured variable must track a single setpoint.

---

## Problems

| # | File | Difficulty | Status |
|---|------|-----------|--------|
| 1 | `problem1_conveyor_speed.m` | ⭐ EASY | *(pending)* |
| 2 | `problem2_carbonator_pressure.m` | ⭐⭐ MEDIUM | *(pending)* |
| 3 | `problem3_cip_tank_temperature.m` | ⭐⭐⭐ HARD | *(pending)* |
| 4 | `problem4_filler_valve_nonlinear.m` | ⭐⭐⭐⭐ HARD | *(pending)* |
| 5 | `problem5_coupled_tanks_mimo.m` | ⭐⭐⭐⭐⭐ VERY HARD | *(pending)* |

---

## What Each Problem Will Show

**Problem 1 — Conveyor Belt Speed**
A first-order plus dead-time (FOPDT) DC-motor-driven conveyor is controlled
by a PID tuned via IMC (lambda tuning). The problem illustrates how choosing
the closed-loop time constant λ trades off speed of response against
robustness, and demonstrates zero steady-state error to a step reference.

**Problem 2 — Carbonator Pressure**
The CO₂ carbonator vessel is modelled as an FOPDT process with moderate dead
time. Cohen-Coon tuning is applied, showing how the method uses the
step-test parameters (gain K, time constant τ, dead time θ) to generate
initial Kp, Ti, Td estimates. The problem highlights the effect of
dead-time-to-time-constant ratio on achievable closed-loop performance.

**Problem 3 — CIP Tank Temperature**
The Clean-In-Place wash tank is a second-order overdamped system (SOPDT)
because the heat exchanger and tank thermal masses interact. Skogestad SIMC
tuning is applied to the identified SOPDT model, and the problem demonstrates
how a poorly-tuned integral can cause integrator windup during the
85 °C setpoint ramp.

**Problem 4 — Filler Valve (Nonlinear)**
The fill-valve actuator exhibits static friction (stiction), so the plant is
nonlinear. A baseline PID is hand-tuned, then an anti-stiction dither signal
is added to the controller output. The problem shows limit-cycle oscillation
with stiction, and how a small high-frequency dither signal suppresses it
while keeping fill volume within ±2 mL.

**Problem 5 — Coupled Tanks MIMO**
Syrup and water feed tanks interact through a shared downstream pressure.
The 2×2 transfer-function matrix is analysed using the Relative Gain Array
(RGA). A static decoupler is designed and a diagonal PI controller is tuned
on the decoupled system. The problem illustrates why loop pairing matters and
how decoupling reduces interaction penalties.

---

## Output

The following files will be generated inside `plots/` when each script is run:

| Script | Output file | Note |
|--------|-------------|------|
| `problem1_conveyor_speed.m` | `plots/p1_step_response.png` | *(generated on run)* |
| `problem2_carbonator_pressure.m` | `plots/p2_step_response.png` | *(generated on run)* |
| `problem3_cip_tank_temperature.m` | `plots/p3_step_and_windup.png` | *(generated on run)* |
| `problem4_filler_valve_nonlinear.m` | `plots/p4_stiction_comparison.png` | *(generated on run)* |
| `problem5_coupled_tanks_mimo.m` | `plots/p5_mimo_decoupled.png` | *(generated on run)* |

---

## Reading the Console Output

Every script prints a summary block like this at the end:

```
=============================================
  Problem 1 — Conveyor Speed PID Results
=============================================
  Kp          =   1.842
  Ki          =   0.307  (Ti = 6.00 s)
  Kd          =   0.000  (Td = 0.00 s)
---------------------------------------------
  Rise time   =   2.31 s
  Settle time =   6.87 s  (2%)
  Overshoot   =   4.2 %
  SS error    =   0.00 %
=============================================
```

The exact gains and metrics will vary once the scripts are implemented.
