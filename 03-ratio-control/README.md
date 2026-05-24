# 03 — Ratio Control

Ratio control maintains a fixed proportional relationship between two flow
streams: the **wild stream** (uncontrolled, measured) and the
**controlled stream** (manipulated). The ratio block computes:

```
SP_controlled = R_desired × PV_wild
```

and a conventional PI flow controller then drives the controlled stream to
that calculated setpoint. If the wild stream suddenly increases, the ratio
block immediately raises the controlled-stream setpoint — the correction
happens before any composition error can accumulate.

**The 5.7 : 1 Coke spec:** Coca-Cola Classic uses a nominal syrup-to-water
volumetric ratio of approximately 5.7 parts finished water to 1 part syrup
concentrate (this ratio varies slightly with market, temperature, and syrup
batch). Maintaining this ratio is directly tied to Brix — the refractometric
measure of dissolved solids. The **consumer perception threshold is ~0.05 °Brix**:
deviations above this are detectable as "too sweet" or "too watery" by
trained tasters, which is why ratio control (not just level control) is
mandatory on every blending line.

**Why ratio matters over simple proportioning:**
- Upstream supply-pressure fluctuations change both streams simultaneously;
  a pure setpoint approach cannot react fast enough.
- Ratio control is inherently feedforward to wild-stream disturbances.
- Combined with a downstream Brix trim loop, ratio + trim achieves both
  speed (from the ratio block) and accuracy (from integral Brix correction).

---

## Problems

| # | File | Difficulty | Status |
|---|------|-----------|--------|
| 1 | `problem1_basic_syrup_water.m` | ⭐ EASY | *(pending)* |
| 2 | `problem2_ratio_with_brix_trim.m` | ⭐⭐ MEDIUM | *(pending)* |
| 3 | `problem3_three_component_blend.m` | ⭐⭐⭐ HARD | *(pending)* |
| 4 | `problem4_product_changeover.m` | ⭐⭐⭐⭐ HARD | *(pending)* |
| 5 | `problem5_adaptive_ratio.m` | ⭐⭐⭐⭐⭐ VERY HARD | *(pending)* |

---

## What Each Problem Will Show

**Problem 1 — Basic Syrup-to-Water Blending**
Two first-order flow processes (syrup and water lines) are connected through
a simple 5.7:1 ratio block. A PI controller is tuned on the controlled
(syrup) stream using IMC. The problem shows how a step change in wild-stream
(water) flow propagates and is automatically tracked without any composition
feedback.

**Problem 2 — Ratio with Brix Trim**
The ratio block output is trimmed by a slow PI outer loop that closes on a
downstream Brix analyser (60-second measurement lag). The problem demonstrates
the two-degree-of-freedom structure: the ratio block provides fast
disturbance rejection while integral Brix control removes the steady-state
offset caused by syrup concentration variation between batches.

**Problem 3 — Three-Component Blend**
Three streams are blended: water (wild), syrup (ratio-controlled), and
CO₂-saturated water (second ratio-controlled). The problem sets up two ratio
blocks from the single wild stream and shows how interaction between the
two controlled streams must be avoided through careful setpoint sequencing.

**Problem 4 — Product Changeover**
A ramp transition between two product ratios (e.g., Coke Classic → Coke Zero)
is implemented with a linear ramp generator and integral tracking (bumpless
transfer). The problem shows why a step change in the ratio setpoint creates
a momentary Brix spike and why a ramp with a hold-and-track strategy is
the industry-standard approach.

**Problem 5 — Adaptive Ratio via RLS**
The true syrup-to-Brix gain drifts with syrup batch concentration. A
Recursive Least-Squares estimator (forgetting factor λ = 0.98) continuously
updates the ratio setpoint to compensate. The problem demonstrates online
parameter estimation integrated into a closed-loop ratio scheme, and shows
the trade-off between tracking speed (low λ) and noise sensitivity (high λ).

---

## Output

| Script | Output file | Note |
|--------|-------------|------|
| `problem1_basic_syrup_water.m` | `plots/p1_ratio_tracking.png` | *(generated on run)* |
| `problem2_ratio_with_brix_trim.m` | `plots/p2_brix_trim.png` | *(generated on run)* |
| `problem3_three_component_blend.m` | `plots/p3_three_stream.png` | *(generated on run)* |
| `problem4_product_changeover.m` | `plots/p4_changeover_ramp.png` | *(generated on run)* |
| `problem5_adaptive_ratio.m` | `plots/p5_rls_adaptive.png` | *(generated on run)* |

---

## Reading the Console Output

```
=============================================
  Problem 1 — Basic Syrup/Water Ratio
=============================================
  Desired ratio R   =   5.700
  PI (syrup flow)
    Kp              =   0.621
    Ki              =   0.207  (Ti = 3.00 s)
---------------------------------------------
  Ratio error (SS)  =   0.000
  Brix deviation    =   0.000 deg  (target 10.50)
=============================================
```
