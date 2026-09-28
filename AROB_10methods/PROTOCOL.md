# Ten-method comparison — measurement protocol

**Status: written 2026-09-28, before any ten-method run. Commit this before the
first run.** Everything here is fixed in advance so that the metrics cannot be
chosen after seeing results. Nothing in this file may be changed once runs
begin; if something must change, the affected runs are discarded and rerun, and
the change is recorded in the log at the bottom.

Paper: `~/papers/gp7-AROB-10methods`. Harness: this folder. The four-method
study (`~/papers/gp7-AROB`, `../AROB/`) is frozen and is not edited.

---

## 1. Scope

Ten joint-space trajectory-generation methods, one controlled protocol, two
experiments on one simulated Yaskawa GP7.

| # | Method | Family | Jerk | Via-point |
|---|---|---|---|---|
| 1 | LSPB (trapezoidal) | trapezoidal | impulsive | stitched only |
| 2 | Cubic | polynomial | impulsive at ends | stitched only |
| 3 | Quintic | polynomial | bounded | stitched only |
| 4 | Septic | polynomial | bounded, zero at ends | stitched only |
| 5 | Cycloidal | trigonometric | bounded | stitched only |
| 6 | Modified sine | cam | bounded | stitched only |
| 7 | Modified trapezoid | cam | bounded | stitched only |
| 8 | Double-S | jerk-limited trapezoidal | bounded by `j_max` | stitched only |
| 9 | Clamped cubic B-spline | spline | bounded | non-stopping |
| 10 | LSPB with via-point blends | trapezoidal | impulsive | non-stopping |

Ruckig and TOPP-RA are **not** among the ten. They are used as cross-checks and
as the time-optimal floor (§7).

The ten are not claimed to be the most important methods. They are claimed to
represent distinct families, which is what the taxonomy table reports.

## 2. Shared limits

| Limit | Value | Applies to |
|---|---|---|
| `v_max` | 0.5 rad/s | all ten |
| `a_max` | 1.0 rad/s² | all ten |
| `j_max` | 445.0 °/s³ = 7.7667 rad/s³ | methods with bounded jerk only (§3) |

`v_max` and `a_max` are unchanged from the four-method study, so the retained
segments are directly comparable.

`j_max` is set to the largest commanded jerk in the four-method study (the
stitched quintic in Experiment 2, 445.0 °/s³). Chosen this way, the jerk term
does not bind for any retained method, so no retained duration changes. Verified
on the Experiment 1 segment (|Δq| = 0.47319 rad):

| Method | `T_vel` | `T_acc` | `T_jerk` | binding | jerk binds? |
|---|---|---|---|---|---|
| cubic | 1.4196 | 1.6850 | 0.9009 | 1.6850 (acc) | no |
| quintic | 1.7745 | 1.6529 | 1.5405 | 1.7745 (vel) | no |
| septic | 2.0702 | 1.8856 | 1.4734 | 2.0702 (vel) | no |
| cycloidal | 1.8928 | 1.7243 | 1.3398 | 1.8928 (vel) | no |

This must be re-verified for methods 6–8 once implemented. If the jerk term
binds for any method, that is a real result and is reported, not tuned away.

## 3. Timing rule

For single-segment (point-to-point) methods with shape function `s(τ)`:

```
T  >=  max(  |s'|max·|Δq| / v_max ,
             sqrt( |s''|max·|Δq| / a_max ) ,
             ( |s'''|max·|Δq| / j_max )^(1/3)  )      <- jerk term only if |s'''|max finite
```

and each method is given the **shortest** `T` satisfying it. Joints are
synchronised: the largest displacement paces the segment and fixes `T`.

**The jerk term is applied only where `|s'''|max` is finite.** LSPB has
piecewise-constant acceleration and method 2 (cubic) steps in acceleration at
both endpoints, so both have impulsive jerk and satisfy no finite `j_max`. This
is a property of the methods, not an exemption granted to them, and the paper
must say so rather than implying all ten respect all three limits. Double-S is
the jerk-limited counterpart of LSPB and exists precisely to bound it.

LSPB timing follows its own blend rule (unchanged from the four-method study).

**Shape-function constants** (verified 2026-09-28 by exact symbolic
differentiation, not finite differences, against closed forms where they exist):

| Method | `s(τ)` | `\|s'\|max` | `\|s''\|max` | `\|s'''\|max` |
|---|---|---|---|---|
| cubic | 3τ²−2τ³ | 3/2 = 1.50000 | 6 | 12 |
| quintic | 10τ³−15τ⁴+6τ⁵ | 15/8 = 1.87500 | 10/√3 = 5.77350 | 60 |
| septic | 35τ⁴−84τ⁵+70τ⁶−20τ⁷ | 35/16 = 2.18750 | 7.51319 | 105/2 = 52.5 |
| cycloidal | τ − sin(2πτ)/2π | 2 | 2π = 6.28319 | 4π² = 39.47842 |
| modified sine | cam | [VERIFY] | [VERIFY] | [VERIFY] |
| modified trapezoid | cam | [VERIFY] | [VERIFY] | [VERIFY] |

Constants for methods 6 and 7 are computed by script and checked against a cited
cam-design reference before use. They are **not** copied from a table.

**B-spline (method 9) duration.** Knot times are set in proportion to each
segment's largest joint displacement, then scaled uniformly until the first of
`v_max`/`a_max` binds exactly. This is the shortest duration **for that
allocation of time between segments**, not over all allocations, and the paper
states it that way. On the four-method via-point path this leaves peak
acceleration at 92 % of `a_max` (velocity binds).

**Method 10 blend rule.** Parabolic blends at interior waypoints, blend time set
by `a_max`: `t_blend = |Δv| / a_max`, where `Δv` is the change in the pacing
joint's cruise velocity across the waypoint. Fixed here before any run.

## 4. What is sent to the controller

| Setting | Value |
|---|---|
| Sample spacing `dt` | **0.01 s, all methods, both experiments** |
| Point contents | position + velocity + acceleration, all six arm joints |
| Interface | `FollowJointTrajectory` action to `/arm_controller` |
| Controller | `joint_trajectory_controller`, 1 kHz, re-interpolates between points |
| Physics | Gazebo Harmonic, Bullet Featherstone, 1 ms step |

`dt = 0.01` matches the four-method Experiment 1. **The four-method Experiment 2
used `dt = 0.02`**, so it is not consistent with Experiment 1 and is rerun (§6).

Velocities and accelerations come from the analytic derivatives of `s`, never
from finite differences of the sampled positions.

## 5. Metrics

Fixed here, before any run.

**Commanded** (deterministic, computed once per method, no repeats):
peak jerk; integrated squared jerk (°² s⁻⁵); peak velocity; peak acceleration;
duration; number of zero-velocity instants.

**Measured in simulation** (30 repeats, mean ± SD):
- **Tracking RMSE (mm)** — commanded joint positions against `/joint_states`,
  mapped through forward kinematics to a Cartesian error at the `tcp`.
- **Execution time (s)** — wall clock across the action call. This includes two
  DDS round-trips, so it bounds rather than isolates controller overhead, and
  the paper says so.
- **Dynamic torque (N m)** — see below.

**Derived:**
- **% above time-optimal duration**, from TOPP-RA along the same geometric path.
- **Spearman rank correlation** of tracking RMSE against commanded peak
  velocity, peak acceleration and peak jerk, across the eight distinct
  Experiment 1 profiles. Reported as correlation only, with no causal claim.
  This replaces the four-method study's "holding speed fixed" argument, which
  compared profiles at equal *peak* velocity and silently treated that as equal
  mean speed. It is not, and that argument is dropped.

### Torque: definition fixed before any run

Peak torque in the four-method study was gravity-dominated and did not
discriminate: spread across methods 2.3 N m against a within-method SD of about
19 N m on identical commands. It is replaced by **Option A, dynamic torque**:

```
tau_dyn(t)  =  tau(t)  -  tau_static(q(t))
```

- `tau(t)` is the effort reported by the hardware interface for the six arm
  joints, logged from `/joint_states`.
- `tau_static(q)` is the gravity torque at the logged configuration, computed
  from the **same URDF inertial model Gazebo loads** (`gp7_macro.xacro`: seven
  links, per-link mass and COM origin, 34.001 kg total) by the
  Jacobian-transpose method with zero velocity and acceleration. No new
  dependency: the project's own forward kinematics supplies the link frames.
- **Reported:** per-joint RMS of `tau_dyn` over the run, aggregated as the
  **maximum over the six joints**. Peak `|tau_dyn|` is secondary.
- Gravity depends only on configuration, so subtracting it removes the component
  that is identical along the same path for every method. The approximate mass
  distribution biases absolute values but not the comparison between methods,
  and the paper states this.
- The gravity model is validated before the first run by checking `tau_dyn ≈ 0`
  on a slow quasi-static move, and the residual is recorded here.

**This requires logging the effort time series, which the four-method harness
discarded** — it wrote only a scalar `peak_torque_Nm` per run. The new harness
saves per-sample time, position and effort for every run.

The paper discloses that peak torque was examined in the four-method study,
found non-discriminating for the reason above, and replaced before these runs.

## 6. Experiments and repeats

**Experiment 1 — point-to-point.** Approach-to-grasp descent, the same segment
as the four-method study: pacing joint 5, |Δq| = 27.11°, zero velocity at both
ends. Methods 1–8 give eight distinct profiles. Methods 9 and 10 are generated
and **verified numerically to be identical** to the cubic and to LSPB
respectively on two waypoints; they are reported as identical, are not executed
as separate data points, and are excluded from the tests and the rank
correlation.

**Experiment 2 — via-point.** grasp → lift → transit → place-approach, the same
four waypoints as the four-method study. Methods 9 and 10 pass through the
interior waypoints without stopping; methods 1–8 are stitched with a forced stop
at both interior waypoints. All ten are distinct.

**Repeats: 30 per profile.** Experiment 1: 8 × 30 = 240 runs. Experiment 2:
10 × 30 = 300 runs. Total 540.

> **Everything is rerun. Nothing is carried over from the four-method study.**
> The plan asked to retain the LSPB, cubic and quintic results and "the
> 30-repeat protocol", but there was no 30-repeat protocol: the logged data is
> 9 runs per method in Experiment 1 and 9–10 in Experiment 2
> (`../AROB/results_experiment*.csv`, 27 and 38 rows). Retaining them is also
> blocked independently: the torque definition changed and no effort time series
> was saved, and Experiment 2 used a different `dt`. Mixing 9-run and 30-run
> methods in one comparison table would not be defensible, so all ten methods
> are run fresh under this protocol.

Each run re-establishes the start pose before the measured motion. The repeats
are **not independent physical trials**: every run executes an identical
command, so they sample timing jitter in the ROS 2 stack (asynchronous action
round-trip, start pose re-established only to 1e-3 rad). The paper says this
rather than implying physical replication.

## 7. Cross-checks and the time-optimal floor

- **Ruckig** with the same `v_max`, `a_max`, `j_max` should reproduce double-S on
  a rest-to-rest segment. Agreement is a correctness check on method 8.
- **TOPP-RA** with `v_max` and `a_max` only should reproduce LSPB on a
  rest-to-rest segment. Agreement is a correctness check on method 1.
- **TOPP-RA along the same geometric path** gives the minimum admissible
  duration, and every method's duration is reported as "% above optimal".

Both are pinned to a specific version, recorded here on install, and neither is
counted among the ten or entered into the statistical contrasts.

**Both cross-checks have been run (2026-09-28) and pass**, before any simulation
run, by `crosscheck_ruckig.py` and `crosscheck_toppra.py`:

| Check | Result |
|---|---|
| Ruckig vs our double-S (ruckig 0.19.4) | durations equal to 1e-17 s; positions to 1.1e-16 rad, over all three limit regimes |
| TOPP-RA vs LSPB (toppra, straight-line path) | durations agree to 1.4e-6 s, positions to 6.3e-7 rad |

Ruckig's `update()` returns the state at `t+dt`, not `t`. Labelling it `t`
shifts the profile by one sample and produces a `v_max*dt` position error that
looks like a real disagreement; the first run of the check reported exactly that
before the alignment was corrected.

## 8. Statistics — pre-registered contrasts

Forty-five pairwise tests per metric would be uninterpretable. The following
contrasts are fixed now; **only these are tested**, Holm-corrected within each
metric, by Welch's t-test on the per-run values.

**Experiment 1 (tracking RMSE, dynamic torque):**

| ID | Contrast | Question |
|---|---|---|
| C1 | double-S vs LSPB | does bounding jerk change tracking? |
| C2 | cubic vs quintic vs septic (3 pairs) | does polynomial order change tracking? |
| C3 | cycloidal vs quintic | trigonometric vs polynomial at the same continuity class |
| C4 | modified sine vs modified trapezoid | within the cam family |

**Experiment 2 (execution time, tracking RMSE, dynamic torque):**

| ID | Contrast | Question |
|---|---|---|
| C5 | B-spline vs stitched cubic | non-stopping vs stitched, spline family |
| C6 | LSPB-blends vs stitched LSPB | non-stopping vs stitched, trapezoidal family |
| C7 | B-spline vs LSPB-blends | between the two non-stopping methods |

Everything else is reported descriptively (mean ± SD) with no significance
claim. Spearman correlations per §5 are reported with their coefficients.

Anything found by inspecting the results and not listed above is labelled
**exploratory** in the paper and carries no p-value.

## 9. Guardrails

- No method is called best, second best or worst. No scores, no ranking. The
  Discussion maps requirements to characteristics only.
- No novelty claim for any method. All ten are standard.
- The paper is a comparison or evaluation, never a "review": that would imply
  broader literature coverage than 4–6 pages can carry.
- No number enters the paper except through a macro generated from logged data.
- Unverifiable reference or number → `[VERIFY]`, never a guess.

## 9b. Pre-run findings (2026-09-28, before any run)

Found while preparing `CHAT_BRIEFING.md`. None changes a fixed value; items 4-7
are decisions to record here, with a date, before run 1.

1. **`j_max` binds for modified sine on Exp 2 segment 1** (needs 460 deg/s^3 at
   its velocity-limited duration; T = 1.5826 s instead of 1.5595 s). It binds
   nowhere else. Report it as a result.
2. **The 445.0 deg/s^3 basis for `j_max` does not reproduce.** Analytically, on
   the waypoints that ran, the four-method stitched quintic peaks at 331.9 and
   the cubic at 70.3 (reported 445.0 and 84.0). Stitched durations do reproduce.
   `j_max` = 445.0 stays as a chosen limit, not a derived one.
3. **Exp 2 carries a grasped box.** The gravity model must include the gripper
   links and the box, or the box's weight stays inside `tau_dyn`.
4. **Tracking-RMSE alignment is biased by goal-acceptance latency.** The log's
   t = 0 precedes the controller's start, giving an error of about velocity x
   latency, which grows with commanded speed. Recommended fix: schedule the
   start with `header.stamp` and align to it.
5. **Blended LSPB is underspecified:** whether it passes exactly through
   interior waypoints, and how its segment durations are set.
6. **TOPP-RA floor in Exp 2:** methods follow different geometric paths.
   Recommended: each method's floor along its own path.
7. **Spearman jerk ranks:** LSPB's jerk is impulsive. Recommended: rank
   impulsive profiles highest (tied), report with and without them.

## 10. Change log

Any change after the first run is recorded here with the date, the reason, and
which runs were discarded.

| Date | Change | Runs discarded |
|---|---|---|
| 2026-09-28 | Protocol written. No runs yet. | — |
| 2026-09-28 | §12 decisions settled, defaults taken: (1) 30 repeats per profile, 540 runs; (2) tracking alignment by scheduled header.stamp; (3) primary execution time = sim time, wall time also logged; (4) blended LSPB passes exactly through interior waypoints (pseudo-via-points); (5) blended LSPB linear-segment durations: pacing joint cruises at v_max; (6) TOPP-RA floor along each method's own path; (7) impulsive-jerk profiles ranked highest (tied), jerk correlation reported with and without them; (8) pre-positioning always executed, then wait for settling; (9) j_max = 445.0 deg/s^3 kept as a chosen limit | — |
| 2026-09-28 | Added to §8: Mann-Whitney U reported alongside Welch's t for every contrast. Added to phase 6 smoke test: per-run tracking RMSE must not sit on a few discrete levels; if it still does after the alignment fix, investigate before run 1 | — |
| 2026-09-28 | §9b corrected. Item 2: the earlier Exp 2 jerk row (cubic 84.0, quintic 445.0, B-spline 66.7 deg/s^3) was computed on the z=0.14 grasp waypoint, not the z=0.05 one that ran; on the waypoints that ran: 70.3, 331.9, 52.8. The 445.0 basis was therefore an error; j_max stays as a chosen limit. Item 1: without the jerk term, modsine Exp 2 seg 1 is acceleration-limited at 1.5652 s (velocity term 1.5596 s) and needs 460.0 deg/s^3; j_max lengthens it to 1.5826 s | — |
