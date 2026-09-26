"""
Experiment 1 -- LSPB vs cubic vs quintic on a single must-stop segment
(approach -> grasp), same v_max/a_max, each at its shortest admissible
T (AROB paper section 5.1/5.3, Fig. 3(a)).

Reads AROB/lspb.json (real, FK-verified waypoints). Outputs:
  - a 3-panel comparison plot (position, velocity, acceleration)
    matching Fig. 3(a) of the draft, for the pacing joint
  - a summary table (time, peak v, peak a, peak jerk where meaningful)
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import lspb_full_profile as lfp
import cubic_trajectory as ct
import quintic_trajectory as qt

V_MAX = 0.5   # rad/s -- TODO: confirm final value for paper Table 1
A_MAX = 1.0   # rad/s^2 -- TODO: confirm final value for paper Table 1

with open("lspb.json") as f:
    d = json.load(f)
q0 = np.array(d["q0"])
qf = np.array(d["qf"])

delta = qf - q0
pacing_idx = int(np.argmax(np.abs(delta)))
print(f"pacing joint index: {pacing_idx}  |delta_q| = {np.rad2deg(abs(delta[pacing_idx])):.3f} deg")

# ---- LSPB ----
ts_l, pos_l, vel_l, acc_l, tb_l, tf_l = lfp.lspb_full_profile(q0, qf, V_MAX, A_MAX, dt=0.001)
p_l, v_l, a_l = pos_l[:, pacing_idx], vel_l[:, pacing_idx], acc_l[:, pacing_idx]

# ---- Cubic ----
T_c = ct.segment_duration(delta[pacing_idx], V_MAX, A_MAX)
ts_c, pos_c, vel_c, acc_c = ct.cubic_full_profile(q0, qf, T_c, dt=0.001)
p_c, v_c, a_c = pos_c[:, pacing_idx], vel_c[:, pacing_idx], acc_c[:, pacing_idx]

# ---- Quintic ----
T_q = qt.segment_duration(delta[pacing_idx], V_MAX, A_MAX)
ts_q, pos_q, vel_q, acc_q, jerk_q = qt.quintic_full_profile(q0, qf, T_q, dt=0.001)
p_q, v_q, a_q, j_q = pos_q[:, pacing_idx], vel_q[:, pacing_idx], acc_q[:, pacing_idx], jerk_q[:, pacing_idx]

# also get cubic jerk (constant within segment, = 6*a3, well-defined here since single segment)
delta_pace = delta[pacing_idx]
a3_cubic = -2 * delta_pace / T_c**3
jerk_c_const = 6 * a3_cubic
print(f"cubic constant jerk within segment: {np.rad2deg(jerk_c_const):.3f} deg/s^3")

# ---- Table ----
print("\n=== Summary Table (pacing joint) ===")
print(f"{'method':>8} {'T(s)':>8} {'peak_v(deg/s)':>15} {'peak_a(deg/s^2)':>17} {'peak_jerk(deg/s^3)':>20}")
print(f"{'LSPB':>8} {tf_l:>8.4f} {np.rad2deg(np.max(np.abs(v_l))):>15.3f} {np.rad2deg(np.max(np.abs(a_l))):>17.3f} {'unbounded (impulsive)':>20}")
print(f"{'Cubic':>8} {T_c:>8.4f} {np.rad2deg(np.max(np.abs(v_c))):>15.3f} {np.rad2deg(np.max(np.abs(a_c))):>17.3f} {np.rad2deg(jerk_c_const):>20.3f}")
print(f"{'Quintic':>8} {T_q:>8.4f} {np.rad2deg(np.max(np.abs(v_q))):>15.3f} {np.rad2deg(np.max(np.abs(a_q))):>17.3f} {np.rad2deg(np.max(np.abs(j_q))):>20.3f}")

# ---- Plot: 3-panel, matching Fig 3(a) style ----
fig, axes = plt.subplots(3, 1, figsize=(6, 8), sharex=False)

axes[0].plot(ts_l, np.rad2deg(p_l), label="Trapezoidal (LSPB)")
axes[0].plot(ts_c, np.rad2deg(p_c), label="Cubic")
axes[0].plot(ts_q, np.rad2deg(p_q), label="Quintic")
axes[0].set_ylabel("Pos. (deg)")
axes[0].legend(fontsize=8)

axes[1].plot(ts_l, np.rad2deg(v_l))
axes[1].plot(ts_c, np.rad2deg(v_c))
axes[1].plot(ts_q, np.rad2deg(v_q))
axes[1].set_ylabel("Vel. (deg/s)")

axes[2].plot(ts_l, np.rad2deg(a_l))
axes[2].plot(ts_c, np.rad2deg(a_c))
axes[2].plot(ts_q, np.rad2deg(a_q))
axes[2].set_ylabel("Acc. (deg/s$^2$)")
axes[2].set_xlabel("Time (s)")

plt.tight_layout()
plt.savefig("fig_experiment1.png", dpi=150)
print("\nwrote fig_experiment1.png")
