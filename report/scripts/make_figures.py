"""Generate the figures used in report/report.tex from the project's own code.

Run from anywhere (needs numpy + matplotlib, no ROS):
    /usr/bin/python3 report/scripts/make_figures.py
Figures are written to report/figures/.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
PKG = os.path.join(REPO, "src", "gp7_pick_place", "gp7_pick_place")
OUT = os.path.join(REPO, "report", "figures")
sys.path.insert(0, PKG)
os.makedirs(OUT, exist_ok=True)

import kinematics as kin   # noqa: E402
import trajectory as traj  # noqa: E402
import jacobian as jac     # noqa: E402

# Same constants as run_pick_place.py (not imported: that module needs rclpy)
V_MAX, ACCEL, DT = 0.5, 1.0, 0.05
WAYPOINTS_XYZ = [
    [0.55, -0.25, 0.30],
    [0.55, -0.25, 0.05],
    [0.55, -0.25, 0.30],
    [0.50, 0.50, 0.30],
    [0.50, 0.50, 0.325],
    [0.50, 0.50, 0.330],
]
GRIPPER_ACTIONS = {1: 0.75, 4: 0.0}
R_DOWN = np.diag([1.0, -1.0, -1.0])
GRIPPER_TIME = 2.0

plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                     "savefig.dpi": 200, "savefig.bbox": "tight"})


def solve_waypoints():
    sols, prev = [], None
    for xyz in WAYPOINTS_XYZ:
        T = np.eye(4); T[:3, :3] = R_DOWN; T[:3, 3] = xyz
        q = kin.inverse_kinematics_tcp_continuous(T, previous_solution=prev)
        sols.append(q); prev = q
    return sols


def tcp_pose(q):
    T = kin.forward_kinematics(q)
    p = T[:3, 3] + kin.L_TCP * T[:3, 2]
    return p, T[:3, :3]


# --------------------------------------------------------------------------
# 1. Coordinate frames at the zero pose, computed from the D-H model
# --------------------------------------------------------------------------
def _proj(p):
    """Cabinet (oblique) projection: X right, Z up, Y receding at 35 deg, half scale."""
    k, ang = 0.5, np.deg2rad(35)
    return np.array([p[0] + k * p[1] * np.cos(ang), p[2] + k * p[1] * np.sin(ang)])


def fig_frames():
    Ts = jac.all_frame_transforms([0.0] * 6)
    fig, ax = plt.subplots(figsize=(6.4, 5.4))
    origins = [T[:3, 3] for T in Ts]
    tcp = origins[6] + kin.L_TCP * Ts[6][:3, 2]
    chain = np.array([_proj(p) for p in origins + [tcp]])
    ax.plot(chain[:, 0], chain[:, 1], "-", color="0.6", lw=6, alpha=0.5, solid_capstyle="round", zorder=1)
    # world axes at the base
    for v, name in ((np.array([0.18, 0, 0]), "X"), (np.array([0, 0.30, 0]), "Y"), (np.array([0, 0, 0.12]), "Z")):
        a, b = _proj(np.zeros(3)), _proj(v)
        ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="->", color="0.5", lw=0.8, ls="--"))
        ax.text(*(b + 0.012), f"world {name}", color="0.45", fontsize=7)
    L = 0.075
    cols = {"x": "tab:red", "y": "tab:green", "z": "tab:blue"}
    # Frames that share an origin are drawn displaced vertically and joined by
    # a dotted leader, so each triad stays readable.
    draw_at = {i: origins[i].copy() for i in range(7)}
    draw_at[1] = origins[1] + np.array([-0.17, 0, 0])
    draw_at[2] = origins[2] + np.array([-0.17, 0, 0])
    draw_at[3] = origins[3] + np.array([0.0, 0, 0.13])
    draw_at[4] = origins[4] + np.array([0.0, 0, -0.17])
    draw_at[5] = origins[5] + np.array([0.0, 0, 0.13])
    draw_at[6] = origins[6] + np.array([0.06, 0, -0.20])
    for i, T in enumerate(Ts):
        o, d0 = _proj(origins[i]), _proj(draw_at[i])
        ax.plot(*o, "ko", ms=3.5, zorder=4)
        if not np.allclose(o, d0):
            ax.plot([o[0], d0[0]], [o[1], d0[1]], ":", color="0.35", lw=0.8)
        for k, name in enumerate("xyz"):
            tip = _proj(draw_at[i] + T[:3, k] * L)
            ax.annotate("", xy=tip, xytext=d0,
                        arrowprops=dict(arrowstyle="-|>", color=cols[name], lw=1.4, mutation_scale=9))
            ax.text(*(tip + (tip - d0) * 0.18), f"${name}_{i}$", color=cols[name], fontsize=8,
                    ha="center", va="center")
    p = _proj(tcp)
    ax.plot(*p, "k*", ms=8)
    ax.text(p[0] - 0.01, p[1] - 0.045, "TCP", fontsize=8)
    for i, txt, off in ((0, "$o_0$", (-0.05, -0.03)), (1, "$o_1$", (0.01, -0.035)),
                        (2, "$o_2$", (0.015, -0.03)), (3, "$o_3$", (0.015, 0.01)),
                        (4, "$o_4\\equiv o_5$\n(wrist centre)", (-0.05, -0.065)),
                        (6, "$o_6$ (tool0)", (-0.06, 0.03))):
        q = _proj(origins[i])
        ax.text(q[0] + off[0], q[1] + off[1], txt, fontsize=8)
    ax.set_xlim(-0.30, 0.92); ax.set_ylim(-0.06, 1.02)
    ax.set_aspect("equal"); ax.grid(False)
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title("D-H frames at $q = 0$, computed from the D-H table (oblique view)", fontsize=9, pad=14)
    fig.savefig(os.path.join(OUT, "frames_zero_pose.png"))
    plt.close(fig)


# --------------------------------------------------------------------------
# 2. LSPB profile of the transit segment (Box B held), analytic derivatives
# --------------------------------------------------------------------------
def lspb_derivs(t, q0, qf, tb, tf):
    if q0 == qf:
        return 0.0, 0.0
    V = (qf - q0) / (tf - tb); a = V / tb
    if t < tb:
        return a * t, a
    if t <= tf - tb:
        return V, 0.0
    return a * (tf - t), -a


def fig_lspb(sols):
    q0, qf = sols[2], sols[3]   # lift -> above Box A (Box B in the gripper)
    dq = [qf[i] - q0[i] for i in range(6)]
    p = max(range(6), key=lambda i: abs(dq[i]))
    tb, tf = traj.segment_timing(dq[p], V_MAX, ACCEL)
    t = np.linspace(0, tf, 1200)
    fig, axs = plt.subplots(3, 1, figsize=(6.4, 6.0), sharex=True)
    for i in range(6):
        if abs(dq[i]) < 1e-9:
            continue
        pos = [np.rad2deg(traj.lspb_position(tt, q0[i], qf[i], tb, tf)) for tt in t]
        vel, acc = zip(*[lspb_derivs(tt, q0[i], qf[i], tb, tf) for tt in t])
        lab = f"$q_{i+1}$"
        axs[0].plot(t, pos, label=lab)
        axs[1].plot(t, np.rad2deg(vel), label=lab)
        axs[2].plot(t, np.rad2deg(acc), label=lab)
    # the 20 Hz samples actually sent to the controller (pacing joint)
    pts = traj.generate_segment_trajectory(list(q0), list(qf), V_MAX, ACCEL, DT)
    ts = np.arange(len(pts)) * DT
    ts[-1] = min(ts[-1], tf)
    axs[0].plot(ts, [np.rad2deg(pp[p]) for pp in pts], "k.", ms=3,
                label=f"samples sent ($q_{p+1}$)")
    for ax in axs:
        for x in (tb, tf - tb):
            ax.axvline(x, color="0.4", ls=":", lw=1)
    axs[0].set_ylabel("position (deg)")
    axs[1].set_ylabel("velocity (deg/s)")
    axs[2].set_ylabel("accel. (deg/s$^2$)")
    axs[2].set_xlabel("time (s)")
    axs[0].legend(ncol=3, fontsize=7, loc="center right")
    axs[1].text(tb / 2, np.rad2deg(V_MAX) * 0.55, "blend", ha="center", fontsize=8)
    axs[1].text(tf / 2, np.rad2deg(V_MAX) * 0.55, "cruise", ha="center", fontsize=8)
    axs[1].text(tf - tb / 2, np.rad2deg(V_MAX) * 0.55, "blend", ha="center", fontsize=8)
    axs[0].set_title(f"Transit segment, $t_b$ = {tb:.2f} s, $t_f$ = {tf:.2f} s")
    fig.savefig(os.path.join(OUT, "lspb_transit.png"))
    plt.close(fig)
    return tb, tf


# --------------------------------------------------------------------------
# 3. Whole mission: TCP height, tilt and yaw of the gripper vs time
# --------------------------------------------------------------------------
def mission_timeline(sols):
    home = [0.0] * 6
    seq = [home] + sols
    t_all, q_all, events, seg_rows = [], [], [], []
    t0 = 0.0
    for k in range(len(seq) - 1):
        pts = traj.generate_segment_trajectory(list(seq[k]), list(seq[k + 1]), V_MAX, ACCEL, DT)
        dur = (len(pts) - 1) * DT
        for j, q in enumerate(pts):
            t_all.append(t0 + j * DT); q_all.append(q)
        seg_rows.append((k, len(pts), dur, np.rad2deg(np.max(np.abs(np.diff(np.array(pts), axis=0)) / DT))))
        t0 += dur
        wp_idx = k - 1            # seq index k+1 == waypoint index k
        if k >= 1 and wp_idx + 1 in GRIPPER_ACTIONS:
            events.append((t0, "close" if GRIPPER_ACTIONS[wp_idx + 1] > 0 else "open"))
            t_all.append(t0 + GRIPPER_TIME); q_all.append(pts[-1])
            t0 += GRIPPER_TIME
    return np.array(t_all), np.array(q_all), events, seg_rows


def fig_orientation(sols):
    t, Q, events, seg_rows = mission_timeline(sols)
    z, tilt, err = [], [], []
    for q in Q:
        p, R = tcp_pose(q)
        z.append(p[2])
        tilt.append(np.rad2deg(np.arccos(np.clip(-R[2, 2], -1, 1))))
        Re = R_DOWN.T @ R   # rotation still needed to reach the commanded orientation
        err.append(np.rad2deg(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))))
    z, tilt, err = map(np.array, (z, tilt, err))
    t_close = [e[0] for e in events if e[1] == "close"][0]
    t_open = [e[0] for e in events if e[1] == "open"][0]
    held = (t >= t_close + GRIPPER_TIME) & (t <= t_open)

    fig, axs = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True)
    axs[0].plot(t, z, color="tab:blue"); axs[0].set_ylabel("TCP height z (m)")
    axs[1].plot(t, err, color="tab:purple", lw=2.2, label="total orientation error")
    axs[1].plot(t, tilt, color="tab:red", lw=1.0, ls="--", label="tilt of approach axis from $-Z$")
    axs[1].set_ylabel("angle (deg)"); axs[1].legend(fontsize=7, loc="upper right")
    axs[1].set_xlabel("mission time (s)")
    for ax in axs:
        ax.axvspan(t_close, t_open + GRIPPER_TIME, color="tab:orange", alpha=0.15, lw=0)
        for te, name in events:
            ax.axvline(te, color="0.3", ls="--", lw=0.8)
    for te, name in events:
        axs[0].text(te + 0.1, 0.78, f"gripper\n{name}", fontsize=7, va="top")
    axs[1].text(t_close + 0.2, 40, "Box B in gripper:\nerror = 0", fontsize=8, color="tab:orange")
    fig.savefig(os.path.join(OUT, "mission_orientation.png"))
    plt.close(fig)
    return float(np.max(err[held])), float(np.max(tilt[held])), seg_rows, t[-1]


def print_waypoint_table(sols):
    print("waypoint IK (deg) and invariants")
    for i, q in enumerate(sols):
        d = np.rad2deg(q)
        p, R = tcp_pose(q)
        print(f"  wp{i+1}: {np.round(d, 2)}  limits_ok={kin.within_limits(q)}  "
              f"q2-q3+q5={d[1]-d[2]+d[4]:.4f}  q1+q6={d[0]+d[5]:.4f}  "
              f"FK tcp={np.round(p, 4)}")


if __name__ == "__main__":
    sols = solve_waypoints()
    print_waypoint_table(sols)
    fig_frames()
    tb, tf = fig_lspb(sols)
    max_err, max_tilt, seg_rows, t_end = fig_orientation(sols)
    print(f"transit tb={tb:.3f} tf={tf:.3f}")
    print(f"while held: max orientation error = {max_err:.2e} deg, max tilt = {max_tilt:.2e} deg")
    print("segments (index, points, duration s, peak joint speed deg/s):")
    for r in seg_rows:
        print(f"  {r[0]}: {r[1]} pts, {r[2]:.2f} s, {r[3]:.1f} deg/s")
    print(f"mission length incl. gripper = {t_end:.2f} s")
    print("figures written to", OUT)
