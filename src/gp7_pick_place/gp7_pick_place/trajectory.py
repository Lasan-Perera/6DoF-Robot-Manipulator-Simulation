import numpy as np


def segment_timing(delta_q: float, v_max: float, accel: float) -> tuple[float, float]:
    # Check if the distance is enough to reach max velocity
    if abs((delta_q)) >= v_max**2 / accel:
        tb = v_max / accel
        tf = tb + abs(delta_q) / v_max
    else:
        tb = (abs(delta_q) / accel)** 0.5
        tf = 2 * tb
    return tb, tf


def lspb_position(t: float, q0: float, qf: float, tb: float, tf: float) -> float:
    V_i = (qf - q0) / (tf - tb)
    alpha_i = V_i / tb
    # compute position based on the time t
    if t < tb:
        q = q0 + (alpha_i / 2) * t**2
    elif t <= tf - tb:
        a0 = (q0 + qf - V_i * tf) / 2
        q = a0 + V_i * t
    else:
        t_prime = tf - t
        q = qf - (alpha_i / 2) * t_prime**2

    return q


def generate_segment_trajectory(q0: list[float], qf: list[float],
                                 v_max: float, accel: float,
                                 dt: float = 0.05) -> list[list[float]]:
    # trajectory generation using LSPB (Linear Segment with Parabolic Blends)
    
    delta_qs = [qf[i] - q0[i] for i in range(6)]
    pacing_idx = max(range(6), key=lambda i: abs(delta_qs[i]))
    tb, tf = segment_timing(delta_qs[pacing_idx], v_max, accel)

    trajectory = []
    t = 0.0
    while t <= tf:
        point = []
        for i in range(6):
            if q0[i] == qf[i]:
                point.append(q0[i])
            else:
                point.append(lspb_position(t, q0[i], qf[i], tb, tf))
        trajectory.append(point)
        t += dt

    if trajectory[-1] != list(qf):
        trajectory.append(list(qf))

    return trajectory