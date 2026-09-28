"""Cross-check: does our double-S reproduce Ruckig under the same limits?

PROTOCOL.md section 7 states that on a rest-to-rest segment Ruckig with the same
v_max, a_max and j_max should reproduce the double-S profile. That is an
assertion about two independent implementations of the same textbook
construction, so it is checked here rather than cited.

Agreement validates double_s.py. Disagreement means one of them is wrong, and
the paper must not claim the equivalence.
"""
from __future__ import annotations
import math
import numpy as np

import double_s as ds


def ruckig_profile(h, v_max, a_max, j_max, dt=1e-4):
    from ruckig import Ruckig, InputParameter, OutputParameter, Result
    inp = InputParameter(1)
    inp.current_position = [0.0]
    inp.current_velocity = [0.0]
    inp.current_acceleration = [0.0]
    inp.target_position = [h]
    inp.target_velocity = [0.0]
    inp.target_acceleration = [0.0]
    inp.max_velocity = [v_max]
    inp.max_acceleration = [a_max]
    inp.max_jerk = [j_max]

    otg = Ruckig(1, dt)
    out = OutputParameter(1)
    ts, qs, vs, acc = [], [], [], []
    t = 0.0
    res = Result.Working
    guard = 0
    while res == Result.Working and guard < 2_000_000:
        res = otg.update(inp, out)
        # update() advances one step: new_position is the state at t+dt, not t.
        # Labelling it t shifts the whole profile by one sample and shows up as
        # a v_max*dt position error that looks like a real disagreement.
        ts.append(t + dt); qs.append(out.new_position[0])
        vs.append(out.new_velocity[0]); acc.append(out.new_acceleration[0])
        out.pass_to_input(inp)
        t += dt
        guard += 1
    return np.array(ts), np.array(qs), np.array(vs), np.array(acc), out.trajectory.duration


def main():
    cases = [
        ('Exp 1 segment',            0.47319, 0.5, 1.0),
        ('short (v_max not reached)', 0.02,   0.5, 1.0),
        ('very short (neither)',      0.0005, 0.5, 1.0),
    ]
    j_max = math.radians(445.0)
    print(f'j_max = {j_max:.4f} rad/s^3\n')
    print(f"{'case':28} {'ours T':>9} {'ruckig T':>9} {'dT':>10} {'max |dq|':>11}")
    worst = 0.0
    for label, h, v_max, a_max in cases:
        Tj1, Ta, Tv, Tj2, Td, _ = ds.phase_durations(h, v_max, a_max, j_max)
        T_ours = Ta + Tv + Td
        ts, q_r, v_r, a_r, T_ruckig = ruckig_profile(h, v_max, a_max, j_max)
        # compare positions on Ruckig's own sample times
        q_o, v_o, a_o, j_o, _ = ds.evaluate(ts, h, v_max, a_max, j_max)
        dq = np.max(np.abs(q_o - q_r))
        worst = max(worst, dq)
        print(f'{label:28} {T_ours:9.6f} {T_ruckig:9.6f} '
              f'{abs(T_ours - T_ruckig):10.2e} {dq:11.2e}')
    print(f'\nworst position deviation across cases: {worst:.3e} rad '
          f'({math.degrees(worst):.2e} deg)')
    print('VERDICT:', 'equivalent' if worst < 1e-5 else 'NOT equivalent -- do not claim it')


if __name__ == '__main__':
    main()
