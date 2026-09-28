"""Phase 2 acceptance checks: double-S for six joints (CHAT_BRIEFING.md sec. 13).

Run from AROB_10methods/ with double_s.py, shape_trajectories.py, lspb.json and
data/verification/via_point.json present. Exits non-zero on any failure.
"""
import json, os, sys
import numpy as np
import double_s as ds
import shape_trajectories as st
from double_s_trajectory import METHOD as DS

V, A, J = 0.5, 1.0, st.J_MAX
fails = []
def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok: fails.append(msg)

lspb = json.load(open('lspb.json')); q0, qf = np.array(lspb['q0']), np.array(lspb['qf'])
dqv = qf - q0; pace = int(np.argmax(np.abs(dqv))); h = abs(dqv[pace])

print('\n[1] Experiment 1 duration and phase structure vs briefing')
d = DS.describe(h, V, A, J)
print(f"      T={d['T']:.4f}  regime {d['regime']}  Tj={d['Tj1']:.4f}  Ta={d['Ta']:.4f}  Tv={d['Tv']:.4f}  a_lim={d['a_lim']:.4f}")
check(abs(d['T'] - 1.5751) < 6e-5, f"T = {d['T']:.4f} (briefing 1.5751)")
check(d['regime'] == 1 and abs(d['Tj1'] - 0.1288) < 6e-5 and abs(d['Ta'] - 0.6288) < 6e-5
      and abs(d['Tv'] - 0.3176) < 6e-5, 'regime 1 with Tj 0.1288, Ta 0.6288, Tv 0.3176')

print('\n[2] six-joint profile on the real segment')
T = DS.segment_duration(h, V, A, J)
ts, pos, vel, acc, jerk = DS.full_profile(q0, qf, T, 0.01, V, A, J, with_jerk=True)
e_pos = max(np.max(np.abs(pos[0]-q0)), np.max(np.abs(pos[-1]-qf)))
e_va = max(np.max(np.abs(vel[[0, -1]])), np.max(np.abs(acc[[0, -1]])))
check(e_pos < 1e-14 and e_va < 1e-12, f"endpoints: pos err {e_pos:.1e}, vel/acc at both ends {e_va:.1e}")
check(bool(np.all(np.diff(ts) > 0)) and ts[0] == 0 and abs(ts[-1] - T) < 1e-12, 'time grid strictly increasing, ends at T')
s = np.array([(pos[:, j] - q0[j]) / dqv[j] for j in range(6) if abs(dqv[j]) > 1e-9])
check(np.max(np.abs(s - s[0])) < 1e-14, f"same s(tau) on every moving joint (straight joint-space path), max dev {np.max(np.abs(s - s[0])):.1e}")
ratio_ok = all(abs(np.max(np.abs(vel[:, j])) / np.max(np.abs(vel[:, pace])) - abs(dqv[j]) / h) < 1e-12 for j in range(6))
check(ratio_ok, 'non-pacing joints have peaks scaled by |dq_j|/|dq_pace|')
check(np.max(np.abs(vel[:, pace])) <= V * (1 + 1e-9) and np.max(np.abs(acc[:, pace])) <= A * (1 + 1e-9)
      and np.max(np.abs(jerk[:, pace])) <= J * (1 + 1e-9), 'pacing joint respects v_max, a_max and j_max')

print('\n[3] all three regimes through the six-joint wrapper (regime 2 needs 2a^3/j^2 < h < ~0.31 rad)')
cases = (('regime 1 (Exp 1 segment)', h, 1), ('regime 2 (v_max not reached)', 0.1, 2),
         ('regime 3 (no limit reached)', 0.02, 3), ('regime 3, very short', 0.0005, 3))
for label, hh, want in cases:
    dd = DS.describe(hh, V, A, J)
    qa = np.zeros(6); qb = np.array([hh, -0.5*hh, 0.0, 0.25*hh, 0.1*hh, -hh])   # mixed signs
    T3 = DS.segment_duration(hh, V, A, J)
    t3, p3, v3, a3, j3 = DS.full_profile(qa, qb, T3, 0.001, V, A, J, with_jerk=True)
    ok = (np.max(np.abs(p3[-1]-qb)) < 1e-13 and np.max(np.abs(v3[[0, -1]])) < 1e-12
          and np.max(np.abs(a3[[0, -1]])) < 1e-12 and np.max(np.abs(j3)) <= J*(1+1e-9)
          and np.max(np.abs(v3)) <= V*(1+1e-9) and np.max(np.abs(a3)) <= A*(1+1e-9))
    check(ok and dd['regime'] == want,
          f"{label:30s} h={hh:<7g} T={T3:.4f}  regime {dd['regime']} (want {want})  endpoints/limits hold, mixed-sign joints")

print('\n[4] derivatives are mutually consistent (integrate a -> v -> q on a 1 us grid)')
tt = np.arange(0, T + 1e-6, 1e-6)
_, p, v, a, j = DS.full_profile([0.0], [h], T, 1e-6, V, A, J, with_jerk=True)
tt = np.arange(len(v)) * 1e-6
v_int = np.concatenate([[0], np.cumsum((a[1:, 0] + a[:-1, 0]) / 2 * 1e-6)])
p_int = np.concatenate([[0], np.cumsum((v[1:, 0] + v[:-1, 0]) / 2 * 1e-6)])
check(np.max(np.abs(v_int - v[:, 0])) < 1e-9 and np.max(np.abs(p_int - (p[:, 0] - p[0, 0]))) < 1e-9,
      f"int(a)=v to {np.max(np.abs(v_int - v[:, 0])):.1e}, int(v)=q to {np.max(np.abs(p_int - (p[:, 0] - p[0, 0]))):.1e}")

print('\n[5] analytic peaks: exact at the peak instants, never exceeded by any sample; ISJ vs dense sum')
for label, hh in (('Exp 1 segment', h), ('regime 2', 0.1), ('regime 3', 0.02), ('very short', 0.0005)):
    dd = DS.describe(hh, V, A, J); T5 = dd['T']
    pv, pa, pj = DS.commanded_peaks(hh, V, A, J)
    # exact instants: a peaks at t=Tj1, v peaks at t=Ta (a = 0 there), jerk is +-j_max inside phase 1
    _, v_at, a_at, j_at, _T = (lambda r: (r[0], r[1], r[2], r[3], r[4]))(ds.evaluate(np.array([dd['Tj1'], dd['Ta'], 0.5*dd['Tj1']]), hh, V, A, J))
    e_exact = max(abs(v_at[1] - pv) / pv, abs(a_at[0] - pa) / pa, abs(abs(j_at[2]) - pj) / pj)
    _, p5, v5, a5, j5 = DS.full_profile([0.0], [hh], T5, 2e-6, V, A, J, with_jerk=True)
    not_exceeded = (np.max(np.abs(v5)) <= pv * (1 + 1e-12) and np.max(np.abs(a5)) <= pa * (1 + 1e-12)
                    and np.max(np.abs(j5)) <= pj * (1 + 1e-12))
    num_isj = float(np.sum(j5[:-1, 0] ** 2) * 2e-6)
    e_isj = abs(num_isj - DS.isj(hh, V, A, J)) / DS.isj(hh, V, A, J)
    check(e_exact < 1e-12 and not_exceeded and e_isj < 1e-4,
          f"{label:14s} peak err at exact instants {e_exact:.1e}, none exceeded: {not_exceeded};  ISJ exact {DS.isj(hh, V, A, J):.4f}, dense sum {num_isj:.4f} (rel {e_isj:.1e}, boundary error is O(dt/Tj))")

print('\n[6] single-joint agreement with double_s.py itself')
tq = np.linspace(0, T, 1001)
q1, v1, a1, j1, T1 = ds.evaluate(tq, h, V, A, J)
_, p6, v6, a6 = DS.full_profile([0.0], [h], T, T / 1000, V, A, J)
n = min(len(tq), len(p6))
check(abs(T1 - T) < 1e-12 and np.max(np.abs(p6[:n, 0] - q1[:n])) < 1e-12, f"pos vs double_s.evaluate: {np.max(np.abs(p6[:n, 0] - q1[:n])):.1e}")

print('\n[7] Experiment 2 stitched double-S durations (NOT in the briefing table; recorded here)')
for path in ('data/verification/via_point.json', 'via_point.json'):
    if os.path.exists(path): vp = json.load(open(path)); break
W = np.array([vp['q0'], vp['qv1'], vp['qv2'], vp['qf']])
dqs = [float(np.max(np.abs(x))) for x in np.diff(W, axis=0)]
rows = [DS.describe(x, V, A, J) for x in dqs]
print('      segments: ' + '; '.join(f"{r['T']:.4f} s (regime {r['regime']})" for r in rows) + f"; total {sum(r['T'] for r in rows):.4f} s")
def lspb_T(dq, v, a):                       # the LSPB duration rule of briefing sec. 4.3, written out here
    dq = abs(dq)
    return v / a + dq / v if dq >= v * v / a else 2 * np.sqrt(dq / a)
diffs = [r['T'] - lspb_T(x, V, A) for r, x in zip(rows, dqs)]
check(all(r['regime'] == 1 and r['a_lim'] >= A * (1 - 1e-9) for r in rows) and all(abs(x - A / J) < 1e-12 for x in diffs),
      f"independent relation: T_double-S = T_LSPB + a_max/j_max = +{A/J:.5f} s on every segment (differences {[round(x, 5) for x in diffs]})")
d1 = DS.describe(h, V, A, J)['T'] - lspb_T(h, V, A)
check(abs(d1 - A / J) < 1e-12, f"same relation on the Experiment 1 segment: 1.4464 + {A/J:.4f} = {DS.describe(h, V, A, J)['T']:.4f}")

print('\n' + ('ALL PHASE 2 CHECKS PASS' if not fails else f'{len(fails)} FAILURE(S)'))
sys.exit(1 if fails else 0)
