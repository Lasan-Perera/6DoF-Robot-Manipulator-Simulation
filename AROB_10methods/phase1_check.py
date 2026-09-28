"""Phase 1 acceptance checks (CHAT_BRIEFING.md sec. 13, phase 1).

Run from AROB_10methods/ with shape_functions.py, lspb.json and
data/verification/via_point.json present. Exits non-zero on any failure.
"""
import json, os, sys
import numpy as np
import shape_trajectories as st
import shape_functions as sf

V_MAX, A_MAX, J_MAX, J_MAX_DEG = 0.5, 1.0, st.J_MAX, st.J_MAX_DEG
fails = []
def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok: fails.append(msg)

# ---- 1. analytic shapes vs the numerical shapes in shape_functions.py -------
print('\n[1] analytic shapes vs shape_functions.py (2,000,001-point grid)')
num = sf.all_shapes()
name_map = {'cubic': 'cubic', 'quintic': 'quintic', 'septic': 'septic',
            'cycloidal': 'cycloidal', 'modsine': 'modified sine', 'modtrap': 'modified trapezoid'}
for k, nk in name_map.items():
    mine = st.METHODS[k].shape(sf.TAU)
    err = [float(np.max(np.abs(a - b))) for a, b in zip(mine, num[nk])]
    # the numeric cam derivatives are cumulative-trapezoid integrals; the jerk of
    # a cam is discontinuous at breakpoints, so allow a few grid cells there
    tol = 1e-8
    check(max(err[:3]) < tol, f"{k:9s} max|s,s',s''| diff = {max(err[:3]):.2e}   (s''' diff {err[3]:.2e})")

# ---- 2. peak constants ------------------------------------------------------
print('\n[2] peak constants |s\'|,|s\'\'|,|s\'\'\'| max')
closed = {'cubic': (1.5, 6.0, 12.0), 'quintic': (15/8, 10/3**.5, 60.0),
          'septic': (35/16, None, 52.5), 'cycloidal': (2.0, 2*np.pi, 4*np.pi**2)}
for k, c in closed.items():
    e = max(abs(p - c[i]) for i, p in enumerate(st.METHODS[k].peaks) if c[i] is not None)
    check(e < 1e-12, f"{k:9s} vs closed form, max err {e:.1e}")
brief = {'septic': (2.1875, 7.5132, 52.5), 'modsine': (1.7596, 5.5280, 69.4664),
         'modtrap': (2.0000, 4.8881, 61.4260)}
for k, b in brief.items():
    e = max(abs(p - x) for p, x in zip(st.METHODS[k].peaks, b))
    check(e < 6e-5, f"{k:9s} vs briefing table (4 dp), max err {e:.1e}")
for k in ('modsine', 'modtrap'):     # exact relations of the two cam constructions
    m = st.METHODS[k]; kk = m.k
    e = max(abs(m.peaks[1] - kk), abs(m.peaks[2] - 4*np.pi*kk))
    check(e < 1e-12, f"{k:9s} |s''|max = k and |s'''|max = 4*pi*k, max err {e:.1e}")

# ---- 3. endpoints and zero derivatives on the real Experiment 1 segment -----
print('\n[3] Experiment 1 segment, six joints, endpoints')
lspb = json.load(open('lspb.json')); q0, qf = np.array(lspb['q0']), np.array(lspb['qf'])
dqv = qf - q0; pace = int(np.argmax(np.abs(dqv))); dq = abs(dqv[pace])
print(f'      pacing joint {pace+1}, |dq| = {dq:.10f} rad = {np.rad2deg(dq):.4f} deg')
for k, m in st.METHODS.items():
    T = m.segment_duration(dq, V_MAX, A_MAX, J_MAX)
    ts, pos, vel, acc = m.full_profile(q0, qf, T, 0.01)
    e_pos = max(np.max(np.abs(pos[0]-q0)), np.max(np.abs(pos[-1]-qf)))
    e_vel = max(np.max(np.abs(vel[0])), np.max(np.abs(vel[-1])))
    if k == 'cubic':      # acceleration is NOT zero at the ends (defining property)
        e_acc = max(np.max(np.abs(acc[0] - 6*dqv/T**2)), np.max(np.abs(acc[-1] + 6*dqv/T**2)))
        note = 'end acc = +-6 dq/T^2'
    else:
        e_acc = max(np.max(np.abs(acc[0])), np.max(np.abs(acc[-1]))); note = 'end acc = 0'
    mono = bool(np.all(np.diff(ts) > 0)) and abs(ts[-1] - T) < 1e-12 and ts[0] == 0
    check(e_pos < 1e-14 and e_vel < 1e-14 and e_acc < 1e-13 and mono,
          f"{k:9s} pos err {e_pos:.1e}  vel err {e_vel:.1e}  acc err {e_acc:.1e} ({note})  ts increasing to T: {mono}")

# ---- 4. peaks of the generated profile equal p*|dq|/T^n --------------------
print('\n[4] peaks of the generated profile (dense, 2 us) vs p*|dq|/T^n')
for k, m in st.METHODS.items():
    T = m.segment_duration(dq, V_MAX, A_MAX, J_MAX)
    ts, pos, vel, acc = m.full_profile([0.0], [dq], T, 2e-6)
    pv, pa = float(np.max(np.abs(vel))), float(np.max(np.abs(acc)))
    ev, ea, _ = m.commanded_peaks(dq, T)
    check(abs(pv-ev)/ev < 1e-9 and abs(pa-ea)/ea < 1e-9,
          f"{k:9s} rel err vel {abs(pv-ev)/ev:.1e}, acc {abs(pa-ea)/ea:.1e}")
    # an exact jerk-free bound check: limits must hold at the chosen T
    lim = (ev <= V_MAX*(1+1e-9)) and (ea <= A_MAX*(1+1e-9))
    jm = m.commanded_peaks(dq, T)[2]
    lim_j = (not m.jerk_bounded) or jm <= J_MAX*(1+1e-9)
    jtxt = (f"{np.rad2deg(jm):.1f}/{J_MAX_DEG:.0f}" if m.jerk_bounded else 'n/a (impulsive at the ends)')
    check(lim and lim_j, f"{k:9s} limits hold: v {ev:.4f}/{V_MAX}  a {ea:.4f}/{A_MAX}  jerk {jtxt}")

# ---- 5. integrated squared jerk factor vs dense numerical integral ---------
print('\n[5] integral of s\'\'\'^2 dtau (exact) vs dense trapezoid of the analytic jerk')
tau = np.linspace(0, 1, 4_000_001)
for k, m in st.METHODS.items():
    num_i = float(np.trapezoid(m.shape(tau)[3] ** 2, tau))
    check(abs(num_i - m.isj_factor) / m.isj_factor < 1e-6,
          f"{k:9s} exact {m.isj_factor:.6f}  numeric {num_i:.6f}")

# ---- 6. durations against CHAT_BRIEFING.md sec. 6 ---------------------------
print('\n[6] Experiment 1 durations vs briefing table (T, binding limit)')
exp1 = {'cubic': (1.6850, 'acc'), 'quintic': (1.7745, 'vel'), 'septic': (2.0702, 'vel'),
        'cycloidal': (1.8928, 'vel'), 'modsine': (1.6653, 'vel'), 'modtrap': (1.8928, 'vel')}
for k, (Tb, bind) in exp1.items():
    T, Tv, Ta, Tj, b = st.METHODS[k].duration_terms(dq, V_MAX, A_MAX, J_MAX)
    check(abs(T - Tb) < 6e-5 and b == bind, f"{k:9s} T = {T:.4f} (brief {Tb:.4f}), binds {b} (brief {bind});  terms vel {Tv:.4f} acc {Ta:.4f} jerk {Tj:.4f}")

print('\n[7] Experiment 2 stitched durations vs briefing table (per segment, total)')
vp = None
for path in ('data/verification/via_point.json', 'via_point.json'):
    if os.path.exists(path): vp = json.load(open(path)); break
W = np.array([vp['q0'], vp['qv1'], vp['qv2'], vp['qf']])
dqs = [float(np.max(np.abs(d))) for d in np.diff(W, axis=0)]
print('      pacing |dq| (deg): ' + ', '.join(f'{np.rad2deg(x):.2f}' for x in dqs))
exp2 = {'cubic': ([1.6307, 1.9905, 1.8193], 5.4405, 'acc acc acc'),
        'quintic': ([1.6619, 2.4764, 2.0687], 6.2070, 'vel vel vel'),
        'septic': ([1.9389, 2.8891, 2.4135], 7.2415, 'vel vel vel'),
        'cycloidal': ([1.7727, 2.6415, 2.2066], 6.6208, 'vel vel vel'),
        'modsine': ([1.5826, 2.3240, 1.9414], 5.8480, 'jerk vel vel'),
        'modtrap': ([1.7727, 2.6415, 2.2066], 6.6208, 'vel vel vel')}
for k, (Tb, tot_b, bind_b) in exp2.items():
    r = [st.METHODS[k].duration_terms(d, V_MAX, A_MAX, J_MAX) for d in dqs]
    Ts, binds = [x[0] for x in r], ' '.join(x[4] for x in r)
    ok = all(abs(a-b) < 6e-5 for a, b in zip(Ts, Tb)) and abs(sum(Ts)-tot_b) < 1.5e-4 and binds == bind_b
    check(ok, f"{k:9s} segs {[round(float(x),4) for x in Ts]} total {sum(Ts):.4f} (brief {tot_b:.4f})  binds: {binds}")

print('\n[8] where the jerk term binds at j_max = 445 deg/s^3 (expected: only modsine, Exp 2 seg 1)')
hits = []
for label, segs in (('Exp1', [dq]), ('Exp2', dqs)):
    for k, m in st.METHODS.items():
        for i, d in enumerate(segs):
            T, Tv, Ta, Tj, b = m.duration_terms(d, V_MAX, A_MAX, J_MAX)
            if b == 'jerk': hits.append((label, k, i+1, T, max(Tv, Ta)))
print('      binding cases:', [(l, k, f'seg{i}', f'T={T:.4f}', f'T without jerk term={T0:.4f}') for l, k, i, T, T0 in hits])
check(len(hits) == 1 and hits[0][:3] == ('Exp2', 'modsine', 1), 'jerk term binds exactly once: modsine, Exp 2 segment 1')
m = st.METHODS['modsine']; Tt, Tv, Ta, Tj, _b = m.duration_terms(dqs[0], V_MAX, A_MAX, J_MAX)
T0 = max(Tv, Ta)
need = np.rad2deg(m.commanded_peaks(dqs[0], T0)[2])
print(f'      modsine seg 1 terms: velocity {Tv:.4f} s, acceleration {Ta:.4f} s, jerk {Tj:.4f} s')
check(abs(need - 460) < 1.0 and Ta > Tv,
      f"without the jerk term the segment is acceleration-limited at {T0:.4f} s and needs {need:.1f} deg/s^3 (briefing: 460); the jerk limit lengthens it to {Tt:.4f} s")

print('\n' + ('ALL PHASE 1 CHECKS PASS' if not fails else f'{len(fails)} FAILURE(S)'))
sys.exit(1 if fails else 0)
