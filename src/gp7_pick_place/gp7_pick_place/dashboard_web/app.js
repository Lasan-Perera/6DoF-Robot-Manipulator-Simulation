'use strict';
/* GP7 live dashboard: front end. Data arrives from dashboard.py over SSE (/stream). */

const WINDOW_S = 30;            // chart window (sim seconds)
const TRAIL_MAX = 4000;         // TCP trail points kept for the views
const HISTORY_MAX = 400000;     // samples kept for CSV export
const $ = (id) => document.getElementById(id);

const S = {
  cfg: null, last: null, history: [], trail: [], paused: false,
  hidden: { angles: new Set(), vels: new Set() },
  hover: { angles: null, vels: null, grip: null },
  cam: { az: -38, el: 24, zoom: 1 }, dirty: true, lastT: null, C: {},
};

// ------------------------------------------------------------------ helpers
const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? '–' : Number(v).toFixed(d);
const fmtSigned = (v, d = 2) => (v === null || v === undefined) ? '–' : (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(d);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const badge = (cls, icon, text) => `<span class="badge ${cls}">${icon} ${esc(text)}</span>`;
const ICON = { good: '✓', warning: '!', serious: '!', critical: '✕', info: '▶', neutral: '·' };
const wrap180 = (a) => ((a + 180) % 360 + 360) % 360 - 180;

function readColors() {
  const cs = getComputedStyle(document.documentElement);
  const g = (n) => cs.getPropertyValue(n).trim();
  S.C = {
    series: [1, 2, 3, 4, 5, 6].map((i) => g(`--s${i}`)),
    s7: g('--s7'), s8: g('--s8'),
    ink: g('--ink'), ink2: g('--ink-2'), muted: g('--muted'), grid: g('--grid'), axis: g('--axis'),
    surface: g('--surface'), surface2: g('--surface-2'), good: g('--good'), warning: g('--warning'),
    critical: g('--critical'), accent: g('--accent'), boxA: g('--box-a'), boxB: g('--box-b'),
    arm: g('--arm'), trail: g('--trail'), plan: g('--plan'),
  };
  S.dirty = true;
}

function setupCanvas(cv) {
  const dpr = window.devicePixelRatio || 1;
  const r = cv.getBoundingClientRect();
  const w = Math.max(10, Math.round(r.width)), h = Math.max(10, Math.round(r.height));
  if (cv.width !== w * dpr || cv.height !== h * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}

function niceTicks(min, max, count = 5) {
  if (!(max > min)) { max = min + 1; }
  const span = max - min, step0 = span / count;
  const mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => span / s <= count) || 10 * mag;
  const ticks = [];
  for (let v = Math.ceil(min / step) * step; v <= max + 1e-9; v += step) ticks.push(Math.abs(v) < 1e-12 ? 0 : v);
  return { ticks, step };
}

// --------------------------------------------------------------- static UI
function buildStatic(cfg) {
  // joint table
  const tb = $('joint-table').querySelector('tbody');
  tb.innerHTML = cfg.arm_joints.map((j, i) => `
    <tr>
      <td><div class="jname"><span class="swatch" style="background:var(--s${i + 1})"></span>
        <div><div>${cfg.joint_labels[i]}</div><div class="jsub mono">${j}</div></div></div></td>
      <td class="num" id="q${i}">–</td>
      <td class="bar-col"><div class="rbar" title="limits ${cfg.limits_deg[i][0]}° … ${cfg.limits_deg[i][1]}°">
        <div class="track"></div><div class="zero" id="qz${i}"></div><div class="fill" id="qf${i}" style="background:var(--s${i + 1})"></div>
        <div class="refmark" id="qr${i}"></div><div class="mark" id="qm${i}" style="background:var(--s${i + 1})"></div></div>
        <div class="jsub mono" style="display:flex;justify-content:space-between"><span>${cfg.limits_deg[i][0]}</span><span>${cfg.limits_deg[i][1]}</span></div></td>
      <td class="num" id="v${i}">–</td>
      <td class="bar-col"><div class="rbar" title="±V_max = ±${fmt(cfg.v_max_deg, 1)} °/s">
        <div class="track"></div><div class="zero" style="left:50%"></div>
        <div class="vmax" style="left:${50 - 50 / 1.25}%"></div><div class="vmax" style="left:${50 + 50 / 1.25}%"></div>
        <div class="fill" id="vf${i}" style="background:var(--s${i + 1})"></div></div></td>
      <td class="num" id="r${i}">–</td>
      <td class="num" id="e${i}">–</td>
      <td class="num" id="f${i}">–</td>
    </tr>`).join('');
  cfg.limits_deg.forEach(([lo, hi], i) => { $(`qz${i}`).style.left = `${(0 - lo) / (hi - lo) * 100}%`; });

  // stepper
  const steps = [{ index: 0, label: 'Home', xyz: null }, ...cfg.waypoints];
  $('stepper').innerHTML = steps.map((w) => `
    <li class="step" id="step${w.index}"><div class="step-n">${w.index === 0 ? 'Start' : 'WP' + w.index}</div>
      <div class="step-t" title="${esc(w.label)}">${esc(w.label)}</div><div class="step-s" id="steps${w.index}">${w.index === 0 ? 'all joints 0°' : 'pending'}</div></li>`).join('');

  // waypoint table
  const grip = (v) => v === null || v === undefined ? '–' : (v > 0.3 ? `close → ${v} rad` : `open → ${v} rad`);
  $('wp-table').querySelector('tbody').innerHTML =
    `<tr id="wprow0"><td>0</td><td>Home (start)</td><td class="num">–</td><td class="num">–</td><td class="num">–</td>
      ${cfg.home_deg.map((v) => `<td class="num">${fmt(v, 1)}</td>`).join('')}<td>open</td><td class="num">–</td><td class="num">–</td><td id="wpst0">–</td></tr>` +
    cfg.waypoints.map((w) => `<tr id="wprow${w.index}">
      <td>${w.index}</td><td>${esc(w.label)}</td>
      ${w.xyz.map((v) => `<td class="num">${fmt(v, 3)}</td>`).join('')}
      ${w.q_deg.map((v) => `<td class="num">${fmt(v, 1)}</td>`).join('')}
      <td>${grip(w.gripper_after)}</td><td class="num" id="wpd${w.index}">–</td><td class="num" id="wpt${w.index}">–</td>
      <td id="wpst${w.index}">${w.within_limits ? badge('neutral', '·', 'pending') : badge('critical', '✕', 'out of limits')}</td></tr>`).join('');

  // segments
  $('seg-table').querySelector('tbody').innerHTML = cfg.segments.map((s) => `
    <tr id="seg${s.to_index}"><td>${esc(s.from)} → ${esc(s.to)}</td><td class="num">${fmt(s.duration_s, 2)}</td>
    <td class="num" id="segm${s.to_index}">–</td><td class="num">J${s.pacing_joint}</td><td class="num">${fmt(s.max_dq_deg, 1)}</td>
    <td>${s.gripper_after === null || s.gripper_after === undefined ? '–' : (s.gripper_after > 0.3 ? 'close gripper' : 'open gripper')}</td></tr>`).join('') +
    `<tr><td><b>Total arm motion</b></td><td class="num"><b>${fmt(cfg.total_planned_s, 2)}</b></td><td class="num" id="segm-total">–</td><td></td><td></td><td></td></tr>`;
  $('planned-total').textContent = `${fmt(cfg.total_planned_s, 2)} s`;

  // parameters
  const R = cfg.r_target.map((r) => `[${r.map((v) => fmtSigned(v, 0)).join(' ')}]`).join(' ');
  $('param-kv').innerHTML = [
    ['V<sub>max</sub>', `${cfg.v_max} rad/s (${fmt(cfg.v_max_deg, 1)} °/s)`],
    ['Acceleration', `${cfg.accel} rad/s²`],
    ['Sample time dt', `${cfg.dt} s`],
    ['Profile', 'synchronised LSPB'],
    ['L<sub>TCP</sub> (flange→grasp)', `${cfg.l_tcp} m`],
    ['Max wrist-centre reach', `${fmt(cfg.reach_radius, 3)} m`],
    ['Target orientation R', R],
    ['Gripper close cmd', `${cfg.gripper_actions['1'] ?? '–'} rad (full ≈ ${cfg.gripper_closed})`],
    ['World file', cfg.world_sdf],
  ].map(([k, v]) => `<dt>${k}</dt><dd>${esc(v).replace(/&lt;(\/?)sub&gt;/g, '<$1sub>')}</dd>`).join('');
  $('dh-table').querySelector('tbody').innerHTML = cfg.dh.map((r, i) =>
    `<tr><td>${i + 1}</td><td class="num">${fmt(r.a, 3)}</td><td class="num">${r.alpha}</td><td class="num">${fmt(r.d, 3)}</td><td class="mono">${esc(cfg.dh_theta[i])}</td></tr>`).join('');

  // legends
  document.querySelectorAll('.series-legend').forEach((el) => {
    const chart = el.dataset.chart;
    el.innerHTML = cfg.joint_labels.map((l, i) =>
      `<button class="legend-item" data-i="${i}"><span class="legend-line" style="background:var(--s${i + 1})"></span>${esc(l)}</button>`).join('');
    el.querySelectorAll('.legend-item').forEach((b) => b.addEventListener('click', () => {
      const i = +b.dataset.i, set = S.hidden[chart];
      set.has(i) ? set.delete(i) : set.add(i);
      b.classList.toggle('off', set.has(i));
      S.dirty = true;
    }));
  });
  $('legend-3d').innerHTML = [
    ['--arm', 'arm (D-H frame origins)'], ['--trail', 'TCP trail'], ['--plan', 'planned TCP path', true],
    ['--box-a', 'Box A'], ['--box-b', 'Box B'], ['--accent', 'waypoints'],
  ].map(([c, l, dashed]) => `<span class="legend-static"><span class="legend-line" style="background:var(${c});${dashed ? 'opacity:.6' : ''}"></span>${l}</span>`).join('');
}

// ------------------------------------------------------------- live update
function onState(st) {
  S.last = st;
  const pill = $('pill-link');
  if (st.type === 'waiting') {
    pill.className = 'pill stale'; $('link-text').textContent = 'Dashboard up · no /joint_states yet';
    $('phase').textContent = 'Waiting for the simulation… (start Gazebo)';
    return;
  }
  const age = st.link.joint_states_age_s;
  if (age > 1.0) { pill.className = 'pill stale'; $('link-text').textContent = `Stale · last data ${fmt(age, 1)} s ago`; }
  else { pill.className = 'pill live'; $('link-text').textContent = 'Live'; }
  $('sim-time').textContent = `${fmt(st.t, 2)} s`;
  $('js-rate').textContent = `${st.link.joint_states_hz} Hz`;
  $('mission-elapsed').textContent = st.mission.elapsed_s !== null ? `${fmt(st.mission.elapsed_s, 1)} s` : '–';

  // history (one sample per new sim time)
  if (S.lastT !== null && st.t < S.lastT - 0.5) { S.history = []; S.trail = []; }   // sim restarted
  if (st.t !== S.lastT) {
    S.lastT = st.t;
    const g = st.gripper.pos;
    S.history.push({ t: st.t, q: st.arm.q_deg, qd: st.arm.qd_deg, tcp: st.ee.tcp, oe: st.ee.orient_err_deg, g, ph: st.mission.phase });
    if (S.history.length > HISTORY_MAX) S.history.splice(0, S.history.length - HISTORY_MAX);
    const lt = S.trail[S.trail.length - 1];
    if (!lt || Math.hypot(lt[0] - st.ee.tcp[0], lt[1] - st.ee.tcp[1], lt[2] - st.ee.tcp[2]) > 0.002) {
      S.trail.push(st.ee.tcp);
      if (S.trail.length > TRAIL_MAX) S.trail.shift();
    }
  }
  if (!S.paused) S.dirty = true;

  updateJoints(st); updateEE(st); updateMission(st); updateGripper(st); updateBoxes(st); updateSystem(st);
}

function updateJoints(st) {
  const cfg = S.cfg, a = st.arm, vmax = cfg.v_max_deg;
  for (let i = 0; i < 6; i++) {
    const [lo, hi] = cfg.limits_deg[i], q = a.q_deg[i];
    const p = clamp((q - lo) / (hi - lo), 0, 1) * 100, z = (0 - lo) / (hi - lo) * 100;
    const qe = $(`q${i}`);
    qe.textContent = fmt(q, 2); qe.className = 'num' + (a.in_limits[i] ? '' : ' warn-cell');
    $(`qm${i}`).style.left = `${p}%`;
    const f = $(`qf${i}`); f.style.left = `${Math.min(p, z)}%`; f.style.width = `${Math.abs(p - z)}%`; f.style.opacity = 0.35;
    const rm = $(`qr${i}`);
    if (a.ref_deg) { rm.style.display = 'block'; rm.style.left = `${clamp((a.ref_deg[i] - lo) / (hi - lo), 0, 1) * 100}%`; }
    else rm.style.display = 'none';
    const v = a.qd_deg[i];
    $(`v${i}`).textContent = fmtSigned(v, 2);
    const vp = clamp(v / (1.25 * vmax), -1, 1) * 50;
    const vf = $(`vf${i}`); vf.style.left = `${50 + Math.min(0, vp)}%`; vf.style.width = `${Math.abs(vp)}%`;
    $(`r${i}`).textContent = a.ref_deg ? fmt(a.ref_deg[i], 2) : '–';
    $(`e${i}`).textContent = a.err_deg ? fmtSigned(a.err_deg[i], 3) : '–';
    $(`f${i}`).textContent = a.effort && a.effort[i] !== null ? fmt(a.effort[i], 1) : '–';
  }
}

function sevForOrient(e) { return e < 0.5 ? 'good' : e < 2 ? 'warning' : 'critical'; }

function updateEE(st) {
  const e = st.ee, a = st.arm;
  ['x', 'y', 'z'].forEach((k, i) => { $(`tcp-${k}`).textContent = fmt(e.tcp[i], 4); });
  $('tcp-source').textContent = e.source === 'tf' ? 'live TF base_link → tcp' : 'TF unavailable · showing our FK';
  const oe = e.orient_err_deg, sev = sevForOrient(oe);
  const psiDev = wrap180(a.psi_deg - 90), sumDev = wrap180(a.q1_plus_q6_deg + 180);
  const vj = Math.hypot(...e.v_jac_mm_s);
  const rows = [
    ['FK prediction (our model)', `${fmt(e.tcp_fk[0], 4)}, ${fmt(e.tcp_fk[1], 4)}, ${fmt(e.tcp_fk[2], 4)}`],
    ['FK vs live TF', e.fk_vs_tf_mm === null ? '–' : `${fmt(e.fk_vs_tf_mm, 3)} mm`],
    ['Flange tool0', `${fmt(e.tool0[0], 4)}, ${fmt(e.tool0[1], 4)}, ${fmt(e.tool0[2], 4)}`],
    ['Wrist centre', `${fmt(e.wrist_centre[0], 3)}, ${fmt(e.wrist_centre[1], 3)}, ${fmt(e.wrist_centre[2], 3)}`],
    ['Wrist-centre radius', `${fmt(e.wc_radius, 3)} m (${fmt(100 * e.wc_radius / S.cfg.reach_radius, 0)}% of reach)`],
    null,
    ['Roll / pitch / yaw', `${fmt(e.rpy_deg[0], 2)}°, ${fmt(e.rpy_deg[1], 2)}°, ${fmt(e.rpy_deg[2], 2)}°`],
    ['Tool tilt from vertical', `${fmt(e.tilt_deg, 3)}°`],
    ['Orientation error vs target', `${badge(sev, ICON[sev], fmt(oe, 3) + '°')}`],
    ['ψ = q2 − q3 + q5 (target 90°)', `${fmt(a.psi_deg, 2)}° (${fmtSigned(psiDev, 2)})`],
    ['q1 + q6 (target −180°)', `${fmt(a.q1_plus_q6_deg, 2)}° (${fmtSigned(sumDev, 2)})`],
    null,
    ['TCP speed (measured)', `${fmt(e.speed_mm_s, 1)} mm/s`],
    ['TCP speed (J·q̇)', `${fmt(vj, 1)} mm/s`],
  ];
  $('ee-kv').innerHTML = rows.map((r) => r ? `<dt>${r[0]}</dt><dd>${r[1]}</dd>` : '<div class="sep"></div>').join('');
}

function updateMission(st) {
  const m = st.mission, cfg = S.cfg;
  $('phase').textContent = m.phase;
  const g = (o) => `${o.status}${o.active ? ' (active)' : ''} · ${o.count} goal${o.count === 1 ? '' : 's'}`;
  $('arm-goal').textContent = g(m.arm_goal);
  $('grip-goal').textContent = g(m.gripper_goal);
  const n = cfg.waypoints.length;
  for (let k = 0; k <= n; k++) {
    const el = $(`step${k}`);
    const done = k <= m.progress && !(k === 0 && m.progress === 0 && m.arm_goal.active);
    const active = m.arm_goal.active && k === m.progress + 1;
    el.className = 'step' + (done ? ' done' : '') + (active ? ' active' : '');
    if (k > 0) {
      const rt = m.reach_times[k];
      $(`steps${k}`).textContent = rt !== undefined ? `reached t = ${fmt(rt, 2)} s` : (active ? 'moving…' : 'pending');
      $(`wpd${k}`).textContent = fmt(m.dist_mm[k - 1], 1);
      $(`wpt${k}`).textContent = rt !== undefined ? fmt(rt, 2) : '–';
      const wp = cfg.waypoints[k - 1];
      $(`wpst${k}`).innerHTML = !wp.within_limits ? badge('critical', '✕', 'out of limits')
        : rt !== undefined ? badge('good', '✓', 'reached') : active ? badge('info', '▶', 'active') : badge('neutral', '·', 'pending');
      $(`wprow${k}`).className = active ? 'active' : '';
    }
  }
  $('wpst0').innerHTML = m.progress === 0 && !m.arm_goal.active ? badge('info', '●', 'here') : badge('good', '✓', 'left');
  // measured segment times (arrival to arrival, includes the gripper wait)
  let total = null;
  for (let k = 1; k <= n; k++) {
    const t1 = m.reach_times[k], t0 = k === 1 ? m.start_t : m.reach_times[k - 1];
    const v = (t1 !== undefined && t0 !== undefined && t0 !== null) ? t1 - t0 : null;
    $(`segm${k}`).textContent = v === null ? '–' : fmt(v, 2);
  }
  if (m.reach_times[n] !== undefined && m.start_t !== null) total = m.reach_times[n] - m.start_t;
  $('segm-total').textContent = total === null ? '–' : `${fmt(total, 2)} (incl. gripper)`;
}

function updateGripper(st) {
  const g = st.gripper;
  const sev = { open: 'neutral', opening: 'info', closing: 'info', holding: 'good', closed: 'warning', partly: 'neutral', unknown: 'neutral' }[g.code] || 'neutral';
  const ic = { holding: '✓', closed: '!', opening: '▶', closing: '▶' }[g.code] || '·';
  $('grip-status').innerHTML = badge(sev, ic, g.text);
  const pct = g.closure_pct === null ? 0 : clamp(g.closure_pct, 0, 100);
  $('grip-meter').style.width = `${pct}%`;
  const rm = $('grip-ref-mark');
  if (g.ref !== null) { rm.style.display = 'block'; rm.style.left = `${clamp(100 * g.ref / S.cfg.gripper_closed, 0, 100)}%`; } else rm.style.display = 'none';
  $('grip-kv').innerHTML = [
    ['Knuckle angle', g.pos === null ? '–' : `${fmt(g.pos, 3)} rad (${fmt(g.pos * 180 / Math.PI, 1)}°)`],
    ['Closure', g.closure_pct === null ? '–' : `${fmt(g.closure_pct, 0)} %`],
    ['Commanded (controller ref)', g.ref === null ? '–' : `${fmt(g.ref, 3)} rad`],
    ['Velocity', g.vel === null ? '–' : `${fmtSigned(g.vel, 3)} rad/s`],
    ['Plan', 'close 0.75 at WP2 · open 0.0 at WP5'],
  ].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
}

function boxBEstimate(st) {
  const live = st.boxes && st.boxes.box_b;
  if (live) return { xyz: live.xyz, rpy: live.rpy_deg, tilt: live.tilt_deg, src: 'live' };
  const m = st.mission, cfgB = S.cfg.boxes.box_b;
  if (st.gripper.code === 'holding' && m.progress >= 2) {
    return { xyz: [st.ee.tcp[0], st.ee.tcp[1], st.ee.tcp[2] - 0.025], rpy: st.ee.rpy_deg, tilt: null, src: 'estimated' };
  }
  if (m.progress >= 5) {
    const w = S.cfg.waypoints[4].xyz;
    return { xyz: [w[0], w[1], S.cfg.boxes.box_a.pose[2] * 2 + 0.025], rpy: [0, 0, 0], tilt: null, src: 'estimated' };
  }
  return cfgB ? { xyz: cfgB.pose, rpy: [0, 0, 0], tilt: null, src: 'spawn pose' } : null;
}

function updateBoxes(st) {
  const A = S.cfg.boxes.box_a, b = boxBEstimate(st);
  $('box-source').textContent = (st.boxes && st.boxes.box_b) ? 'live pose from Gazebo' : 'no live pose (bridge off) · estimated';
  let status = badge('neutral', '·', 'Unknown');
  if (b && A) {
    const aTop = A.pose[2] + A.size[2] / 2;
    const dxy = Math.hypot(b.xyz[0] - A.pose[0], b.xyz[1] - A.pose[1]);
    const onA = dxy < A.size[0] / 2 && Math.abs(b.xyz[2] - (aTop + 0.025)) < 0.02;
    if (onA) status = badge('good', '✓', 'Placed on Box A');
    else if (b.xyz[2] > 0.04) status = badge('info', '▶', st.gripper.code === 'holding' ? 'Lifted, held by gripper' : 'In the air');
    else status = badge('neutral', '·', 'On the ground');
    if (b.tilt !== null && b.tilt > 5) status += ' ' + badge('critical', '✕', `tilted ${fmt(b.tilt, 1)}°`);
  }
  $('box-status').innerHTML = status;
  const rows = [];
  if (A) {
    rows.push(['Box A centre (SDF)', `${fmt(A.pose[0], 3)}, ${fmt(A.pose[1], 3)}, ${fmt(A.pose[2], 3)}`]);
    rows.push(['Box A size', `${A.size.map((v) => fmt(v * 100, 0)).join(' × ')} cm · top z = ${fmt(A.pose[2] + A.size[2] / 2, 3)} m`]);
  }
  const B0 = S.cfg.boxes.box_b;
  if (B0) rows.push(['Box B spawn (SDF)', `${fmt(B0.pose[0], 3)}, ${fmt(B0.pose[1], 3)}, ${fmt(B0.pose[2], 3)}`]);
  rows.push(null);
  if (b) {
    rows.push([`Box B now (${b.src})`, `${fmt(b.xyz[0], 4)}, ${fmt(b.xyz[1], 4)}, ${fmt(b.xyz[2], 4)}`]);
    rows.push(['Box B roll / pitch / yaw', b.rpy ? b.rpy.map((v) => fmt(v, 2) + '°').join(', ') : '–']);
    rows.push(['Box B tilt from upright', b.tilt === null ? '–' : `${fmt(b.tilt, 3)}°`]);
    if (A) rows.push(['Box B offset from Box A centre', `${fmt(1000 * (b.xyz[0] - A.pose[0]), 1)}, ${fmt(1000 * (b.xyz[1] - A.pose[1]), 1)} mm`]);
  }
  $('box-kv').innerHTML = rows.map((r) => r ? `<dt>${r[0]}</dt><dd>${r[1]}</dd>` : '<div class="sep"></div>').join('');
}

function updateSystem(st) {
  const j = st.jac, w = j.w;
  const sev = w < 0.01 ? 'critical' : w < 0.03 ? 'warning' : 'good';
  const txt = w < 0.01 ? 'Near a singularity' : w < 0.03 ? 'Low manipulability' : 'Well-conditioned';
  $('jac-status').innerHTML = `${badge(sev, ICON[sev], txt)} <span class="mono" style="font-size:14px">w = ${fmt(w, 4)}</span>`;
  const smax = Math.max(...j.sigma, 1e-9);
  $('sv-bars').innerHTML = j.sigma.map((s, i) => `<span class="k">σ${i + 1}</span>
    <div class="svtrack"><div class="svbar" style="width:${100 * s / smax}%"></div></div><span class="mono">${fmt(s, 4)}</span>`).join('');
  const l = st.link;
  $('sys-kv').innerHTML = [
    ['Condition number', j.cond === null ? '∞' : fmt(j.cond, 1)],
    ['/joint_states rate', `${l.joint_states_hz} Hz`],
    ['Last joint_states', `${fmt(1000 * l.joint_states_age_s, 0)} ms ago`],
    ['TCP source', st.ee.source === 'tf' ? 'TF (robot_state_publisher)' : 'our FK'],
    ['Box pose stream', l.box_pose_age_s === null ? 'off' : `${fmt(l.box_pose_age_s, 1)} s ago`],
  ].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  $('ctrl-list').innerHTML = (st.controllers && st.controllers.length)
    ? st.controllers.map((c) => `<div class="ctrl"><span class="mono">${esc(c.name)}</span>${c.state === 'active' ? badge('good', '✓', 'active') : badge('warning', '!', c.state)}</div>`).join('')
    : `<div class="ctrl"><span class="k">controller_manager</span>${badge('warning', '!', 'not reachable')}</div>`;
}

// ------------------------------------------------------------------- charts
function drawLineChart(cv, tipEl, key, getVals, opts) {
  const { ctx, w, h } = setupCanvas(cv);
  const C = S.C, H = S.history;
  const pad = { l: 46, r: 12, t: 8, b: 24 };
  const pw = w - pad.l - pad.r, ph = h - pad.t - pad.b;
  if (!H.length) {
    ctx.fillStyle = C.muted; ctx.font = '12px system-ui'; ctx.fillText('no data yet', pad.l + 8, pad.t + 20);
    return;
  }
  const tEnd = H[H.length - 1].t, tStart = tEnd - WINDOW_S;
  let i0 = H.length - 1; while (i0 > 0 && H[i0 - 1].t >= tStart) i0--;
  const win = H.slice(i0);
  const nS = opts.nSeries, hidden = opts.hidden || new Set();
  let lo = Infinity, hi = -Infinity;
  for (const s of win) { const v = getVals(s); for (let k = 0; k < nS; k++) if (!hidden.has(k) && v[k] !== null) { lo = Math.min(lo, v[k]); hi = Math.max(hi, v[k]); } }
  (opts.hlines || []).forEach((y) => { lo = Math.min(lo, y); hi = Math.max(hi, y); });
  if (opts.yMin !== undefined) lo = Math.min(lo, opts.yMin);
  if (opts.yMax !== undefined) hi = Math.max(hi, opts.yMax);
  if (!(hi > lo)) { lo -= 1; hi += 1; }
  const m = (hi - lo) * 0.08; lo -= m; hi += m;
  const X = (t) => pad.l + (t - tStart) / WINDOW_S * pw;
  const Y = (v) => pad.t + (1 - (v - lo) / (hi - lo)) * ph;

  // grid + axes
  ctx.font = '11px system-ui'; ctx.textBaseline = 'middle';
  const yt = niceTicks(lo, hi, Math.max(3, Math.floor(ph / 40)));
  yt.ticks.forEach((v) => {
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(pad.l, Math.round(Y(v)) + 0.5); ctx.lineTo(w - pad.r, Math.round(Y(v)) + 0.5); ctx.stroke();
    ctx.fillStyle = C.muted; ctx.textAlign = 'right'; ctx.fillText(Number(v.toFixed(6)).toString(), pad.l - 6, Y(v));
  });
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  const xt = niceTicks(tStart, tEnd, Math.max(3, Math.floor(pw / 80)));
  xt.ticks.forEach((t) => { ctx.fillStyle = C.muted; ctx.fillText(`${t.toFixed(xt.step < 1 ? 1 : 0)} s`, X(t), h - pad.b + 6); });
  if (lo < 0 && hi > 0) { ctx.strokeStyle = C.axis; ctx.beginPath(); ctx.moveTo(pad.l, Math.round(Y(0)) + 0.5); ctx.lineTo(w - pad.r, Math.round(Y(0)) + 0.5); ctx.stroke(); }
  (opts.hlines || []).forEach((y) => {
    ctx.save(); ctx.setLineDash([4, 4]); ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.moveTo(pad.l, Y(y)); ctx.lineTo(w - pad.r, Y(y)); ctx.stroke(); ctx.restore();
  });

  // series
  ctx.save(); ctx.beginPath(); ctx.rect(pad.l, pad.t, pw, ph); ctx.clip();
  ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  for (let k = 0; k < nS; k++) {
    if (hidden.has(k)) continue;
    ctx.strokeStyle = opts.colors[k]; ctx.beginPath();
    let started = false;
    for (const s of win) { const v = getVals(s)[k]; if (v === null) { started = false; continue; } const x = X(s.t), y = Y(v); started ? ctx.lineTo(x, y) : ctx.moveTo(x, y); started = true; }
    ctx.stroke();
  }
  ctx.restore();

  // hover crosshair + tooltip
  const hx = S.hover[key];
  if (hx !== null && hx >= pad.l && hx <= w - pad.r) {
    const tH = tStart + (hx - pad.l) / pw * WINDOW_S;
    let best = win[0]; for (const s of win) if (Math.abs(s.t - tH) < Math.abs(best.t - tH)) best = s;
    const x = X(best.t);
    ctx.strokeStyle = C.ink2; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x + 0.5, pad.t); ctx.lineTo(x + 0.5, pad.t + ph); ctx.stroke();
    const v = getVals(best);
    for (let k = 0; k < nS; k++) {
      if (hidden.has(k) || v[k] === null) continue;
      ctx.beginPath(); ctx.arc(x, Y(v[k]), 4, 0, 2 * Math.PI); ctx.fillStyle = opts.colors[k]; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = C.surface; ctx.stroke();
    }
    tipEl.innerHTML = `<div class="tt-h">t = ${best.t.toFixed(2)} s</div>` + opts.labels.map((l, k) => hidden.has(k) || v[k] === null ? '' :
      `<div class="tt-r"><span><span class="swatch" style="display:inline-block;background:${opts.colors[k]}"></span> ${esc(l)}</span><span>${fmt(v[k], opts.dp)}${opts.unit}</span></div>`).join('');
    tipEl.style.display = 'block';
    const tw = tipEl.offsetWidth;
    tipEl.style.left = `${x + 14 + tw > w ? x - tw - 14 : x + 14}px`; tipEl.style.top = `${pad.t + 4}px`;
  } else tipEl.style.display = 'none';
}

function hookHover(cv, key) {
  cv.addEventListener('mousemove', (e) => { S.hover[key] = e.offsetX; S.dirty = true; });
  cv.addEventListener('mouseleave', () => { S.hover[key] = null; S.dirty = true; });
}

// -------------------------------------------------------------- 3D / 2D views
function camBasis() {
  const az = S.cam.az * Math.PI / 180, el = S.cam.el * Math.PI / 180;
  return {
    d: [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)],
    r: [-Math.sin(az), Math.cos(az), 0],
    u: [-Math.sin(el) * Math.cos(az), -Math.sin(el) * Math.sin(az), Math.cos(el)],
  };
}
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

function rpyToR(rpyDeg) {
  const [r, p, y] = rpyDeg.map((v) => v * Math.PI / 180);
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]];
}
function boxCorners(c, size, R) {
  const out = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    const l = [sx * size[0] / 2, sy * size[1] / 2, sz * size[2] / 2];
    out.push([0, 1, 2].map((i) => c[i] + R[i][0] * l[0] + R[i][1] * l[1] + R[i][2] * l[2]));
  }
  return out;
}
const BOX_FACES = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]];
const I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

function draw3D() {
  const cv = $('view3d');
  const { ctx, w, h } = setupCanvas(cv);
  const C = S.C, st = S.last, cfg = S.cfg, B = camBasis();
  const target = [0.35, 0.12, 0.3];
  const scale = Math.min(w, h) / 1.55 * S.cam.zoom;
  const P = (p) => {
    const q = [p[0] - target[0], p[1] - target[1], p[2] - target[2]];
    return [w / 2 + scale * dot3(q, B.r), h / 2 - scale * dot3(q, B.u), dot3(q, B.d)];
  };
  const line = (a, b, color, width = 1, dash) => {
    const pa = P(a), pb = P(b); ctx.save(); if (dash) ctx.setLineDash(dash);
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke(); ctx.restore();
  };
  const poly = (pts, color, width = 2, dash, alpha = 1) => {
    if (pts.length < 2) return; ctx.save(); if (dash) ctx.setLineDash(dash); ctx.globalAlpha = alpha;
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = 'round'; ctx.lineCap = 'round'; ctx.beginPath();
    pts.forEach((p, i) => { const s = P(p); i ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1]); }); ctx.stroke(); ctx.restore();
  };
  // ground grid
  for (let v = -0.6; v <= 1.2001; v += 0.1) {
    const major = Math.abs(v * 2 - Math.round(v * 2)) < 1e-6;
    line([v, -0.8, 0], [v, 1.0, 0], major ? C.axis : C.grid, 1);
  }
  for (let v = -0.8; v <= 1.0001; v += 0.1) {
    const major = Math.abs(v * 2 - Math.round(v * 2)) < 1e-6;
    line([-0.6, v, 0], [1.2, v, 0], major ? C.axis : C.grid, 1);
  }
  // reach ring (horizontal wrist-centre reach) on the ground
  const ring = []; for (let a = 0; a <= 360; a += 5) ring.push([cfg.reach_radius * Math.cos(a * Math.PI / 180), cfg.reach_radius * Math.sin(a * Math.PI / 180), 0]);
  poly(ring, C.muted, 1, [5, 5], 0.8);
  // world axes
  line([0, 0, 0], [0.25, 0, 0], C.s8, 2.5); line([0, 0, 0], [0, 0.25, 0], C.series[5], 2.5); line([0, 0, 0], [0, 0, 0.25], C.series[0], 2.5);
  ctx.font = '11px system-ui'; ctx.fillStyle = C.ink2;
  [['x', [0.28, 0, 0]], ['y', [0, 0.28, 0]], ['z', [0, 0, 0.28]]].forEach(([l, p]) => { const s = P(p); ctx.fillText(l, s[0] - 3, s[1] + 4); });

  // boxes
  const drawBox = (c, size, R, color, label) => {
    const cs = boxCorners(c, size, R).map(P);
    const faces = BOX_FACES.map((f) => ({ f, z: f.reduce((s, i) => s + cs[i][2], 0) / 4 })).sort((a, b) => a.z - b.z);
    faces.forEach(({ f }) => {
      ctx.beginPath(); f.forEach((i, k) => k ? ctx.lineTo(cs[i][0], cs[i][1]) : ctx.moveTo(cs[i][0], cs[i][1])); ctx.closePath();
      ctx.globalAlpha = 0.22; ctx.fillStyle = color; ctx.fill(); ctx.globalAlpha = 1; ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
    });
    if (label) { const s = P([c[0], c[1], c[2] + size[2] / 2 + 0.03]); ctx.fillStyle = C.ink2; ctx.font = '12px system-ui'; ctx.textAlign = 'center'; ctx.fillText(label, s[0], s[1]); ctx.textAlign = 'left'; }
  };
  const A = cfg.boxes.box_a;
  if (A) drawBox(A.pose, A.size, I3, C.boxA, 'Box A');

  // planned path & waypoints
  cfg.planned_path.forEach((seg) => poly(seg, C.plan, 1.5, [4, 4], 0.7));
  cfg.waypoints.forEach((wp) => {
    const s = P(wp.xyz); ctx.beginPath(); ctx.arc(s[0], s[1], 4.5, 0, 2 * Math.PI); ctx.fillStyle = C.accent; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = C.surface2; ctx.stroke();
  });
  if (st && st.type === 'state') {
    const b = boxBEstimate(st);
    if (b) drawBox(b.xyz, cfg.boxes.box_b ? cfg.boxes.box_b.size : [0.05, 0.05, 0.05], rpyToR(b.rpy || [0, 0, 0]), C.boxB, 'Box B');
  } else if (cfg.boxes.box_b) drawBox(cfg.boxes.box_b.pose, cfg.boxes.box_b.size, I3, C.boxB, 'Box B');

  // trail
  poly(S.trail, C.trail, 2);

  if (st && st.type === 'state') {
    const sk = st.ee.skeleton;
    // shadow on the ground
    poly(sk.map((p) => [p[0], p[1], 0]), C.axis, 5, null, 0.6);
    // base
    const base = []; for (let a = 0; a <= 360; a += 15) base.push([0.12 * Math.cos(a * Math.PI / 180), 0.12 * Math.sin(a * Math.PI / 180), 0]);
    poly(base, C.arm, 2);
    // links
    const widths = [9, 8, 7, 6, 5, 4, 3];
    for (let i = 0; i < sk.length - 1; i++) {
      const a = P(sk[i]), b2 = P(sk[i + 1]);
      ctx.strokeStyle = C.arm; ctx.lineWidth = widths[Math.min(i, widths.length - 1)]; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b2[0], b2[1]); ctx.stroke();
    }
    sk.slice(0, 7).forEach((p, i) => {
      const s = P(p); ctx.beginPath(); ctx.arc(s[0], s[1], i === 0 ? 5 : 5, 0, 2 * Math.PI);
      ctx.fillStyle = i === 0 ? C.arm : (C.series[i - 1] || C.arm); ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = C.surface2; ctx.stroke();
    });
    // gripper fingers
    const R = st.ee.R, tcp = st.ee.tcp, zt = [R[0][2], R[1][2], R[2][2]], yt = [R[0][1], R[1][1], R[2][1]];
    const closure = st.gripper.closure_pct === null ? 0 : clamp(st.gripper.closure_pct / 100, 0, 1);
    const half = 0.0425 * (1 - closure) + 0.006;
    const at = (o, s, zz) => [0, 1, 2].map((i) => tcp[i] + yt[i] * o * s + zt[i] * zz);
    [1, -1].forEach((sgn) => { line(at(half, sgn, -0.045), at(half, sgn, 0.012), C.ink, 3); });
    line(at(half, 1, -0.045), at(half, -1, -0.045), C.ink, 3);
    // TCP frame
    const tri = (col, k, c) => line(tcp, [0, 1, 2].map((i) => tcp[i] + 0.08 * R[i][k]), c, 2);
    tri(null, 0, C.s8); tri(null, 1, C.series[5]); tri(null, 2, C.series[0]);
    const s = P(tcp); ctx.beginPath(); ctx.arc(s[0], s[1], 4, 0, 2 * Math.PI); ctx.fillStyle = C.trail; ctx.fill();
  }
  ctx.fillStyle = C.muted; ctx.font = '11px system-ui';
  ctx.fillText(`az ${S.cam.az.toFixed(0)}°  el ${S.cam.el.toFixed(0)}°  ·  grid 0.1 m`, 10, h - 10);
}

function draw2D(cv, axes, bounds, extra) {
  const { ctx, w, h } = setupCanvas(cv);
  const C = S.C, cfg = S.cfg, st = S.last;
  const [u, v] = axes; // index into xyz for horizontal / vertical
  const [u0, u1, v0, v1] = bounds;
  const pad = 26;
  const sc = Math.min((w - 2 * pad) / (u1 - u0), (h - 2 * pad) / (v1 - v0));
  const ox = (w - sc * (u1 - u0)) / 2, oy = (h - sc * (v1 - v0)) / 2;
  const X = (p) => ox + (p[u] - u0) * sc, Y = (p) => h - oy - (p[v] - v0) * sc;
  const mk = (a, b) => { const p = [0, 0, 0]; p[u] = a; p[v] = b; return p; };
  // grid
  ctx.font = '10.5px system-ui';
  for (let a = Math.ceil(u0 * 10) / 10; a <= u1 + 1e-9; a += 0.1) {
    const major = Math.abs(a * 4 - Math.round(a * 4)) < 1e-6;
    ctx.strokeStyle = major ? C.axis : C.grid; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(X(mk(a, 0)), Y(mk(0, v0))); ctx.lineTo(X(mk(a, 0)), Y(mk(0, v1))); ctx.stroke();
    if (major) { ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.fillText(a.toFixed(2), X(mk(a, 0)), h - oy + 12); }
  }
  for (let b = Math.ceil(v0 * 10) / 10; b <= v1 + 1e-9; b += 0.1) {
    const major = Math.abs(b * 4 - Math.round(b * 4)) < 1e-6;
    ctx.strokeStyle = major ? C.axis : C.grid;
    ctx.beginPath(); ctx.moveTo(X(mk(u0, 0)), Y(mk(0, b))); ctx.lineTo(X(mk(u1, 0)), Y(mk(0, b))); ctx.stroke();
    if (major) { ctx.fillStyle = C.muted; ctx.textAlign = 'right'; ctx.fillText(b.toFixed(2), ox - 4, Y(mk(0, b)) + 3); }
  }
  const names = ['x', 'y', 'z'];
  ctx.fillStyle = C.ink2; ctx.textAlign = 'left'; ctx.fillText(`${names[u]} →`, w - ox - 22, h - oy - 4); ctx.fillText(`${names[v]} ↑`, ox + 4, oy + 10);
  ctx.save(); ctx.beginPath(); ctx.rect(ox, oy, sc * (u1 - u0), sc * (v1 - v0)); ctx.clip();
  const poly = (pts, color, width, dash, alpha = 1) => {
    if (pts.length < 2) return; ctx.save(); if (dash) ctx.setLineDash(dash); ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width;
    ctx.lineJoin = 'round'; ctx.lineCap = 'round'; ctx.beginPath(); pts.forEach((p, i) => i ? ctx.lineTo(X(p), Y(p)) : ctx.moveTo(X(p), Y(p))); ctx.stroke(); ctx.restore();
  };
  const rect = (c, size, color) => {
    const x0 = X(mk(c[u] - size[u] / 2, 0)), x1 = X(mk(c[u] + size[u] / 2, 0)), y0 = Y(mk(0, c[v] - size[v] / 2)), y1 = Y(mk(0, c[v] + size[v] / 2));
    ctx.globalAlpha = 0.22; ctx.fillStyle = color; ctx.fillRect(x0, y1, x1 - x0, y0 - y1); ctx.globalAlpha = 1;
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.strokeRect(x0, y1, x1 - x0, y0 - y1);
  };
  if (extra) extra({ ctx, X, Y, mk, poly, sc, C });
  const A = cfg.boxes.box_a; if (A) rect(A.pose, A.size, C.boxA);
  if (st && st.type === 'state') { const b = boxBEstimate(st); if (b) rect(b.xyz, cfg.boxes.box_b ? cfg.boxes.box_b.size : [0.05, 0.05, 0.05], C.boxB); }
  cfg.planned_path.forEach((seg) => poly(seg, C.plan, 1.5, [4, 4], 0.7));
  poly(S.trail, C.trail, 2);
  cfg.waypoints.forEach((wp) => {
    ctx.beginPath(); ctx.arc(X(wp.xyz), Y(wp.xyz), 4, 0, 2 * Math.PI); ctx.fillStyle = C.accent; ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = C.surface2; ctx.stroke();
  });
  if (st && st.type === 'state') {
    const sk = st.ee.skeleton;
    poly(sk, C.arm, 4);
    sk.slice(1, 7).forEach((p, i) => { ctx.beginPath(); ctx.arc(X(p), Y(p), 3.5, 0, 2 * Math.PI); ctx.fillStyle = C.series[i]; ctx.fill(); });
    const t = st.ee.tcp; ctx.beginPath(); ctx.arc(X(t), Y(t), 5, 0, 2 * Math.PI); ctx.fillStyle = C.trail; ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = C.surface2; ctx.stroke();
  }
  ctx.restore();
}

function drawAll() {
  if (!S.cfg) return;
  const labels = S.cfg.joint_labels, cols = S.C.series;
  drawLineChart($('chartAngles'), $('tip-angles'), 'angles', (s) => s.q, { nSeries: 6, colors: cols, labels, unit: '°', dp: 2, hidden: S.hidden.angles });
  const vm = S.cfg.v_max_deg;
  drawLineChart($('chartVels'), $('tip-vels'), 'vels', (s) => s.qd, { nSeries: 6, colors: cols, labels, unit: ' °/s', dp: 2, hidden: S.hidden.vels, hlines: [vm, -vm] });
  drawLineChart($('chartGrip'), $('tip-grip'), 'grip', (s) => [s.g], { nSeries: 1, colors: [S.C.accent], labels: ['knuckle'], unit: ' rad', dp: 3, yMin: 0, yMax: 0.8 });
  draw3D();
  draw2D($('viewTop'), [0, 1], [-0.3, 1.0, -0.6, 0.85], ({ ctx, X, Y, mk, sc, C }) => {
    ctx.save(); ctx.setLineDash([5, 5]); ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.arc(X(mk(0, 0)), Y(mk(0, 0)), S.cfg.reach_radius * sc, 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
    ctx.beginPath(); ctx.arc(X(mk(0, 0)), Y(mk(0, 0)), 0.12 * sc, 0, 2 * Math.PI); ctx.strokeStyle = C.arm; ctx.lineWidth = 1.5; ctx.stroke();
  });
  draw2D($('viewSide'), [1, 2], [-0.6, 0.85, -0.02, 1.0], ({ ctx, X, Y, mk, C }) => {
    ctx.strokeStyle = C.ink2; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(X(mk(-1, 0)), Y(mk(0, 0))); ctx.lineTo(X(mk(1.5, 0)), Y(mk(0, 0))); ctx.stroke();
  });
}

function loop() {
  if (S.dirty && !document.hidden) { S.dirty = false; try { drawAll(); } catch (e) { console.error(e); } }
  requestAnimationFrame(loop);
}

// ---------------------------------------------------------------- controls
function hook3D() {
  const cv = $('view3d'); let drag = null;
  cv.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY, az: S.cam.az, el: S.cam.el }; cv.setPointerCapture(e.pointerId); });
  cv.addEventListener('pointermove', (e) => {
    if (!drag) return;
    S.cam.az = drag.az - (e.clientX - drag.x) * 0.4;
    S.cam.el = clamp(drag.el + (e.clientY - drag.y) * 0.3, -5, 89);
    S.dirty = true;
  });
  cv.addEventListener('pointerup', () => { drag = null; });
  cv.addEventListener('wheel', (e) => { e.preventDefault(); S.cam.zoom = clamp(S.cam.zoom * Math.exp(-e.deltaY * 0.001), 0.4, 4); S.dirty = true; }, { passive: false });
  cv.addEventListener('dblclick', () => { S.cam = { az: -38, el: 24, zoom: 1 }; S.dirty = true; });
}

function downloadCSV() {
  const head = ['t_s', ...[1, 2, 3, 4, 5, 6].map((i) => `q${i}_deg`), ...[1, 2, 3, 4, 5, 6].map((i) => `qd${i}_deg_s`),
    'tcp_x_m', 'tcp_y_m', 'tcp_z_m', 'orient_err_deg', 'gripper_rad', 'phase'];
  const rows = S.history.map((s) => [s.t.toFixed(4), ...s.q.map((v) => v.toFixed(4)), ...s.qd.map((v) => v.toFixed(4)),
    ...s.tcp.map((v) => v.toFixed(5)), s.oe.toFixed(5), s.g === null ? '' : s.g.toFixed(4), `"${s.ph}"`].join(','));
  const blob = new Blob([head.join(',') + '\n' + rows.join('\n') + '\n'], { type: 'text/csv' });
  const a = document.createElement('a'); a.href = URL.createObjectURL(blob);
  a.download = `gp7_run_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.csv`; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}

function initTheme() {
  let saved = null; try { saved = localStorage.getItem('gp7-theme'); } catch (e) { /* storage blocked */ }
  if (saved) document.documentElement.dataset.theme = saved;
  $('btn-theme').addEventListener('click', () => {
    const cur = document.documentElement.dataset.theme || (matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
    const next = cur === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    try { localStorage.setItem('gp7-theme', next); } catch (e) { /* ignore */ }
    readColors();
  });
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', readColors);
}

async function init() {
  initTheme(); readColors();
  document.querySelectorAll('.win-s').forEach((e) => { e.textContent = WINDOW_S; });
  for (;;) {
    try { S.cfg = await (await fetch('/api/config')).json(); break; }
    catch (e) { $('link-text').textContent = 'Dashboard server not reachable'; $('pill-link').className = 'pill down'; await new Promise((r) => setTimeout(r, 1500)); }
  }
  buildStatic(S.cfg);
  hookHover($('chartAngles'), 'angles'); hookHover($('chartVels'), 'vels'); hookHover($('chartGrip'), 'grip'); hook3D();
  $('btn-pause').addEventListener('click', (e) => { S.paused = !S.paused; e.target.classList.toggle('on', S.paused); e.target.textContent = S.paused ? 'Resume' : 'Pause'; S.dirty = true; });
  $('btn-csv').addEventListener('click', downloadCSV);
  $('btn-clear').addEventListener('click', () => { S.history = []; S.trail = []; S.dirty = true; });
  new ResizeObserver(() => { S.dirty = true; }).observe(document.body);

  requestAnimationFrame(loop);
  poll();
}

// 20 Hz polling of /api/state (simple, and robust behind any proxy)
async function poll() {
  const period = 1000 / 20;
  for (;;) {
    const t0 = performance.now();
    try {
      const r = await fetch('/api/state', { cache: 'no-store' });
      const st = await r.json();
      if (!S.paused) onState(st);
    } catch (e) {
      $('pill-link').className = 'pill down'; $('link-text').textContent = 'Dashboard server stopped · retrying…';
      await new Promise((res) => setTimeout(res, 1000));
    }
    await new Promise((res) => setTimeout(res, Math.max(0, period - (performance.now() - t0))));
  }
}
init();
