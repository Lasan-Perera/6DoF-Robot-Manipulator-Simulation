"""
Live web dashboard for the GP7 pick-and-place.

Subscribes to the running simulation (joint states, TF, controller states,
action status, Gazebo model poses), computes FK / orientation error /
manipulability with this package's own kinematics, and serves a browser UI
that polls /api/state at 20 Hz.  Python standard library only:
no rosbridge, no pip packages.

Run (Gazebo must be running; system Python, not conda):
    cd src/gp7_pick_place/gp7_pick_place
    /usr/bin/python3 dashboard.py            # then open http://localhost:8080
Options: --port 8080  --host 127.0.0.1  --rate 20  --no-box-pose
"""
import argparse
import json
import os
import re
import signal
import sys
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from action_msgs.msg import GoalStatusArray, GoalStatus
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener

try:  # Gazebo's own Python bindings: model poses with their names
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
except ImportError:
    GzNode = None

import kinematics as kin
import trajectory as traj
import jacobian as jac
import run_pick_place as mission

HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(HERE, 'dashboard_web')
WORLD_SDF = os.path.normpath(os.path.join(
    HERE, '..', '..', 'gp7_description', 'worlds', 'pick_place.sdf'))
WORLD_NAME = 'pick_place_world'
MODEL_POSE_TOPIC = f'/world/{WORLD_NAME}/dynamic_pose/info'

ARM = mission.ARM_JOINTS
GRIP = mission.GRIPPER_JOINT[0]
R_DOWN = np.diag([1.0, -1.0, -1.0])
HOME = [0.0] * 6
WP_LABELS = ['Above Box B', 'Grasp Box B', 'Lift', 'Above Box A', 'Release on Box A', 'Retreat']
REACH_RADIUS = 0.040 + 0.445 + 0.4418      # max horizontal wrist-centre reach (m)
GRIPPER_CLOSED = 0.8                        # knuckle angle when fully closed (rad)
WP_TOL = 0.0025                             # "waypoint reached" tolerance (m)
DT = 0.05                                   # trajectory sample time used by the mission

STATUS_NAMES = {0: 'unknown', 1: 'accepted', 2: 'executing', 3: 'canceling',
                4: 'succeeded', 5: 'canceled', 6: 'aborted'}


# ----------------------------------------------------------------- static plan
def parse_boxes(path=WORLD_SDF):
    boxes = {}
    try:
        root = ET.parse(path).getroot()
        for m in root.iter('model'):
            name = m.get('name')
            if name not in ('box_a', 'box_b'):
                continue
            pose = [float(v) for v in m.find('pose').text.split()]
            size = [float(v) for v in m.find('.//visual/geometry/box/size').text.split()]
            static = (m.findtext('static') or 'false').strip() == 'true'
            boxes[name] = {'pose': pose[:3], 'size': size, 'static': static}
    except Exception as e:  # never stop the dashboard over a parse problem
        print(f'[dashboard] could not parse {path}: {e}', file=sys.stderr)
    return boxes


def tcp_of(q):
    T = kin.forward_kinematics(q)
    return T[:3, 3] + kin.L_TCP * T[:3, 2], T


def skeleton(q):
    """Base, D-H frame origins 1..6, then the TCP (for drawing the arm)."""
    Ts = jac.all_frame_transforms(q)
    pts = [T[:3, 3] for T in Ts]
    T6 = Ts[-1]
    pts.append(T6[:3, 3] + kin.L_TCP * T6[:3, 2])
    return pts, T6


def build_plan():
    sols = [list(map(float, q)) for q in mission.solve_waypoints()]
    waypoints = []
    for i, (xyz, q) in enumerate(zip(mission.WAYPOINTS_XYZ, sols)):
        tcp, _ = tcp_of(q)
        waypoints.append({
            'index': i + 1,
            'label': WP_LABELS[i] if i < len(WP_LABELS) else f'Waypoint {i + 1}',
            'xyz': [float(v) for v in xyz],
            'q_deg': [float(np.degrees(v)) for v in q],
            'ik_error_mm': float(np.linalg.norm(tcp - np.array(xyz)) * 1000),
            'within_limits': bool(kin.within_limits(q)),
            'gripper_after': mission.GRIPPER_ACTIONS.get(i, None),
        })
    segments, path = [], []
    chain = [HOME] + sols
    names = ['Home'] + [w['label'] for w in waypoints]
    for k in range(len(chain) - 1):
        pts = traj.generate_segment_trajectory(list(chain[k]), list(chain[k + 1]),
                                               mission.V_MAX, mission.ACCEL, DT)
        dq = np.degrees(np.abs(np.array(chain[k + 1]) - np.array(chain[k])))
        seg_path = [tcp_of(q)[0].tolist() for q in pts[::2]] + [tcp_of(pts[-1])[0].tolist()]
        path.append(seg_path)
        segments.append({
            'from': names[k], 'to': names[k + 1], 'to_index': k + 1,
            'duration_s': round((len(pts) - 1) * DT, 3),
            'pacing_joint': int(np.argmax(dq)) + 1,
            'max_dq_deg': float(dq.max()),
            'gripper_after': mission.GRIPPER_ACTIONS.get(k, None),
        })
    return {
        'arm_joints': ARM,
        'gripper_joint': GRIP,
        'joint_labels': ['J1 S', 'J2 L', 'J3 U', 'J4 R', 'J5 B', 'J6 T'],
        'limits_deg': kin.JOINT_LIMITS_DEG,
        'dh': [{'a': a, 'alpha': al, 'd': d} for a, al, d in kin.DH_PARAMS],
        'dh_theta': ['q1', '90° − q2', 'q3', '−q4', 'q5', '−q6'],
        'l_tcp': kin.L_TCP,
        'v_max': mission.V_MAX, 'accel': mission.ACCEL, 'dt': DT,
        'v_max_deg': float(np.degrees(mission.V_MAX)),
        'r_target': R_DOWN.tolist(),
        'reach_radius': REACH_RADIUS,
        'gripper_closed': GRIPPER_CLOSED,
        'gripper_actions': {str(k): v for k, v in mission.GRIPPER_ACTIONS.items()},
        'home_deg': HOME,
        'waypoints': waypoints,
        'segments': segments,
        'planned_path': path,
        'boxes': parse_boxes(),
        'world_sdf': os.path.relpath(WORLD_SDF, os.path.join(HERE, '..', '..', '..')),
        'total_planned_s': round(sum(s['duration_s'] for s in segments), 2),
    }


# ------------------------------------------------------------------ math utils
def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_deg(R):
    pitch = np.arcsin(np.clip(-R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return [float(np.degrees(v)) for v in (roll, pitch, yaw)]


def rot_angle_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def tilt_deg(R, ref_axis=np.array([0, 0, -1.0])):
    """Angle between the frame's z axis and ref_axis."""
    return float(np.degrees(np.arccos(np.clip(R[:, 2] @ ref_axis, -1, 1))))


def box_tilt_deg(R):
    """Cube tilt: smallest angle between any cube axis and world z (0 = flat)."""
    return float(np.degrees(np.arccos(np.clip(np.max(np.abs(R[2, :])), -1, 1))))


# ------------------------------------------------------------------- ROS node
class DashboardNode(Node):
    def __init__(self, plan, rate_hz):
        super().__init__('gp7_dashboard',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.plan = plan
        self.lock = threading.Lock()
        self.snapshot = json.dumps({'type': 'waiting'})
        self.seq = 0

        self.js = None
        self.js_wall = 0.0
        self.arm_state = None
        self.grip_state = None
        self.arm_goal = {'status': 'none', 'active': False, 'count': 0}
        self.grip_goal = {'status': 'none', 'active': False, 'count': 0}
        self._seen_goals = {'arm': set(), 'grip': set()}
        self.controllers = []
        self.controllers_wall = 0.0
        self.boxes_live = {}
        self.boxes_wall = 0.0

        self.prev_tcp = None
        self.prev_t = None
        self.tcp_speed = 0.0
        self.progress = 0          # 0 = home/start, k = waypoint k reached
        self.reach_times = {}      # waypoint index -> sim time reached
        self.mission_start = None
        self.last_t = None
        self.rate_win = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/arm_controller/controller_state', self._on_arm_state, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/gripper_controller/controller_state', self._on_grip_state, 10)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(GoalStatusArray,
                                 '/arm_controller/follow_joint_trajectory/_action/status',
                                 lambda m: self._on_status(m, 'arm'), status_qos)
        self.create_subscription(GoalStatusArray,
                                 '/gripper_controller/follow_joint_trajectory/_action/status',
                                 lambda m: self._on_status(m, 'grip'), status_qos)

        self.cm_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.create_timer(2.0, self._poll_controllers)
        self.create_timer(1.0 / rate_hz, self._publish)

    # ---- callbacks
    def _on_js(self, msg):
        self.js = msg
        self.js_wall = time.time()
        self.rate_win.append(self.js_wall)
        self.rate_win = [t for t in self.rate_win if self.js_wall - t < 1.0]

    def _on_arm_state(self, msg):
        self.arm_state = msg

    def _on_grip_state(self, msg):
        self.grip_state = msg

    def _on_status(self, msg, which):
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        gid = bytes(latest.goal_info.goal_id.uuid).hex()
        self._seen_goals[which].add(gid)
        active = any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
                     for s in msg.status_list)
        target = self.arm_goal if which == 'arm' else self.grip_goal
        target.update(status=STATUS_NAMES.get(latest.status, str(latest.status)),
                      active=active, count=len(self._seen_goals[which]))

    def on_gz_poses(self, msg):
        """Gazebo Pose_V callback (runs on a gz-transport thread)."""
        for p in msg.pose:
            if p.name in ('box_a', 'box_b'):
                o = p.orientation
                R = quat_to_R(o.x, o.y, o.z, o.w)
                self.boxes_live[p.name] = {
                    'xyz': [p.position.x, p.position.y, p.position.z],
                    'rpy_deg': rpy_deg(R),
                    'tilt_deg': box_tilt_deg(R),
                }
                self.boxes_wall = time.time()

    def _poll_controllers(self):
        if not self.cm_client.service_is_ready():
            self.controllers = []
            return
        fut = self.cm_client.call_async(ListControllers.Request())

        def done(f):
            try:
                self.controllers = [{'name': c.name, 'state': c.state,
                                     'type': c.type.split('/')[-1]}
                                    for c in f.result().controller]
                self.controllers_wall = time.time()
            except Exception:
                pass
        fut.add_done_callback(done)

    # ---- helpers
    def _lookup(self, child):
        try:
            t = self.tf_buffer.lookup_transform('base_link', child, rclpy.time.Time())
            p, r = t.transform.translation, t.transform.rotation
            return np.array([p.x, p.y, p.z]), quat_to_R(r.x, r.y, r.z, r.w)
        except Exception:
            return None, None

    def _gripper_status(self, pos, vel, ref):
        if pos is None:
            return 'unknown', 'No gripper data'
        moving = vel is not None and abs(vel) > 0.1   # fingers jitter ~0.05 rad/s while gripping
        closed_cmd = ref is not None and ref > 0.3
        if closed_cmd:
            if moving:
                return 'closing', 'Closing'
            if pos >= GRIPPER_CLOSED - 0.06:
                return 'closed', 'Closed (empty)'
            if pos > 0.05:
                return 'holding', 'Holding object'
            return 'closing', 'Closing'
        if moving:
            return 'opening', 'Opening'
        if pos < 0.05:
            return 'open', 'Open'
        return 'partly', 'Partly open'

    def _phase(self, tcp, arm_active, grip_active, grip_code, home_dist):
        wps = self.plan['waypoints']
        n = len(wps)
        nxt = self.progress + 1
        if nxt <= n and np.linalg.norm(tcp - np.array(wps[nxt - 1]['xyz'])) < WP_TOL:
            self.progress = nxt
            self.reach_times[nxt] = self.last_t
        if home_dist < 0.5 and not arm_active and self.progress == n:
            pass  # stay "complete" until the arm actually moves away
        elif home_dist < 0.5 and not arm_active:
            self.progress, self.reach_times, self.mission_start = 0, {}, None
        if arm_active and self.mission_start is None and self.progress == 0:
            self.mission_start = self.last_t
        if grip_active:
            return 'Closing gripper' if grip_code in ('closing', 'holding', 'closed') else 'Opening gripper'
        if arm_active and self.progress < n:
            return f'Moving to WP{self.progress + 1}: {wps[self.progress]["label"]}'
        if self.progress == n:
            return 'Mission complete'
        if self.progress == 0:
            return 'At home, idle' if home_dist < 0.5 else 'Idle'
        return f'At WP{self.progress}: {wps[self.progress - 1]["label"]}'

    # ---- main loop
    def _publish(self):
        now = time.time()
        js = self.js
        out = {'type': 'state', 'seq': self.seq, 'wall': now,
               'link': {'joint_states_age_s': (now - self.js_wall) if js else None,
                        'joint_states_hz': len(self.rate_win),
                        'controllers_age_s': (now - self.controllers_wall) if self.controllers_wall else None,
                        'box_pose_age_s': (now - self.boxes_wall) if self.boxes_wall else None}}
        self.seq += 1
        if js is None:
            out['type'] = 'waiting'
            out['controllers'] = self.controllers
            with self.lock:
                self.snapshot = json.dumps(out)
            return

        idx = {n: i for i, n in enumerate(js.name)}
        if not all(j in idx for j in ARM):
            return
        pick = lambda arr, i: (float(arr[i]) if len(arr) > i else None)
        q = [float(js.position[idx[j]]) for j in ARM]
        qd = [pick(js.velocity, idx[j]) or 0.0 for j in ARM]
        eff = [pick(js.effort, idx[j]) for j in ARM]
        t = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
        self.last_t = t

        ref_q = ref_qd = err_q = None
        if self.arm_state is not None and len(self.arm_state.reference.positions) == 6:
            names = list(self.arm_state.joint_names)
            order = [names.index(j) for j in ARM] if all(j in names for j in ARM) else range(6)
            ref_q = [float(self.arm_state.reference.positions[i]) for i in order]
            if len(self.arm_state.reference.velocities) == 6:
                ref_qd = [float(self.arm_state.reference.velocities[i]) for i in order]
            if len(self.arm_state.error.positions) == 6:
                err_q = [float(self.arm_state.error.positions[i]) for i in order]

        # FK with our own model
        pts, T6 = skeleton(q)
        tcp_fk = pts[-1]
        R_fk = T6[:3, :3]
        wc = T6[:3, 3] - kin.DH_PARAMS[5][2] * T6[:3, 2]

        tcp_tf, R_tf = self._lookup('tcp')
        tool0_tf, _ = self._lookup('tool0')
        tcp = tcp_tf if tcp_tf is not None else tcp_fk
        R = R_tf if R_tf is not None else R_fk

        if self.prev_tcp is not None and self.prev_t is not None and t > self.prev_t:
            inst = np.linalg.norm(tcp - self.prev_tcp) / (t - self.prev_t)
            self.tcp_speed = 0.6 * self.tcp_speed + 0.4 * inst
        if self.prev_t is None or t != self.prev_t:
            self.prev_tcp, self.prev_t = tcp.copy(), t

        J = jac.geometric_jacobian(q)
        sv = np.linalg.svd(J, compute_uv=False)
        w = float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
        v_lin = J[:3] @ np.array(qd)

        g_pos = pick(js.position, idx[GRIP]) if GRIP in idx else None
        g_vel = pick(js.velocity, idx[GRIP]) if GRIP in idx else None
        g_ref = None
        if self.grip_state is not None and len(self.grip_state.reference.positions) >= 1:
            g_ref = float(self.grip_state.reference.positions[0])
        g_code, g_text = self._gripper_status(g_pos, g_vel, g_ref)

        home_dist = float(np.degrees(np.max(np.abs(np.array(q)))))
        phase = self._phase(tcp, self.arm_goal['active'], self.grip_goal['active'], g_code,
                            home_dist)

        dists = [float(np.linalg.norm(tcp - np.array(wp['xyz'])) * 1000)
                 for wp in self.plan['waypoints']]

        out.update({
            't': t,
            'arm': {'q_deg': [float(np.degrees(v)) for v in q],
                    'qd_deg': [float(np.degrees(v)) for v in qd],
                    'effort': eff,
                    'ref_deg': [float(np.degrees(v)) for v in ref_q] if ref_q else None,
                    'ref_vel_deg': [float(np.degrees(v)) for v in ref_qd] if ref_qd else None,
                    'err_deg': [float(np.degrees(v)) for v in err_q] if err_q else None,
                    'in_limits': [bool(lo <= np.degrees(v) <= hi)
                                  for v, (lo, hi) in zip(q, kin.JOINT_LIMITS_DEG)],
                    'psi_deg': float(np.degrees(q[1] - q[2] + q[4])),
                    'q1_plus_q6_deg': float(np.degrees(q[0] + q[5]))},
            'ee': {'tcp': tcp.tolist(),
                   'source': 'tf' if tcp_tf is not None else 'fk',
                   'tcp_fk': tcp_fk.tolist(),
                   'fk_vs_tf_mm': float(np.linalg.norm(tcp_fk - tcp_tf) * 1000) if tcp_tf is not None else None,
                   'tool0': tool0_tf.tolist() if tool0_tf is not None else T6[:3, 3].tolist(),
                   'wrist_centre': wc.tolist(),
                   'wc_radius': float(np.hypot(wc[0], wc[1])),
                   'rpy_deg': rpy_deg(R),
                   'R': R.tolist(),
                   'tilt_deg': tilt_deg(R),
                   'orient_err_deg': rot_angle_deg(R, R_DOWN),
                   'speed_mm_s': float(self.tcp_speed * 1000),
                   'v_jac_mm_s': (v_lin * 1000).tolist(),
                   'skeleton': [p.tolist() for p in pts]},
            'jac': {'w': w, 'sigma': sv.tolist(), 'cond': float(sv[0] / sv[-1]) if sv[-1] > 1e-9 else None},
            'gripper': {'pos': g_pos, 'vel': g_vel, 'ref': g_ref, 'code': g_code, 'text': g_text,
                        'closure_pct': (100 * g_pos / GRIPPER_CLOSED) if g_pos is not None else None},
            'mission': {'phase': phase, 'progress': self.progress,
                        'reach_times': self.reach_times,
                        'start_t': self.mission_start,
                        'elapsed_s': (t - self.mission_start) if self.mission_start is not None else None,
                        'dist_mm': dists,
                        'arm_goal': self.arm_goal, 'gripper_goal': self.grip_goal},
            'boxes': self.boxes_live,
            'controllers': self.controllers,
        })
        with self.lock:
            self.snapshot = json.dumps(out)


# ------------------------------------------------------------------ web server
def make_handler(node, plan_json, rate_hz):
    mime = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
            '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml'}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/api/config':
                return self._send(200, plan_json, 'application/json')
            if path == '/api/state':
                with node.lock:
                    body = node.snapshot.encode()
                return self._send(200, body, 'application/json')
            name = 'index.html' if path in ('/', '') else path.lstrip('/')
            if not re.fullmatch(r'[\w.-]+', name):
                return self._send(404, b'not found', 'text/plain')
            f = os.path.join(WEB_DIR, name)
            if not os.path.isfile(f):
                return self._send(404, b'not found', 'text/plain')
            with open(f, 'rb') as fh:
                return self._send(200, fh.read(),
                                  mime.get(os.path.splitext(f)[1], 'application/octet-stream'))
    return Handler


def main():
    ap = argparse.ArgumentParser(description='GP7 live web dashboard')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--host', default='127.0.0.1',
                    help='use 0.0.0.0 to open it from another device on the network')
    ap.add_argument('--rate', type=float, default=20.0, help='UI update rate (Hz)')
    ap.add_argument('--no-box-pose', action='store_true',
                    help='do not subscribe to Gazebo for live box poses')
    args = ap.parse_args()

    plan = build_plan()
    plan_json = json.dumps(plan).encode()

    rclpy.init()
    node = DashboardNode(plan, args.rate)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    gz_node = None
    if not args.no_box_pose:
        if GzNode is None:
            print('[dashboard] gz.transport13 Python bindings not found; live box poses disabled',
                  file=sys.stderr)
        else:
            gz_node = GzNode()
            if not gz_node.subscribe(Pose_V, MODEL_POSE_TOPIC, node.on_gz_poses):
                print(f'[dashboard] could not subscribe to {MODEL_POSE_TOPIC}', file=sys.stderr)

    server = ThreadingHTTPServer((args.host, args.port), make_handler(node, plan_json, args.rate))
    server.daemon_threads = True
    shown = 'localhost' if args.host in ('127.0.0.1', '0.0.0.0') else args.host
    print(f'[dashboard] open http://{shown}:{args.port}   (Ctrl+C to stop)', flush=True)

    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        server.serve_forever()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
