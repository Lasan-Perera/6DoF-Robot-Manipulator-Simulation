#!/usr/bin/env bash
# One command for the whole GP7 demo:
#   Gazebo + controllers  ->  live web dashboard (opens in the browser)  ->  pick-and-place.
# After a run, press Enter to reset the scene (arm home, gripper open, Box B back) and run again.
#
#   ./demo.sh               normal: Gazebo window, browser, waits for Enter before each run
#   ./demo.sh --auto        start the pick-and-place as soon as everything is ready
#   ./demo.sh --headless    no Gazebo window (dashboard only)
#   ./demo.sh --no-dashboard    Gazebo + mission only
#   ./demo.sh --no-browser      start the dashboard but don't open the browser
#   ./demo.sh --build       run colcon build first
# Ctrl+C stops everything this script started.  Logs: log/demo/
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$REPO/src/gp7_pick_place/gp7_pick_place"
WORLD="$REPO/src/gp7_description/worlds/pick_place.sdf"
LOGS="$REPO/log/demo"; mkdir -p "$LOGS"
PY=/usr/bin/python3          # system Python: conda's python3 cannot load rclpy
PORT=8080
AUTO=0; HEADLESS=0; DASH=1; BUILD=0; BROWSER=1
for a in "$@"; do
  case "$a" in
    --auto) AUTO=1 ;; --headless) HEADLESS=1 ;; --no-dashboard) DASH=0 ;; --build) BUILD=1 ;; --no-browser) BROWSER=0 ;;
    --port=*) PORT="${a#*=}" ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "unknown option: $a (see --help)"; exit 1 ;;
  esac
done

# ROS environment (works even when conda's base env is active)
set +u; source /opt/ros/jazzy/setup.bash; source "$REPO/install/setup.bash"; set -u
say() { printf '\033[1;36m[demo]\033[0m %s\n' "$*"; }

PIDS=()
cleanup() {
  trap - INT TERM EXIT
  say "stopping everything..."
  for p in "${PIDS[@]}"; do kill -TERM -- "-$p" 2>/dev/null; done
  sleep 2
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  say "done."
}
trap cleanup INT TERM EXIT

if pgrep -f "gz sim" >/dev/null; then
  say "Gazebo is already running (often a leftover server after its window was closed):"
  pgrep -af "gz sim" | sed 's/^/        /'
  ans=n
  [ -t 0 ] && read -r -p "$(printf '\033[1;36m[demo]\033[0m') Stop it and continue? [y/N] " ans
  if [ "$ans" = y ] || [ "$ans" = Y ]; then
    pkill -f "gz sim"; sleep 2; pkill -9 -f "gz sim" 2>/dev/null; sleep 1
  else
    say "Not started. Close it first (or: pkill -f 'gz sim'), then run ./demo.sh again."
    trap - EXIT; exit 1
  fi
fi

if [ "$BUILD" = 1 ]; then
  say "colcon build..."
  (cd "$REPO" && colcon build --symlink-install) || exit 1
  set +u; source "$REPO/install/setup.bash"; set -u
fi

# ---- 1. Gazebo + robot + controllers
LAUNCH="$(ros2 pkg prefix gp7_description)/share/gp7_description/launch/gazebo.launch.py"
if [ "$HEADLESS" = 1 ]; then
  sed "s|'gz_args': f'-r |'gz_args': f'-s -r --headless-rendering |" "$LAUNCH" > "$LOGS/headless.launch.py"
  LAUNCH="$LOGS/headless.launch.py"
fi
say "starting Gazebo (log: log/demo/gazebo.log)..."
setsid ros2 launch "$LAUNCH" > "$LOGS/gazebo.log" 2>&1 &
PIDS+=($!)

say "waiting for the 3 controllers to become active..."
for i in $(seq 1 90); do
  n=$(ros2 control list_controllers 2>/dev/null | grep -c " active")
  [ "$n" -ge 3 ] && break
  sleep 1
done
if [ "${n:-0}" -lt 3 ]; then say "controllers did not come up; see log/demo/gazebo.log"; exit 1; fi
sleep 3   # the launch file opens the gripper right after its controller starts
say "simulation ready."

# ---- 2. dashboard
if [ "$DASH" = 1 ]; then
  (cd "$PKG" && exec setsid "$PY" dashboard.py --port "$PORT" > "$LOGS/dashboard.log" 2>&1) &
  PIDS+=($!)
  for i in $(seq 1 30); do curl -s -o /dev/null "http://127.0.0.1:$PORT/" && break; sleep 0.5; done
  say "dashboard: http://localhost:$PORT"
  [ "$BROWSER" = 1 ] && xdg-open "http://localhost:$PORT" >/dev/null 2>&1 &
fi

# ---- helpers for re-runs
box_b_spawn() {   # x y z of Box B from the world file
  sed -n '/<model name="box_b">/,/<\/pose>/p' "$WORLD" | grep -o '<pose>[^<]*' | sed 's/<pose>//' | awk '{print $1, $2, $3}'
}
reset_scene() {
  say "resetting: arm home, gripper open, Box B back to its spawn pose..."
  ros2 topic pub --once -w 1 /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [robotiq_85_left_knuckle_joint], points: [{positions: [0.0], time_from_start: {sec: 1}}]}" >/dev/null 2>&1
  sleep 1.5
  # joint-space move back to home (all joints 0) over 7 s
  ros2 topic pub --once -w 1 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 7}}]}" >/dev/null 2>&1
  sleep 7.5
  read -r bx by bz <<< "$(box_b_spawn)"
  gz service -s /world/pick_place_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
    --req "name: \"box_b\", position: {x: $bx, y: $by, z: $bz}, orientation: {w: 1}" >/dev/null 2>&1 \
    && say "Box B reset to ($bx, $by, $bz)" || say "could not reset Box B (gz service failed)"
  sleep 1
}

# ---- 3. pick-and-place, as many times as you like
run=0
while true; do
  if [ "$AUTO" = 1 ] && [ "$run" = 0 ]; then :; else
    if [ "$run" = 0 ]; then
      read -r -p "$(printf '\033[1;36m[demo]\033[0m') Press Enter to run the pick-and-place (Ctrl+C to quit) " _ || break
    else
      read -r -p "$(printf '\033[1;36m[demo]\033[0m') Press Enter to reset and run again (Ctrl+C to quit) " _ || break
      reset_scene
    fi
  fi
  run=$((run + 1))
  say "run $run: pick-and-place (log: log/demo/mission_$run.log)"
  (cd "$PKG" && "$PY" run_pick_place.py 2>&1 | tee "$LOGS/mission_$run.log" | grep --line-buffered -E "finished|complete|REJECTED|FAILED")
  say "run $run finished."
  if [ "$AUTO" = 1 ] && [ ! -t 0 ]; then break; fi
done
