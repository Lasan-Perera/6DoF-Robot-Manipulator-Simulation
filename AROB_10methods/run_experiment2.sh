#!/bin/bash
# Experiment 2 orchestration: restarts Gazebo fresh before EVERY single
# (method, repeat) run, since the box ends up dropped at the platform and
# each run needs it back at its original resting position.
#
# Usage: ./run_experiment2.sh [n_repeats]   (default 30)
#
# Run from AROB_10methods/, with the workspace already built
# (colcon build in ~/6DoF-Robot-Manipulator-Simulation).

set -e
N_REPEATS=${1:-30}
METHODS=(lspb cubic quintic septic cycloidal modsine modtrap doubles bspline lspbblend)
WS=~/6DoF-Robot-Manipulator-Simulation

for method in "${METHODS[@]}"; do
  for rep in $(seq 1 "$N_REPEATS"); do
    echo ""
    echo "=== $method, repeat $rep/$N_REPEATS ==="

    # launch Gazebo in the background, headless-friendly (no --gui needed
    # for data collection; drop the redirect if you want to watch it)
    bash -c "cd $WS && source install/setup.bash && ros2 launch gp7_description gazebo.launch.py" \
      > /tmp/gazebo_last_run.log 2>&1 &
    GAZEBO_PID=$!

    # wait for the arm controller's action server to actually be up,
    # rather than a fixed sleep -- more robust to variable startup time
    echo "waiting for /arm_controller/follow_joint_trajectory..."
    for i in $(seq 1 60); do
      if ros2 action list 2>/dev/null | grep -q "/arm_controller/follow_joint_trajectory"; then
        break
      fi
      sleep 1
    done

    python3 run_one_experiment2.py "$method"
    RUN_STATUS=$?

    # kill this run's Gazebo (and everything under it) before the next iteration
    kill -- -$(ps -o pgid= "$GAZEBO_PID" | tr -d ' ') 2>/dev/null || true
    wait "$GAZEBO_PID" 2>/dev/null || true
    sleep 2   # let ports/processes fully release before the next launch

    if [ "$RUN_STATUS" -ne 0 ]; then
      echo "WARNING: $method repeat $rep exited with status $RUN_STATUS -- check /tmp/gazebo_last_run.log"
    fi
  done
done

echo ""
echo "done -- results_table5.csv"
