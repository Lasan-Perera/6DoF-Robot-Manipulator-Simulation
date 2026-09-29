#!/bin/bash
# Experiment 2 orchestration: restarts Gazebo fresh before EVERY single
# (method, repeat) run, since the box ends up dropped at the platform and
# each run needs it back at its original resting position.
#
# Usage: ./run_experiment2.sh [n_repeats]   (default 30)
#
# Run from AROB_10methods/, with the workspace already built
# (colcon build in ~/6DoF-Robot-Manipulator-Simulation).
#
# A single run's failure (e.g. "never received /joint_states", a transient
# startup-timing issue) is retried once with a fresh Gazebo restart, then
# skipped with a clear warning if it fails again -- it does NOT kill the
# whole batch. Found 2026-09-29: a bare (unwrapped) call to the python
# driver under `set -e` DOES kill the whole script the instant any run
# crashes, silently losing every remaining method/repeat. Confirmed via a
# standalone if-vs-bare-statement test under set -e before applying this fix.

set -e
N_REPEATS=${1:-30}
WS=~/6DoF-Robot-Manipulator-Simulation
FAILED_LOG="failed_runs.txt"

# Resume support: set METHODS_OVERRIDE (space-separated) to run only some
# methods, and START_REP to make the FIRST method in that list begin at a
# given repeat instead of 1 (every method after the first still starts at
# 1). Example, resuming after lspb/cubic/quintic/septic finished and
# cycloidal got through repeat 9:
#   METHODS_OVERRIDE="cycloidal modsine modtrap doubles bspline lspbblend" \
#   START_REP=10 ./run_experiment2.sh 30
if [ -n "$METHODS_OVERRIDE" ]; then
  read -ra METHODS <<< "$METHODS_OVERRIDE"
else
  METHODS=(lspb cubic quintic septic cycloidal modsine modtrap doubles bspline lspbblend)
fi
START_REP=${START_REP:-1}

launch_and_wait() {
  setsid bash -c "cd $WS && source install/setup.bash && ros2 launch gp7_description gazebo.launch.py" \
    > /tmp/gazebo_last_run.log 2>&1 &
  GAZEBO_PID=$!
  echo "waiting for /arm_controller/follow_joint_trajectory..."
  for i in $(seq 1 60); do
    if ros2 action list 2>/dev/null | grep -q "/arm_controller/follow_joint_trajectory"; then
      break
    fi
    sleep 1
  done
}

kill_gazebo() {
  kill -- -$(ps -o pgid= "$GAZEBO_PID" | tr -d ' ') 2>/dev/null || true
  wait "$GAZEBO_PID" 2>/dev/null || true
  sleep 2   # let ports/processes fully release before the next launch
}

FIRST_METHOD=1
for method in "${METHODS[@]}"; do
  if [ "$FIRST_METHOD" -eq 1 ]; then
    REP_START=$START_REP
    FIRST_METHOD=0
  else
    REP_START=1
  fi
  for rep in $(seq "$REP_START" "$N_REPEATS"); do
    echo ""
    echo "=== $method, repeat $rep/$N_REPEATS ==="

    launch_and_wait
    if python3 run_one_experiment2.py "$method"; then
      RUN_STATUS=0
    else
      RUN_STATUS=$?
    fi
    kill_gazebo

    if [ "$RUN_STATUS" -ne 0 ]; then
      echo "WARNING: $method repeat $rep failed (status $RUN_STATUS) -- retrying once with a fresh restart"
      launch_and_wait
      if python3 run_one_experiment2.py "$method"; then
        RETRY_STATUS=0
      else
        RETRY_STATUS=$?
      fi
      kill_gazebo

      if [ "$RETRY_STATUS" -ne 0 ]; then
        echo "WARNING: $method repeat $rep failed AGAIN (status $RETRY_STATUS) -- skipping this repeat, batch continues"
        echo "$method,$rep,$RETRY_STATUS" >> "$FAILED_LOG"
      else
        echo "retry succeeded"
      fi
    fi
  done
done

echo ""
echo "done -- results_table5.csv"
if [ -f "$FAILED_LOG" ]; then
  echo "some repeats failed twice and were skipped -- see $FAILED_LOG"
fi
