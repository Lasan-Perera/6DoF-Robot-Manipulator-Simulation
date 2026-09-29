"""tau_dyn: max across the six joints of the per-joint RMS
gravity-compensated torque residual over all valid logged samples in a run.

    residual_j(t) = measured_effort_j(t) - gravity_torque_full(q(t), ...)_j
    tau_dyn = max_j ( sqrt( mean_t( residual_j(t)^2 ) ) )

"Valid" samples exclude any where the gripper's own effort is logged as NaN
(the arm joints only; the gripper's own 7th effort entry is never part of
this calculation) and, if given, samples outside [t_start, t_end] -- e.g. to
exclude the unlogged pre-positioning move, which run_and_log_*.py already
keeps out of the log entirely, but kept here as a defensive option.
"""
import numpy as np
import gravity_model_full as gmf


def compute_tau_dyn(log_times, log_positions, log_efforts, gripper_theta, carrying_box):
    """log_times, log_positions (N,6), log_efforts (N,6): from a run's
    /joint_states log (arm joints only). gripper_theta/carrying_box: the
    gripper state that applied throughout THIS run (fixed for one run, not
    per-sample, matching how run_and_log_via_point.py already operates --
    gripper closes/opens outside the logged window)."""
    log_positions = np.asarray(log_positions, dtype=float)
    log_efforts = np.asarray(log_efforts, dtype=float)
    n = log_positions.shape[0]

    predicted = np.zeros((n, 6))
    for i in range(n):
        predicted[i] = gmf.gravity_torque_full(log_positions[i], gripper_theta, carrying_box)

    residual = log_efforts - predicted
    valid = ~np.any(np.isnan(residual), axis=1)
    residual = residual[valid]
    if residual.shape[0] == 0:
        raise ValueError('no valid (non-NaN) samples to compute tau_dyn from')

    rms_per_joint = np.sqrt(np.mean(residual ** 2, axis=0))
    return float(np.max(rms_per_joint)), rms_per_joint
