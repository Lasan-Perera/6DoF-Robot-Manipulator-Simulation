import numpy as np
import jacobian as jac


def manipulability(joint_angles_rad: list[float]) -> float:
    J = jac.geometric_jacobian(joint_angles_rad)
    det_val = np.linalg.det(J @ J.T)
    return np.sqrt(max(det_val, 0.0))

def singular_values(joint_angles_rad: list[float]) -> np.ndarray:
    return np.linalg.svd(jac.geometric_jacobian(joint_angles_rad), compute_uv=False)


def jacobian_rank(joint_angles_rad: list[float], tol: float = 1e-6) -> int:
    return np.linalg.matrix_rank(jac.geometric_jacobian(joint_angles_rad), tol=tol)