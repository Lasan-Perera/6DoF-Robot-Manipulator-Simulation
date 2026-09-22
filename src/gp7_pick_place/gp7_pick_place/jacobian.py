import numpy as np
import kinematics as kin


def all_frame_transforms(joint_angles_rad: list[float]) -> list[np.ndarray]:
    theta_dh = kin._apply_joint_conventions(joint_angles_rad)

    transforms = [np.eye(4)]  # T_0 = base frame, identity

    T = np.eye(4)
    for (a, alpha, d), theta in zip(kin.DH_PARAMS, theta_dh):
        A_i = kin.dh_transform(a, alpha, d, theta)
        T = T @ A_i
        transforms.append(T.copy())

    return transforms


def frame_origins_and_axes(joint_angles_rad: list[float]):
    transforms = all_frame_transforms(joint_angles_rad)
    origins = [T[:3, 3] for T in transforms]
    z_axes = [T[:3, 2] for T in transforms]

    return origins, z_axes

def geometric_jacobian(joint_angles_rad: list[float]) -> np.ndarray:
    origins, z_axes = frame_origins_and_axes(joint_angles_rad)
    o_n = origins[6]

    J = np.zeros((6, 6))
    for i in range(6):
        z = z_axes[i]
        o = origins[i]
        J[0:3, i] = np.cross(z, o_n - o)
        J[3:6, i] = z

    # convert from D-H joint-velocity convention to physical joint velocity
    # (matches kinematics._apply_joint_conventions -- only sign matters
    # here since a derivative kills the +90 offset on joint 2)
    physical_sign = np.array([1, -1, 1, -1, 1, -1])
    J = J * physical_sign  # scales each column

    return J

def check_jacobian_numerically(joint_angles_rad, dq=1e-6):
    J = geometric_jacobian(joint_angles_rad)

    T0 = kin.forward_kinematics(joint_angles_rad)
    p0 = T0[:3, 3]

    print(f"{'joint':>6}{'Jv (predicted)':>30}{'Jv (numerical)':>30}")
    for i in range(6):
        perturbed = list(joint_angles_rad)
        perturbed[i] += dq

        T1 = kin.forward_kinematics(perturbed)
        p1 = T1[:3, 3]

        numerical_v = (p1 - p0) / dq
        predicted_v = J[0:3, i]

        print(f"{i+1:>6}{np.round(predicted_v, 4)!s:>30}{np.round(numerical_v, 4)!s:>30}")
