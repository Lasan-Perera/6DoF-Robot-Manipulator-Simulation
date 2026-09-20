import numpy as np

# DH parameters for the GP7 robot arm by yasakawa
DH_PARAMS = [
    (0.040,   90, 0.330),
    (0.445,    0, 0.000),
    (0.040,   90, 0.000),
    (0.000,  270, 0.440),
    (0.000,   90, 0.000),
    (0.000,    0, 0.080),
]

# Transformation matrix from Denavit-Hartenberg parameters
def dh_transform(a: float, alpha_deg: float, d: float, theta_deg: float) -> np.ndarray:
    alpha = np.deg2rad(alpha_deg)
    theta = np.deg2rad(theta_deg)
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,      1],
    ])

# we convert joint angles to match with TF in Gazebo
# DH z1, z3, z5 point opposite to the URDF axes of joints 2, 4, 6 (+y, -x, -x),
# so those angles flip sign; joint 2 also has a 90 deg offset.
# Checked against robot_state_publisher on random joint vectors (full transforms)
def _apply_joint_conventions(joint_angles_rad: list[float]) -> list[float]:
    j = [np.rad2deg(a) for a in joint_angles_rad]
    return [
        j[0],
        90 - j[1],
        j[2],
        -j[3],
        j[4],
        -j[5],
    ]

# Forward kinematics
def forward_kinematics(joint_angles_rad: list[float]) -> np.ndarray:
    theta_dh = _apply_joint_conventions(joint_angles_rad)
    T = np.eye(4)
    for (a, alpha, d), theta in zip(DH_PARAMS, theta_dh):
        T = T @ dh_transform(a, alpha, d, theta)
    return T

# Location of the wrsit center
def wrist_center(p06: np.ndarray, R06: np.ndarray, d6: float = 0.080) -> np.ndarray:
    z5_direction = R06[:, 2]
    return p06 - d6 * z5_direction


A1 = 0.040
D1 = 0.330
A2 = 0.445
A3 = 0.040
D4 = 0.440


def solve_position_joints(wc: np.ndarray) -> tuple[float, float, float]:
    xc, yc, zc = wc

    theta1 = np.arctan2(yc, xc)

    r = np.sqrt(xc**2 + yc**2)
    r_planar = r - A1
    s = zc - D1

    L = np.hypot(r_planar, s)
    l_eff = np.hypot(A3, D4)
    phi = np.arctan2(A3, D4)

    alpha = np.arctan2(s, r_planar)
    beta = np.arccos(np.clip((A2**2 + L**2 - l_eff**2) / (2 * A2 * L), -1, 1))
    gamma = np.arccos(np.clip((A2**2 + l_eff**2 - L**2) / (2 * A2 * l_eff), -1, 1))

    theta2 = np.pi/2 - (alpha + beta)
    theta3 = (gamma - phi) - np.pi/2

    return theta1, theta2, theta3


def _rotation_0_to_3(theta1: float, theta2: float, theta3: float) -> np.ndarray:
    theta_dh = _apply_joint_conventions([theta1, theta2, theta3, 0, 0, 0])
    T = np.eye(4)
    for (a, alpha, d), theta in zip(DH_PARAMS[:3], theta_dh[:3]):
        T = T @ dh_transform(a, alpha, d, theta)
    return T[:3, :3]


def solve_wrist_orientation(theta1, theta2, theta3, R06: np.ndarray) -> tuple[float, float, float]:
    R03 = _rotation_0_to_3(theta1, theta2, theta3)
    R36 = R03.T @ R06

    theta5_dh = np.arccos(np.clip(R36[2, 2], -1, 1))
    theta4_dh = np.arctan2(R36[1, 2], R36[0, 2])
    theta6_dh = np.arctan2(R36[2, 1], -R36[2, 0])

    theta4 = -theta4_dh
    theta5 = theta5_dh
    theta6 = -theta6_dh

    return theta4, theta5, theta6


def inverse_kinematics(target_pose: np.ndarray) -> list[list[float]]:
    p06 = target_pose[:3, 3]
    R06 = target_pose[:3, :3]

    wc = wrist_center(p06, R06)
    theta1, theta2, theta3 = solve_position_joints(wc)
    theta4, theta5, theta6 = solve_wrist_orientation(theta1, theta2, theta3, R06)

    return [[theta1, theta2, theta3, theta4, theta5, theta6]]

L_TCP = 0.112
D6 = 0.080

R_FLANGE_TO_TOOL0 = np.array([
    [0.0,  0.0, 1.0],
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
])


def inverse_kinematics_tcp(target_pose_tcp: np.ndarray) -> list[list[float]]:
    p_tcp = target_pose_tcp[:3, 3]
    R_tcp = target_pose_tcp[:3, :3]

    R06 = R_tcp
    wc = p_tcp - (L_TCP + D6) * R06[:, 2]

    theta1, theta2, theta3 = solve_position_joints(wc)
    theta4, theta5, theta6 = solve_wrist_orientation(theta1, theta2, theta3, R06)

    return [[theta1, theta2, theta3, theta4, theta5, theta6]]
