"""
Workspace/reachability analysis via random joint-space sampling.
"""
import numpy as np
import kinematics as kin

JOINT_LIMITS_RAD = [
    (np.radians(-170), np.radians(170)),
    (np.radians(-65),  np.radians(145)),
    (np.radians(-70),  np.radians(190)),
    (np.radians(-190), np.radians(190)),
    (np.radians(-135), np.radians(135)),
    (np.radians(-360), np.radians(360)),
]

def sample_workspace(n_samples=20000, seed=42):
    rng = np.random.default_rng(seed)
    points = np.zeros((n_samples, 3))
    for i in range(n_samples):
        q = [rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS_RAD]
        T = kin.forward_kinematics(q)
        points[i] = T[:3, 3]
    return points


def max_reach_upper_bound():
    """Triangle-inequality bound: no configuration can exceed this."""
    A1, A2, A3, D1, D4, D6, L_TCP = kin.A1, kin.A2, kin.A3, kin.D1, kin.D4, kin.D6, kin.L_TCP
    l_eff = np.hypot(A3, D4)
    return A1 + A2 + l_eff + D1 + (D6 + L_TCP)

def plot_workspace(points, save_path='workspace_plot.png'):
    """3D scatter of the reachable point cloud, plus a top-down and
    side view for clearer 2D reading in the report."""
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(15, 5))

    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, alpha=0.15, c='steelblue')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
    ax1.set_title('3D reachable workspace')

    ax2 = fig.add_subplot(132)
    ax2.scatter(points[:, 0], points[:, 1], s=1, alpha=0.15, c='steelblue')
    ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
    ax2.set_title('Top-down (XY)')
    ax2.axhline(0, color='gray', lw=0.5); ax2.axvline(0, color='gray', lw=0.5)
    ax2.set_aspect('equal')

    ax3 = fig.add_subplot(133)
    r = np.hypot(points[:, 0], points[:, 1])
    ax3.scatter(r, points[:, 2], s=1, alpha=0.15, c='steelblue')
    ax3.axhline(0, color='red', lw=1, linestyle='--', label='ground (z=0)')
    ax3.set_xlabel('radial distance from base axis (m)'); ax3.set_ylabel('Z (m)')
    ax3.set_title('Side view (R-Z)')
    ax3.legend()

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f'saved to {save_path}')
