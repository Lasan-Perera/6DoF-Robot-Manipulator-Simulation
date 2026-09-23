import numpy as np

CAM_X, CAM_Y, CAM_Z = 0.52, 0.02, 1.8
IMG_W, IMG_H = 1280, 960
HFOV = 1.047  # rad


def pixel_to_world(u_norm: float, v_norm: float, object_top_z: float) -> tuple[float, float]:
    """
    u_norm, v_norm: normalized (0-1) pixel coordinates, Moondream .point()
    convention (origin top-left, x right, y down).
    object_top_z: known world z of the object's visible top surface.
    Returns (world_x, world_y).
    """
    u = u_norm * IMG_W
    v = v_norm * IMG_H
    f = (IMG_W / 2) / np.tan(HFOV / 2)
    H = CAM_Z - object_top_z

    dx_img = (u - IMG_W / 2) / f * H
    dy_img = (v - IMG_H / 2) / f * H

    world_x = CAM_X - dy_img
    world_y = CAM_Y - dx_img
    return world_x, world_y
