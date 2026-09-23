import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))
import pixel_to_world as p2w

tests = [
    ("red",   0.5869140625, 0.5126953125, 0.05, (0.5, -0.15)),
    ("green", 0.4765625,    0.4140625,    0.05, (0.65, 0.05)),
    ("blue",  0.4345703125, 0.5771484375, 0.05, (0.4, 0.15)),
    ("plat",  0.2490234375, 0.4833984375, 0.20, (0.55, 0.5)),
]

for name, u, v, top_z, actual in tests:
    wx, wy = p2w.pixel_to_world(u, v, top_z)
    print(f"{name}: predicted=({wx:.4f},{wy:.4f})  actual={actual}")
