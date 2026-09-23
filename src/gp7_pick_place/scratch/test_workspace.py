import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))
import numpy as np
import workspace as ws

points = ws.sample_workspace(n_samples=20000)
radii = np.linalg.norm(points, axis=1)
print("max radius sampled:", np.round(radii.max(), 4), "m")
print("expected (a1+a2+l_eff):", 0.9268, "m")
print("min z:", np.round(points[:,2].min(), 4))
print("max z:", np.round(points[:,2].max(), 4))

bound = ws.max_reach_upper_bound()
print("\ntriangle-inequality upper bound:", np.round(bound, 4))
print("sampled max <= bound:", radii.max() <= bound)
print("\npoints with TCP below ground (z<0):", (points[:,2] < 0).sum(), "/", len(points))
