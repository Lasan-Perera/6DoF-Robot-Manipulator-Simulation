import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))
import workspace as ws

points = ws.sample_workspace(n_samples=20000)
ws.plot_workspace(points, save_path='workspace_plot.png')
