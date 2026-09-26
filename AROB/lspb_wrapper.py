"""
Thin wrapper importing the already-verified LSPB trajectory generator
from the main gp7_pick_place package, so Experiment 1/2 comparisons in
AROB/ use the exact same LSPB code as the rest of the project rather
than a duplicate.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                 "src", "gp7_pick_place", "gp7_pick_place"))
from trajectory import segment_timing, lspb_position, generate_segment_trajectory

# re-exported for the comparison scripts
__all__ = ["segment_timing", "lspb_position", "generate_segment_trajectory"]
