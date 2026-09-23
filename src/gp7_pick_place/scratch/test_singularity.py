import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gp7_pick_place"))

import numpy as np
import singularity as sing

q = [0.3, 0.2, -0.1, 0.4, 0.15, 0.2]
print("manipulability:", sing.manipulability(q))
print("singular values:", np.round(sing.singular_values(q), 4))
print("rank:", sing.jacobian_rank(q))

print("\n--- at theta5 = 0 (expected wrist singularity) ---")
q_sing = [0.3, 0.2, -0.1, 0.4, 0.0, 0.2]
print("manipulability:", sing.manipulability(q_sing))
print("singular values:", np.round(sing.singular_values(q_sing), 4))
print("rank:", sing.jacobian_rank(q_sing))
