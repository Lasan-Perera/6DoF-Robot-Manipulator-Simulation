import numpy as np
import jacobian as jac

origins, z_axes = jac.frame_origins_and_axes([0.3, 0.2, -0.1, 0.4, 0.15, 0.2])
print("o_6:", np.round(origins[-1], 4))
print("expected (from earlier FK verification): [0.6095, 0.1934, 0.6615]")

J = jac.geometric_jacobian([0, 0, 0, 0, 0, 0])
print(np.round(J, 4))

jac.check_jacobian_numerically([0.3, 0.2, -0.1, 0.4, 0.15, 0.2])
