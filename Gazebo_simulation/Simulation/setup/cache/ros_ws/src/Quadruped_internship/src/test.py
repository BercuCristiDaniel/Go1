import numpy as np

vy_vals = np.array([0.2, 0.5])
omega_vals = np.array([0.01, 0.162])

coeffs = np.polyfit(vy_vals, omega_vals, 1)  # degree 1 = linear
a, b = coeffs
print(f"omega_z ≈ {a:.3f} * vy + {b:.3f}")