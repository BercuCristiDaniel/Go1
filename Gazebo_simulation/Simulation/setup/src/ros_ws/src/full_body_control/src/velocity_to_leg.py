import numpy as np

class QuadrupedVelocityTransformer:
    def __init__(self, L, C):
        """
        Initialize transformer with robot geometry.
        Parameters:
        - L: Half of the robot width (from side to side)
        - C: Half of the robot length (from front to back)
        """
        self.L = L  # Half of the robot width
        self.C = C  # Half of the robot lenght

    def transform_body_to_leg_velocities(self, vx, vy, omega_z):
        """
        Converts body-frame velocity (vx, vy, omega_z) into approximate
        leg-specific velocities and their heading angles.

        Returns:
            A dictionary with leg velocity magnitudes and heading angles.
        """
        velocity_matrix = np.array([
            [1, 0, self.L / 2],  # Front left
            [1, 0, -self.L / 2],  # Front right
            [0, 1, self.C / 2],  # Rear left
            [0, 1, -self.C / 2],  # Rear right
        ])

        body_velocity = np.array([vx, vy, omega_z])
        leg_velocities = velocity_matrix @ body_velocity  # Matrix multiplication

        v_d, v_e, v_f, v_t = leg_velocities  # Extract velocity components

        v_fd = np.sqrt(v_f**2 + v_d**2)  # Front left
        v_fe = np.sqrt(v_f**2 + v_e**2)  # Front right
        v_td = np.sqrt(v_t**2 + v_d**2)  # Rear left
        v_te = np.sqrt(v_t**2 + v_e**2)  # Rear right

        theta_fd = np.arctan2(v_f, v_d)  # Front left
        theta_fe = np.arctan2(v_f, v_e)  # Front right
        theta_td = np.arctan2(v_t, v_d)  # Rear left
        theta_te = np.arctan2(v_t, v_e)  # Rear right

        return {
            "v_fd": v_fd, "v_fe": v_fe, "v_td": v_td, "v_te": v_te,
            "theta_fd": theta_fd, "theta_fe": theta_fe, "theta_td": theta_td, "theta_te": theta_te
        }

if __name__ == "__main__":
    C = 0.47
    L = 0.30

    transformer = QuadrupedVelocityTransformer(L, C)

    vx, vy, omega_z = 0.2, 0.0, 0.0

    results = transformer.transform_body_to_leg_velocities(vx, vy, omega_z)
    print("Leg Velocities and Orientations:")
    for key, value in results.items():
        print(f"{key}: {value:.4f}")