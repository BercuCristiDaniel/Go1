import casadi as ca

def create_go1_dynamics_function():
    """Creates the full Single Rigid Body Dynamics (SRBD) function for the quadruped with 12 control inputs."""

    M = 12 
    g = ca.MX([0, 0, -9.81])  # Gravity vector

    I_values = ca.MX([0.206, 0.480, 0.397])  
    I = ca.diag(I_values)

    #   Define State Variables
    p = ca.MX.sym('p', 3)  # Position (x, y, z)
    v = ca.MX.sym('v', 3)  # Velocity (vx, vy, vz)
    R_flat = ca.MX.sym('R', 9)  # Rotation matrix (flattened 3x3)
    omega = ca.MX.sym('omega', 3)  # Angular velocity (wx, wy, wz)

    #   Reshape Rotation Matrix
    R = ca.reshape(R_flat, 3, 3)

    #   Define Control Inputs (Ground Reaction Forces - GRFs)
    F1 = ca.MX.sym('F1', 3)  # GRF for front-left foot
    F2 = ca.MX.sym('F2', 3)  # GRF for front-right foot
    F3 = ca.MX.sym('F3', 3)  # GRF for rear-left foot
    F4 = ca.MX.sym('F4', 3)  # GRF for rear-right foot

    #   Define Foot Positions Relative to CoM
    r1 = ca.MX.sym('r1', 3)  # FL foot relative position
    r2 = ca.MX.sym('r2', 3)  # FR foot relative position
    r3 = ca.MX.sym('r3', 3)  # RL foot relative position
    r4 = ca.MX.sym('r4', 3)  # RR foot relative position

    #   Compute Total Forces
    F_total = F1 + F2 + F3 + F4

    #   Compute Torques (Moment Contributions)
    tau_total = (
        skew_symmetric(R @ r1) @ F1 +
        skew_symmetric(R @ r2) @ F2 +
        skew_symmetric(R @ r3) @ F3 +
        skew_symmetric(R @ r4) @ F4
    )

    #   Compute State Derivatives
    p_dot = v  # Linear position derivative
    v_dot = (F_total / M) + g  # Newton's Second Law (Force Balance)

    #   Compute Rotation Matrix Derivative (Lie Algebra Mapping)
    R_dot = R @ skew_symmetric(omega)

    #   Compute Angular Acceleration
    omega_dot = ca.mtimes(ca.inv(I), (tau_total - ca.cross(omega, ca.mtimes(I, omega))))

    #   Reshape R_dot for Concatenation
    R_dot_flat = ca.reshape(R_dot, 9, 1)

    #   Construct System Dynamics
    x = ca.vertcat(p, v, R_flat, omega)  # 18 states
    u = ca.vertcat(F1, F2, F3, F4)  # 12 control inputs (3 per foot)
    r = ca.vertcat(r1, r2, r3, r4)  # Foot relative positions as inputs

    #   Compute Time Derivative of State
    x_dot = ca.vertcat(p_dot, v_dot, R_dot_flat, omega_dot)

    #   Create CasADi Function
    f = ca.Function('f', [x, u, r], [x_dot])

    return f

def skew_symmetric(v):
    """Creates a skew-symmetric matrix from a 3D vector."""
    return ca.vertcat(
        ca.horzcat(0, -v[2], v[1]),
        ca.horzcat(v[2], 0, -v[0]),
        ca.horzcat(-v[1], v[0], 0)
    )
