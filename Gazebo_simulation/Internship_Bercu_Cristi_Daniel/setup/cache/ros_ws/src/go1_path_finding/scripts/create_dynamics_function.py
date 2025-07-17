import casadi as ca


def create_dynamics_function():
    # State Variables
    x_b = ca.MX.sym('x_b')    # x-position
    y_b = ca.MX.sym('y_b')    # y-position
    psi = ca.MX.sym('psi')    # yaw orientation
    phi = ca.MX.sym('phi')    # roll angle
    phi_dot = ca.MX.sym('phi_dot')  # roll angle velocity

    # Control Inputs
    v = ca.MX.sym('v')        # linear velocity
    omega = ca.MX.sym('omega')  # angular velocity
    u_phi = ca.MX.sym('u_phi')  # roll control input

    # Parameters
    g = 9.81  # gravitational acceleration (m/s^2)
    h = 0.31  # height of the center of mass (m)
    c_phi = 0.1  # roll damping coefficient (adjust as needed)

    # State Vector and Control Vector
    x = ca.vertcat(x_b, y_b, psi, phi, phi_dot)
    u = ca.vertcat(v, omega, u_phi)

    # Kinematic Equations
    x_b_dot = v * ca.cos(psi)
    y_b_dot = v * ca.sin(psi)
    psi_dot = omega

    # Roll Dynamics
    phi_ddot = -g / h * ca.sin(phi) + v * omega / h - c_phi * phi_dot + u_phi

    # State Derivatives
    xdot = ca.vertcat(x_b_dot, y_b_dot, psi_dot, phi_dot, phi_ddot)

    # CasADi Function
    f = ca.Function('f', [x, u], [xdot])
    return f