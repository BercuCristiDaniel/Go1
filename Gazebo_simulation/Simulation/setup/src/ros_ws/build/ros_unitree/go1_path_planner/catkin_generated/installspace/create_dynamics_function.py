import casadi as ca

def create_dynamics_function():
    x_b = ca.MX.sym('x_b')    # x-position
    y_b = ca.MX.sym('y_b')    # y-position
    psi = ca.MX.sym('psi')    # yaw orientation
    phi = ca.MX.sym('phi')    # roll angle
    phi_dot = ca.MX.sym('phi_dot')  # roll angle velocity

    v = ca.MX.sym('v')        # linear velocity
    omega = ca.MX.sym('omega')  # angular velocity
    u_phi = ca.MX.sym('u_phi')  # roll control input

    g = 9.81
    h = 0.31
    c_phi = 0.1

    x = ca.vertcat(x_b, y_b, psi, phi, phi_dot)
    u = ca.vertcat(v, omega, u_phi)

    x_b_dot = v * ca.cos(psi)
    y_b_dot = v * ca.sin(psi)
    psi_dot = omega
    phi_ddot = -g / h * ca.sin(phi) + v * omega / h - c_phi * phi_dot + u_phi

    xdot = ca.vertcat(x_b_dot, y_b_dot, psi_dot, phi_dot, phi_ddot)
    return ca.Function('f', [x, u], [xdot])
