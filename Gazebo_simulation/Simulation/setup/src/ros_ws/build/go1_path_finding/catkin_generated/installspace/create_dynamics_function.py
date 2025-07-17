import casadi as ca

def create_dynamics_function():
    x_b = ca.MX.sym('x_b')
    y_b = ca.MX.sym('y_b')
    psi = ca.MX.sym('psi')
    phi = ca.MX.sym('phi')
    phi_dot = ca.MX.sym('phi_dot')

    v = ca.MX.sym('v')
    omega = ca.MX.sym('omega')
    u_phi = ca.MX.sym('u_phi')

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
