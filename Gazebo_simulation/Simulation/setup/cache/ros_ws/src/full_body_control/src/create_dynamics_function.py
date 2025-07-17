import casadi as ca


def create_full_dynamics_function():
    # State Variables
    x_b = ca.MX.sym('x_b')
    y_b = ca.MX.sym('y_b')
    z_b = ca.MX.sym('z_b')
    phi = ca.MX.sym('phi')
    theta = ca.MX.sym('theta')
    psi = ca.MX.sym('psi')

    # Control Inputs (Body-frame linear & angular velocities)
    v_x = ca.MX.sym('v_x')
    v_y = ca.MX.sym('v_y')
    v_z = ca.MX.sym('v_z')
    omega_phi = ca.MX.sym('omega_phi')
    omega_theta = ca.MX.sym('omega_theta')
    omega_psi = ca.MX.sym('omega_psi')

    x = ca.vertcat(x_b, y_b, z_b, phi, theta, psi)
    u = ca.vertcat(v_x, v_y, v_z, omega_phi, omega_theta, omega_psi)

    # Rotation Matrix (Body to World)
    cos_psi, sin_psi = ca.cos(psi), ca.sin(psi)
    cos_theta, sin_theta = ca.cos(theta), ca.sin(theta)
    cos_phi, sin_phi = ca.cos(phi), ca.sin(phi)

    R = ca.vertcat(
        ca.horzcat(cos_psi * cos_theta, cos_psi * sin_theta * sin_phi - sin_psi * cos_phi, cos_psi * sin_theta * cos_phi + sin_psi * sin_phi),
        ca.horzcat(sin_psi * cos_theta, sin_psi * sin_theta * sin_phi + cos_psi * cos_phi, sin_psi * sin_theta * cos_phi - cos_psi * sin_phi),
        ca.horzcat(-sin_theta, cos_theta * sin_phi, cos_theta * cos_phi)
    )

    # Translational velocities in world frame
    v_world = R @ ca.vertcat(v_x, v_y, v_z)

    # Euler angle rates transformation matrix
    T = ca.vertcat(
        ca.horzcat(1, sin_phi * ca.tan(theta), cos_phi * ca.tan(theta)),
        ca.horzcat(0, cos_phi, sin_phi),
        ca.horzcat(0, -sin_phi / cos_theta, cos_phi / cos_theta)
    )

    omega_body = ca.vertcat(omega_phi, omega_theta, omega_psi)
    euler_dot = T @ omega_body

    # Time derivatives
    xdot = ca.vertcat(v_world, euler_dot)

    # CasADi Function
    f = ca.Function('f', [x, u], [xdot])

    return f
