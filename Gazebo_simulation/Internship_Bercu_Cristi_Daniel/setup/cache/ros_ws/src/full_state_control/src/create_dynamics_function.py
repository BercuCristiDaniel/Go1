import casadi as ca


def create_full_dynamics_function():
    x = ca.MX.sym('x')
    y = ca.MX.sym('y')
    theta = ca.MX.sym('theta')

    # Control Inputs
    vx = ca.MX.sym('v_x')
    vy = ca.MX.sym('v_y')
    omega = ca.MX.sym('omega')

    # State and input vectors
    state = ca.vertcat(x, y, theta)
    control = ca.vertcat(vx, vy, omega)

    # Dynamics equations
    x_dot = vx * ca.cos(theta) - vy * ca.sin(theta)
    y_dot = vx * ca.sin(theta) + vy * ca.cos(theta)
    theta_dot = omega

    # Time derivatives
    xdot = ca.vertcat(x_dot, y_dot, theta_dot)

    # CasADi Function
    f = ca.Function('f_simple', [state, control], [xdot])

    return f