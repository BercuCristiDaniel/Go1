import casadi as ca

def create_dynamics_function():
    # State Variables
    x_b = ca.MX.sym('x_b')    # x-position
    y_b = ca.MX.sym('y_b')    # y-position
    psi = ca.MX.sym('psi')    # yaw orientation

    # Control Inputs
    v = ca.MX.sym('v')        # linear velocity
    omega = ca.MX.sym('omega')  # yaw rate



    # State Vector and Control Vector
    x = ca.vertcat(x_b, y_b, psi)
    u = ca.vertcat(v, omega)

    # Kinematic Equations
    x_b_dot = v * ca.cos(psi)
    y_b_dot = v * ca.sin(psi)
    psi_dot = omega


    xdot = ca.vertcat(x_b_dot, y_b_dot, psi_dot)

    # CasADi Function
    f = ca.Function('f', [x, u], [xdot])
    return f
