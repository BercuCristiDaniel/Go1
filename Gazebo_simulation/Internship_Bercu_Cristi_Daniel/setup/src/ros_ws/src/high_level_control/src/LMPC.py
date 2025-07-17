import casadi as ca
import numpy as np

def linear_MPC(Npred, x0, u0, n, m, dt, Q, R, xmin, xmax, xref, P, b):
    solver = ca.Opti()

    # Decision variables
    x = solver.variable(n, Npred + 1)  # States
    u = solver.variable(m, Npred)      # Controls

    # Parameters
    xinit = solver.parameter(n)
    uinit = solver.parameter(m)
    xref_param = solver.parameter(n, Npred + 1)

    # Set parameter values
    solver.set_value(xinit, x0)
    solver.set_value(uinit, u0)
    solver.set_value(xref_param, xref)

    # Initial condition
    solver.subject_to(x[:, 0] == xinit)

    # Objective function
    objective = 0
    for k in range(Npred):
        # Dynamics: simple integrator model (x_k+1 = x_k + dt * u_k)
        solver.subject_to(x[:, k + 1] == x[:, k] + dt * u[:, k])

        # State and control constraints
        # solver.subject_to(xmin <= x[:, k + 1])
        # solver.subject_to(x[:, k + 1] <= xmax)
        solver.subject_to(P @ u[:, k] <= b)

        # Cost: tracking + control effort
        x_err = x[:, k] - xref_param[:, k]
        if k == 0:
            u_err = u[:, k]
        else:
            u_err = u[:, k] - u[:, k-1]
        objective += ca.mtimes([x_err.T, Q, x_err]) + ca.mtimes([u_err.T, R, u_err])

    # Terminal cost
    terminal_err = x[:, Npred] - xref_param[:, Npred]
    objective += ca.mtimes([terminal_err.T, Q, terminal_err])

    # Set objective
    solver.minimize(objective)

    # Solver settings
    solver.solver('ipopt', {
        'ipopt.print_level': 0,
        'print_time': 0,
        'verbose': False
    })

    # Solve the problem
    sol = solver.solve()

    # Return first control input
    return sol.value(u[:, 0])
