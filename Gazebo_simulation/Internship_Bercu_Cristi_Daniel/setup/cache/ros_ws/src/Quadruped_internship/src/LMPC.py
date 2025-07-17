import casadi as ca
import numpy as np
from feedback_linearization import FeedbackLinearization

def linear_MPC(Npred, x0, u0, n, m, dt, Q, R,xmin, xmax, umin, umax, xref, roll, pitch, yaw):
    solver = ca.Opti()

    x = solver.variable(n, Npred + 1)
    u = solver.variable(m, Npred)

    xinit = solver.parameter(n)
    uinit = solver.parameter(m)

    solver.set_value(xinit, x0)
    solver.set_value(uinit, u0)

    solver.subject_to(x[:, 0] == xinit)
    feedback_linearizer = FeedbackLinearization()

    objective = 0
    for k in range(Npred):
        solver.subject_to(x[:, k+1] == x[:, k] + dt * u[:, k])
        solver.subject_to(xmin<=x[:, k+1])
        solver.subject_to(x[:, k+1] <= xmax)
        solver.subject_to(umin<=u[:, k])
        solver.subject_to(u[:, k] <= umax)

        state_error = x[:, k] - xref[:, k]
        control_effort = u[:, k]
        objective += ca.mtimes(state_error.T, ca.mtimes(Q, state_error))
        objective += ca.mtimes(control_effort.T, ca.mtimes(R, control_effort))

    terminal_error = x[:, Npred] - xref[:, Npred]
    objective += ca.mtimes(terminal_error.T, ca.mtimes(Q, terminal_error))

    solver.minimize(objective)

    solver.solver('ipopt', {'ipopt.print_level': 0, 'print_time': 0})
    sol = solver.solve()

    return sol.value(u[:, 0])