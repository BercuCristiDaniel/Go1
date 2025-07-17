#!/usr/bin/python3

import casadi as ca
import numpy as np

def MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, delta_u_min, delta_u_max, Q, R, P, xref):
    solver = ca.Opti()
    
    # Decision variables
    x = solver.variable(n, Npred + 1)  # States over the horizon
    u = solver.variable(m, Npred)     # Control inputs over the horizon

    # Parameters
    xinit = solver.parameter(n)  # Current state
    uinit = solver.parameter(m)  # Previous control input

    solver.set_value(xinit, x0)
    solver.set_value(uinit, u0)

    # Initial state constraint
    solver.subject_to(x[:, 0] == xinit)

    # Dynamics and constraints
    for k in range(Npred):
        solver.subject_to(x[:, k+1] == x[:, k] + dt * f(x[:, k], u[:, k]))  # Dynamics
        solver.subject_to(umin <= u[:, k])  # Control constraints
        solver.subject_to(u[:, k] <= umax)
        solver.subject_to(xmin <= x[:, k+1])  # State constraints
        solver.subject_to(x[:, k+1] <= xmax)
        
        if k == 0:
            solver.subject_to(delta_u_min <= u[:, k] - uinit)  # Rate of change constraints
            solver.subject_to(u[:, k] - uinit <= delta_u_max)
        else:
            solver.subject_to(delta_u_min <= u[:, k] - u[:, k-1])
            solver.subject_to(u[:, k] - u[:, k-1] <= delta_u_max)

    # Objective function
    objective = 0
    for k in range(Npred):
        state_error = x[:, k] - xref[:, k]
        control_effort = u[:, k]
        objective += ca.mtimes(state_error.T @ Q, state_error)
        objective += ca.mtimes(control_effort.T @ R, control_effort)

    # Terminal cost
    terminal_error = x[:, Npred] - xref[:, Npred]
    objective += ca.mtimes(terminal_error.T @ P, terminal_error)

    solver.minimize(objective)

    # Solver options
    solver.solver('ipopt', {'ipopt.print_level': 0, 'print_time': 0})
    sol = solver.solve()

    return sol.value(u[:, 0])




# def mpc(Npred, dt, dynamics_fn, constraints_fn, Q, R, P, umin, umax, xmin, xmax, delta_u_min, delta_u_max):
    # """
    # Sets up the MPC optimization problem using encapsulated dynamics and constraints.
    # Returns the MPC solver and its symbolic parameters.
    # """
    # solver = ca.Opti()  # CasADi optimization problem

    # # Dimensions
    # state_dim = 12
    # control_dim = 12  # GRFs (4 legs x 3 components)

    # # Decision variables
    # X = solver.variable(state_dim, Npred + 1)  # States
    # U = solver.variable(control_dim, Npred)    # Controls (GRFs)

    # # Parameters
    # X0 = solver.parameter(state_dim)    # Initial state
    # Xref = solver.parameter(state_dim)  # Reference state
    # r_leg_live = solver.parameter(3, 4)  # Live leg positions (3x4 matrix)

    # # Objective function
    # cost = 0
    # for k in range(Npred):
    #     # State cost
    #     cost += ca.mtimes((X[:, k] - Xref).T @ Q, (X[:, k] - Xref))
    #     # Control cost
    #     cost += ca.mtimes(U[:, k].T @ R, U[:, k])

    # # Terminal cost
    # cost += ca.mtimes((X[:, Npred] - Xref).T @ P, (X[:, Npred] - Xref))

    # # Dynamics and constraints
    # for k in range(Npred):
    #     # Use the encapsulated dynamics function
    #     state_dot = dynamics_fn(X[:, k], U[:, k], r_leg_live)
    #     solver.subject_to(X[:, k+1] == X[:, k] + dt * state_dot)

    #     # Control limits
    #     solver.subject_to(umin <= U[:, k])
    #     solver.subject_to(U[:, k] <= umax)
    #     # State limits
    #     solver.subject_to(xmin <= X[:, k+1])
    #     solver.subject_to(X[:, k+1] <= xmax)
    #     # Delta control limits
    #     if k > 0:
    #         solver.subject_to(delta_u_min <= U[:, k] - U[:, k-1])
    #         solver.subject_to(U[:, k] - U[:, k-1] <= delta_u_max)

    #     # Contact constraints
    #     solver.subject_to(constraints_fn(U[:, k]) <= 0)

    # # Initial state constraint
    # solver.subject_to(X[:, 0] == X0)

    # # Solver options
    # solver.minimize(cost)
    # solver.solver('ipopt')

    # # Return solver and parameters
    # return solver, X0, Xref, r_leg_live





