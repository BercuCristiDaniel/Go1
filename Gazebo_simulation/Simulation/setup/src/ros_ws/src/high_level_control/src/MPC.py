#!/usr/bin/python3

import casadi as ca
import numpy as np

def MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, Q, R, P, xref):
    """
    Nonlinear Model Predictive Control (NMPC) solver using CasADi's Opti interface.
    
    Solves a constrained finite-horizon optimization using the provided nonlinear dynamics f(x, u).

    Args:
        Npred (int): Prediction horizon
        x0 (np.array): Initial state vector
        u0 (np.array): Previous control input (for smoothness penalty)
        n (int): Number of state variables
        m (int): Number of control inputs
        dt (float): Discretization timestep
        f (casadi.Function): Dynamics function, maps (x, u) → ẋ
        umin (np.array): Min control input (m-dimensional)
        umax (np.array): Max control input
        xmin (np.array): Min state bounds (n-dimensional)
        xmax (np.array): Max state bounds
        Q (np.array): State cost matrix
        R (np.array): Control effort cost matrix
        P (np.array): Terminal cost matrix
        xref (np.array): Desired state trajectory (n x Npred+1)
    """
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
        

    # Objective function
    objective = 0
    for k in range(Npred):
        state_error = x[:, k] - xref[:,k]
        if k==0:
            control_effort = u[:, k]
        else:
            control_effort = u[:, k] - u[:, k-1]
        objective += ca.mtimes(state_error.T @ Q, state_error)
        objective += ca.mtimes(control_effort.T @ R, control_effort)

    # Terminal cost
    terminal_error = x[:, Npred] - xref[:,Npred]
    objective += ca.mtimes(terminal_error.T @ P, terminal_error)

    solver.minimize(objective)

    # Solver options
    solver.solver('ipopt', {'ipopt.print_level': 0, 'print_time': 0})
    sol = solver.solve()

    return sol.value(u[:, 0])







