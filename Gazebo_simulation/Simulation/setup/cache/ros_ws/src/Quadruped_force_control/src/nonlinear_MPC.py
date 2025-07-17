import casadi as ca
import rospy

def MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, delta_u_min, delta_u_max, Q, R, P, xref, r_foot_positions):
    solver = ca.Opti()

    # Define Decision Variables
    x = solver.variable(n, Npred + 1)  # State trajectory (CoM pos, vel, orientation, etc.)
    u = solver.variable(m, Npred)      # Control input trajectory (ground reaction forces)

    # Define Parameters (Fixed values passed into solver)
    xinit = solver.parameter(n)  # Initial state
    uinit = solver.parameter(m)  # Previous control input
    r = solver.parameter(12)     # Foot positions relative to CoM

    solver.set_value(xinit, x0)
    solver.set_value(uinit, u0)
    solver.set_value(r, r_foot_positions)

    # Initial Condition Constraint
    solver.subject_to(x[:, 0] == xinit)

    # Friction Coefficient
    mu = 1  # Friction coefficient (adjustable based on surface)

    # Dynamics and Constraints
    for k in range(Npred):
        # Forward Euler Integration
        x_next = x[:, k] + dt * f(x[:, k], u[:, k], r)
        solver.subject_to(x[:, k+1] == x_next)
        
        # Control Input Constraints
        solver.subject_to(umin <= u[:, k])
        solver.subject_to(u[:, k] <= umax)

        # State Constraints
        solver.subject_to(xmin <= x[:, k+1])
        solver.subject_to(x[:, k+1] <= xmax)
        
        # # Friction Cone Constraint (No Division by Zero)
        # for i in range(4):  # 4 legs
        #     Fx = u[3*i + 0, k]
        #     Fy = u[3*i + 1, k]
        #     Fz = u[3*i + 2, k]

            # # Ensure a minimum Fz to prevent NaN issues
            # solver.subject_to(Fz >= 1.0)

            # # Friction Cone: sqrt(Fx² + Fy²) <= mu * Fz
            # solver.subject_to(Fx**2 + Fy**2 <= (mu * Fz) ** 2)

        # Control Rate Constraints (Prevent Large Jumps)
        if k == 0:
            solver.subject_to(delta_u_min <= u[:, k] - uinit)
            solver.subject_to(u[:, k] - uinit <= delta_u_max)
        else:
            solver.subject_to(delta_u_min <= u[:, k] - u[:, k-1])
            solver.subject_to(u[:, k] - u[:, k-1] <= delta_u_max)

    # Define Cost Function (Minimization Objective)
    objective = 0
    for k in range(Npred):
        state_error = x[:, k] - xref
        control_effort = u[:, k]

        # Quadratic cost on state deviation and control effort
        objective += ca.mtimes(state_error.T @ Q, state_error) + ca.mtimes(control_effort.T @ R, control_effort)

    # Terminal Cost
    terminal_error = x[:, Npred] - xref
    objective += ca.mtimes(terminal_error.T @ P, terminal_error)

    # Set Optimization Objective
    solver.minimize(objective)

    # Solver Settings (Improved Stability)
    solver.solver('ipopt', {
        'ipopt.print_level': 0, 
        'print_time': 0, 
        'ipopt.tol': 1e-6,  # Higher precision
        'ipopt.max_iter': 1000,  # More iterations for complex problems
    })

    try:
        # Solve the Problem
        sol = solver.solve()
        return sol.value(u[:, 0])  # Return first control action
    except RuntimeError as e:
        rospy.logerr("Solver failed! Debugging values...")

        # Print Debug Values
        for k in range(Npred):
            rospy.loginfo(f"Step {k}: x = {solver.debug.value(x[:, k])}, u = {solver.debug.value(u[:, k])}")

        raise e  # Re-raise the error for further debugging
