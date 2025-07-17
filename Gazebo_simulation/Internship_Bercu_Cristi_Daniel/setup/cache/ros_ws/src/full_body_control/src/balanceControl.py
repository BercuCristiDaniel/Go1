import numpy as np
import casadi as ca

class BalanceController:
    def __init__(self, mass, Ib, pcb, alpha=0.001, beta=0.1, fric_ratio=0.4):
        self.mass = mass
        self.Ib = Ib
        self.pcb = pcb
        self.alpha = alpha
        self.beta = beta
        self.fric_ratio = fric_ratio
        self.g = np.array([0, 0, -9.81])
        self.Fprev = np.zeros((12,))
        self.S = np.diag([20, 20, 50, 450, 450, 450])
        self.W = np.diag([10, 10, 4] * 4)
        self.U = np.diag([3.0] * 12)

        self.fric_mat = np.array([
            [ 1,  0,  self.fric_ratio],
            [-1,  0,  self.fric_ratio],
            [ 0,  1,  self.fric_ratio],
            [ 0, -1,  self.fric_ratio],
            [ 0,  0,  1.0]
        ])

    def compute(self, ddPcd, dWbd, rotM, feet_pos, contact):
        A = self._build_A(feet_pos, rotM)
        print(A)
        # Compute desired wrench
        bd = np.zeros(6)
        bd[:3] = self.mass * (ddPcd - self.g)
        bd[3:] = rotM @ self.Ib @ rotM.T @ dWbd

        # QP cost
        G = A.T @ self.S @ A + self.alpha * self.W + self.beta * self.U
        G = 0.5 * (G + G.T)  # enforce symmetry
        g0 = -((self.S @ A).T @ bd + self.beta * self.U @ self.Fprev)

        # QP variables
        F = ca.MX.sym("F", 12)
        objective = 0.5 * ca.mtimes([F.T, G, F]) + ca.dot(g0, F)

        # Constraints
        constraints = []
        lb = []
        ub = []

        for i in range(4):
            F_leg = ca.reshape(F[3*i:3*(i+1)], (3, 1))
            if contact[i] == 1:
                for j in range(5):
                    row = ca.DM(self.fric_mat[j].reshape(1, 3))
                    constraints.append(ca.mtimes(row, F_leg))
                    lb.append(-ca.inf)
                    ub.append(0.0)
            else:
                for j in range(3):
                    constraints.append(F_leg[j])
                    lb.append(0.0)
                    ub.append(0.0)

        # Solve QP
        qp = {"x": F, "f": objective, "g": ca.vertcat(*constraints)}
        opts = {
            "printLevel": "none",      # Disable qpOASES printout
            "verbose": False           # Optional: disable other CasADi logs
        }

        solver = ca.qpsol("solver", "qpoases", qp, opts)
        sol = solver(lbg=lb, ubg=ub)

        F_opt = np.array(sol["x"]).flatten()
        self.Fprev = F_opt

        # Logging
        F_opt_reshaped = F_opt.reshape(4, 3)
        F_total = np.sum(F_opt_reshaped, axis=0)
        torque_total = np.sum(np.cross(feet_pos - rotM @ self.pcb, F_opt_reshaped), axis=0)

        print("[BalanceCtrl] F_total:", F_total, "N")
        print("[BalanceCtrl] Torque_total:", torque_total, "Nm")
        for i in range(4):
            print(f"  Leg {i}: Force {F_opt_reshaped[i]} N")

        return F_opt_reshaped

    def _build_A(self, feet_pos, rotM):
        A = np.zeros((6, 12))
        for i in range(4):
            A[:3, 3*i:3*(i+1)] = np.eye(3)
            r = feet_pos[i] - rotM @ self.pcb
            A[3:, 3*i:3*(i+1)] = self._skew(r)
        return A

    def _skew(self, v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])