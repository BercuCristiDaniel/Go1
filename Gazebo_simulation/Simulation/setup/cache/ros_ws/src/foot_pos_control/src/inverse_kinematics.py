import pinocchio as pin
import numpy as np
import numpy as np
from pathlib import Path

class Go1LegDynamics:
    def __init__(self, model, data, foot_frame):
        self.model = model
        self.data = data
        self.foot_frame = foot_frame
        self.foot_id = self.model.getFrameId(foot_frame)
        self.leg_frame_id = self.model.getFrameId(foot_frame)

    def forward_kinematics(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.leg_frame_id].translation
    

    def jacobian(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.computeFrameJacobian(self.model, self.data, q, self.leg_frame_id, pin.LOCAL)
        return J

    def mass_matrix(self, q):
        return pin.crba(self.model, self.data, q)

    def coriolis_matrix(self, q, dq):
        return pin.computeCoriolisMatrix(self.model, self.data, q, dq)

    def gravity_vector(self, q):
        return pin.computeGeneralizedGravity(self.model, self.data, q)
    
    @staticmethod
    def inverse_kinematics_pinocchio(model, data, frame_id, q_init, x_des, max_iters=30, eps=1e-4, alpha=0.5):
        q = q_init.copy()
        for _ in range(max_iters):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacement(model, data, frame_id)
            x_curr = data.oMf[frame_id].translation
            err = x_des - x_curr
            if np.linalg.norm(err) < eps:
                return q
            J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL_WORLD_ALIGNED)[0:3, :]
            dq = alpha * np.linalg.pinv(J) @ err
            q[:len(dq)] += dq
        print("IK did not converge!")
        return q

    def inverse_kinematics(self, p, l1=0.23, l2=0.23):
        # Hip abduction (q1)
        px, py, pz = p
        q1 = np.arctan2(py, -pz)

        # Rotate foot into sagittal plane (X-Z)
        c1, s1 = np.cos(q1), np.sin(q1)
        x = px
        z = c1 * pz + s1 * py

        # Distance in XZ plane
        D = np.sqrt(x**2 + z**2)
        D = np.clip(D, 1e-6, l1 + l2)

        # Law of cosines for knee
        cos_q3 = (l1**2 + l2**2 - D**2) / (2 * l1 * l2)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        q3 = np.arccos(cos_q3) - np.pi # bend direction

        # Law of cosines for hip
        cos_angle = (l1**2 + D**2 - l2**2) / (2 * l1 * D)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)

        # Thigh pitch
        q2 = -np.arctan2(x, -z) + angle

        return np.array([q1, q2, q3]) 

    
    def jacobian_dot(self, q, qdot):
        pin.forwardKinematics(self.model, self.data, q, qdot)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, q, qdot)
        pin.updateFramePlacements(self.model, self.data)
        Jdot = pin.getFrameJacobianTimeVariation(self.model, self.data, self.leg_frame_id, pin.LOCAL)
        return Jdot
    