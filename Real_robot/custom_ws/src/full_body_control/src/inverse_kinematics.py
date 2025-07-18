import pinocchio as pin
import numpy as np
import numpy as np
from pathlib import Path

class Go1LegDynamics:
    def __init__(self, model, data, foot_frame):
        """
        Initializes the dynamics object for a single leg of the Go1 robot.
        
        Args:
            model (pin.Model): The Pinocchio model of the robot.
            data (pin.Data): The Pinocchio data structure for storing results.
            foot_frame (str): Name of the foot frame for this leg.
        """ 
        self.model = model
        self.data = data
        self.foot_frame = foot_frame
        self.foot_id = self.model.getFrameId(foot_frame)
        self.leg_frame_id = self.model.getFrameId(foot_frame)

    def forward_kinematics(self, q):
        """
        Computes the position of the foot in the base frame given joint positions.

        Args:
            q (np.ndarray): Joint positions (angles) for the leg.

        Returns:
            np.ndarray: 3D position of the foot frame in the base frame.
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.leg_frame_id].translation
    

    def jacobian(self, q):
        """
        Computes the spatial Jacobian of the foot in the local frame.

        Args:
            q (np.ndarray): Joint positions.

        Returns:
            np.ndarray: Jacobian matrix of size (6 x n_joints).
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.computeFrameJacobian(self.model, self.data, q, self.leg_frame_id, pin.LOCAL)
        return J

    def mass_matrix(self, q):
        """
        Computes the joint-space mass (inertia) matrix using CRBA.

        Args:
            q (np.ndarray): Joint positions.

        Returns:
            np.ndarray: Mass matrix of the leg.
        """
        return pin.crba(self.model, self.data, q)

    def coriolis_matrix(self, q, dq):
        """
        Computes the Coriolis matrix, which accounts for dynamic coupling terms.

        Args:
            q (np.ndarray): Joint positions.
            dq (np.ndarray): Joint velocities.

        Returns:
            np.ndarray: Coriolis matrix.
        """
        return pin.computeCoriolisMatrix(self.model, self.data, q, dq)

    def gravity_vector(self, q):
        """
        Computes the generalized gravity vector for the current configuration.

        Args:
            q (np.ndarray): Joint positions.

        Returns:
            np.ndarray: Gravity torque vector.
        """
        return pin.computeGeneralizedGravity(self.model, self.data, q)
    
    @staticmethod
    def inverse_kinematics_pinocchio(model, data, frame_id, q_init, x_des, max_iters=50, eps=1e-4, alpha=0.5):
        """
        Solves inverse kinematics using iterative Jacobian pseudoinverse method.

        Args:
            model (pin.Model): Pinocchio model.
            data (pin.Data): Pinocchio data.
            frame_id (int): Frame ID of the target frame.
            q_init (np.ndarray): Initial joint configuration.
            x_des (np.ndarray): Desired Cartesian position of the foot.
            max_iters (int): Maximum number of iterations.
            eps (float): Tolerance for convergence.
            alpha (float): Step size factor.

        Returns:
            np.ndarray: Joint configuration that reaches the desired position (if converged).
        """
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

    
    def jacobian_dot(self, q, qdot):
        """
        Computes the time derivative of the spatial Jacobian (J̇) of the foot frame.

        Args:
            q (np.ndarray): Joint positions.
            qdot (np.ndarray): Joint velocities.

        Returns:
            np.ndarray: Time derivative of the Jacobian.
        """
        pin.forwardKinematics(self.model, self.data, q, qdot)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, q, qdot)
        pin.updateFramePlacements(self.model, self.data)
        Jdot = pin.getFrameJacobianTimeVariation(self.model, self.data, self.leg_frame_id, pin.LOCAL)
        return Jdot
    