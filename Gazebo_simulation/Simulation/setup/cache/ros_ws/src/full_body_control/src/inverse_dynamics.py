#!/usr/bin/env python3

import numpy as np

def impedance_control_leg(q, qdot, x_d, xdot_d, xddot_d, leg):
    # Gains
    Kx = np.diag([1000, 1000, 1000])  # stiffness
    Bx = np.diag([44, 44, 44])        # damping


    return tau
