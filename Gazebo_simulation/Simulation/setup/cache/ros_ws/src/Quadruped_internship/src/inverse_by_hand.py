import numpy as np

ABAD_LINK_LENGTH = 0.08         # Abduction link offset (0.08m ≈ 0.08, from hip joint origin shift)
HIP_LINK_LENGTH = 0.213         # Thigh (hip-to-knee) length
KNEE_LINK_LENGTH = 0.213        # Calf (knee-to-foot) length

# Example hip position relative to body (e.g., front-right leg)
P_HIP_TO_BODY_FR = np.array([0.1881, -0.04675, 0.0])
P_HIP_TO_BODY_FL = np.array([0.1881,  0.04675, 0.0])
P_HIP_TO_BODY_RR = np.array([-0.1881, -0.04675, 0.0])
P_HIP_TO_BODY_RL = np.array([-0.1881,  0.04675, 0.0])

def forward_kinematics(q, side_sign, abad_len=ABAD_LINK_LENGTH, hip_len=HIP_LINK_LENGTH, knee_len=KNEE_LINK_LENGTH):
    l1 = side_sign * abad_len
    l2 = -hip_len
    l3 = -knee_len

    q1, q2, q3 = q
    s1, s2, s3 = np.sin(q)
    c1, c2, c3 = np.cos(q)

    c23 = c2 * c3 - s2 * s3
    s23 = s2 * c3 + c2 * s3

    x = l3 * s23 + l2 * s2
    y = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1
    z =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2

    return np.array([x, y, z])

def ee_position_body_frame(q, p_hip_to_body, side_sign):
    return p_hip_to_body + forward_kinematics(q, side_sign)

def jacobian(q, side_sign, abad_len=ABAD_LINK_LENGTH, hip_len=HIP_LINK_LENGTH, knee_len=KNEE_LINK_LENGTH):
    l1 = side_sign * abad_len
    l2 = -hip_len
    l3 = -knee_len

    q1, q2, q3 = q
    s1, s2, s3 = np.sin(q)
    c1, c2, c3 = np.cos(q)

    c23 = c2 * c3 - s2 * s3
    s23 = s2 * c3 + c2 * s3

    J = np.zeros((3, 3))
    J[0, 0] = 0
    J[1, 0] = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1
    J[2, 0] = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1

    J[0, 1] = l3 * c23 + l2 * c2
    J[1, 1] = l3 * s1 * s23 + l2 * s1 * s2
    J[2, 1] = -l3 * c1 * s23 - l2 * c1 * s2

    J[0, 2] = l3 * c23
    J[1, 2] = l3 * s1 * s23
    J[2, 2] = -l3 * c1 * s23

    return J

def q1_ik(py, pz, l1):
    L = np.sqrt(py**2 + pz**2 - l1**2)
    return np.arctan2(pz * l1 + py * L, py * l1 - pz * L)

def q3_ik(b3z, b4z, b):
    temp = (b3z**2 + b4z**2 - b**2) / (2 * abs(b3z * b4z))
    temp = np.clip(temp, -1.0, 1.0)
    return -(np.pi - np.arccos(temp))

def q2_ik(q1, q3, px, py, pz, b3z, b4z):
    a1 = py * np.sin(q1) - pz * np.cos(q1)
    a2 = px
    m1 = b4z * np.sin(q3)
    m2 = b3z + b4z * np.cos(q3)
    return np.arctan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1)

def inverse_kinematics(p_ee, frame='HIP', p_hip_to_body=None, side_sign=1):
    if frame == 'BODY':
        assert p_hip_to_body is not None
        p = p_ee - p_hip_to_body
    else:
        p = p_ee

    px, py, pz = p
    b2y = side_sign * ABAD_LINK_LENGTH
    b3z = -HIP_LINK_LENGTH
    b4z = -KNEE_LINK_LENGTH
    a = ABAD_LINK_LENGTH
    c = np.linalg.norm(p)
    b = np.sqrt(c**2 - a**2)

    q1 = q1_ik(py, pz, b2y)
    q3 = q3_ik(b3z, b4z, b)
    q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z)

    return np.array([q1, q2, q3])


def calc_qd_from_vee(q, v_ee, side_sign):
    J = jacobian(q, side_sign)
    return np.linalg.pinv(J) @ v_ee