import numpy as np

class FrameType:
    HIP = "HIP"
    BODY = "BODY"

class QuadrupedLeg:
    def __init__(self, legID, abadLinkLength, hipLinkLength, kneeLinkLength, pHip2B):
        self._abadLinkLength = abadLinkLength
        self._hipLinkLength = hipLinkLength
        self._kneeLinkLength = kneeLinkLength
        self._pHip2B = np.array(pHip2B)

        if legID in [0, 2]:
            self._sideSign = -1
        elif legID in [1, 3]:
            self._sideSign = 1
        else:
            raise ValueError("Leg ID incorrect!")

    def calcPEe2H(self, q):
        l1 = self._sideSign * self._abadLinkLength
        l2 = -self._hipLinkLength
        l3 = -self._kneeLinkLength

        s1, s2, s3 = np.sin(q)
        c1, c2, c3 = np.cos(q)

        c23 = c2 * c3 - s2 * s3
        s23 = s2 * c3 + c2 * s3

        pEe2H = np.zeros(3)
        pEe2H[0] = l3 * s23 + l2 * s2
        pEe2H[1] = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1
        pEe2H[2] =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2

        return pEe2H

    def calcPEe2B(self, q):
        return self._pHip2B + self.calcPEe2H(q)

    def calcVEe(self, q, qd):
        return self.calcJaco(q) @ qd

    def calcQ(self, pEe, frame):
        if frame == FrameType.HIP:
            pEe2H = np.array(pEe)
        elif frame == FrameType.BODY:
            pEe2H = np.array(pEe) - self._pHip2B
        else:
            raise ValueError("The frame of QuadrupedLeg::calcQ can only be HIP or BODY!")

        px, py, pz = pEe2H
        b2y = self._abadLinkLength * self._sideSign
        b3z = -self._hipLinkLength
        b4z = -self._kneeLinkLength
        a = self._abadLinkLength
        c = np.sqrt(px**2 + py**2 + pz**2)
        b = np.sqrt(c**2 - a**2)

        q1 = self.q1_ik(py, pz, b2y)
        q3 = self.q3_ik(b3z, b4z, b)
        q2 = self.q2_ik(q1, q3, px, py, pz, b3z, b4z)

        return np.array([q1, q2, q3])

    def calcQd(self, *args):
        if len(args) == 2:
            q, vEe = args
            return np.linalg.inv(self.calcJaco(q)) @ vEe
        elif len(args) == 3:
            pEe, vEe, frame = args
            q = self.calcQ(pEe, frame)
            return np.linalg.inv(self.calcJaco(q)) @ vEe
        else:
            raise ValueError("Invalid arguments for calcQd")

    def calcTau(self, q, force):
        return self.calcJaco(q).T @ force

    def calcJaco(self, q):
        l1 = self._abadLinkLength * self._sideSign
        l2 = -self._hipLinkLength
        l3 = -self._kneeLinkLength

        s1, s2, s3 = np.sin(q)
        c1, c2, c3 = np.cos(q)

        c23 = c2 * c3 - s2 * s3
        s23 = s2 * c3 + c2 * s3

        jaco = np.zeros((3, 3))
        jaco[0, 0] = 0
        jaco[1, 0] = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1
        jaco[2, 0] = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1
        jaco[0, 1] = l3 * c23 + l2 * c2
        jaco[1, 1] = l3 * s1 * s23 + l2 * s1 * s2
        jaco[2, 1] = -l3 * c1 * s23 - l2 * c1 * s2
        jaco[0, 2] = l3 * c23
        jaco[1, 2] = l3 * s1 * s23
        jaco[2, 2] = -l3 * c1 * s23

        return jaco

    def q1_ik(self, py, pz, l1):
        L = np.sqrt(py**2 + pz**2 - l1**2)
        return np.arctan2(-pz * l1 + py * L, py * l1 + pz * L)

    def q3_ik(self, b3z, b4z, b):
        temp = (b3z**2 + b4z**2 - b**2) / (2 * abs(b3z * b4z))
        temp = np.clip(temp, -1.0, 1.0)
        return -(np.pi - np.arccos(temp))

    def q2_ik(self, q1, q3, px, py, pz, b3z, b4z):
        a1 = py * np.sin(q1) - pz * np.cos(q1)
        a2 = px
        m1 = b4z * np.sin(q3)
        m2 = b3z + b4z * np.cos(q3)
        return np.arctan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1)
    


