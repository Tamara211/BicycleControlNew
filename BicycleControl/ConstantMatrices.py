import numpy as np
from BicycleParameters import BicycleParameters

class ConstantMatrices:
    def __init__(self):

        self.params = BicycleParameters()

        self.M = []
        self.C_1 = []
        self.K_0 = []
        self.K_2 = []

        self.z_T = 0.0
        self.x_T = 0.0
        self.m_T = 0.0 #total bicycle mass
        self.I_Rxx = 0.0 #mass moments of inertia for the rear wheel
        self.I_Ryy = 0.0 #mass moments of inertia for the rear wheel
        self.I_Fxx = 0.0 #mass moments of inertia for the front wheel
        self.I_Fyy = 0.0 #mass moments of inertia for the front wheel

     # Taken from simulator code
    def calculateMatrices(self):
        # parameters

        w = self.params.wheelBase
        c = self.params.frontFrameTrail
        lmbd = self.params.frontFrameTilt

        m_R = self.rearWheelMass
        m_F = self.frontWheelMass
        m_B = self.rearFrameMass
        m_H = self.frontFrameMass

        I_R = self.rearWheelInertialMatrix
        I_B = self.rearFrameInertialMatrix
        I_H = self.frontFrameInertialMatrix
        I_F = self.frontWheelInertialMatrix

        r_R = self.params.rearWheelRadius
        r_F = self.params.frontWheelRadius

        cmass_B = self.rearFrameCenterOfMass
        cmass_H = self.frontFrameCenterOfMass

        I_Rxx = I_R[0, 0]
        I_Ryy = I_R[1, 1]
        I_Rzz = I_R[2, 2]

        I_Bxx = I_B[0, 0]
        I_Bxz = I_B[0, 2]
        I_Bzz = I_B[2, 2]

        I_Hxx = I_H[0, 0]
        I_Hxz = I_H[0, 2]
        I_Hzz = I_H[2, 2]

        I_Fxx = I_F[0, 0]
        I_Fyy = I_F[1, 1]
        I_Fzz = I_F[2, 2]

        x_B = cmass_B[0]
        z_B = cmass_B[2]

        x_H = cmass_H[0]
        z_H = cmass_H[2]

        # Equation instantiation

        # (A1)
        m_T = m_R + m_F + m_B + m_H

        # (A2)
        self.x_T = (x_B * m_B + x_H * m_H + w * m_F) / m_T
        # (A3)
        self.z_T = (-r_R * m_R + z_B * m_B + z_H * m_H - r_F * m_F) / m_T

        # (A4)
        I_Txx = I_Rxx + I_Bxx + I_Hxx + I_Fxx \
                + m_R * np.square(r_R) + m_B * np.square(z_B) + m_H * np.square(z_H) + m_F * np.square(r_F)

        # (A5)
        I_Txz = I_Bxz + I_Hxz - m_B * x_B * z_B - m_H * x_H * z_H + m_F * w * r_F

        # (A 6)
        I_Rzz = I_Rxx
        I_Fzz = I_Fxx

        # (A 7)
        I_Tzz = I_Rzz + I_Bzz + I_Hzz + I_Fzz + m_B * np.square(x_B) + m_H * np.square(x_H) + m_F * np.square(w)

        # (A 8)
        m_A = m_H + m_F

        # (A 9)
        x_A = (x_H * m_H + w * m_F) / m_A
        z_A = (z_H * m_H - r_F * m_F) / m_A

        # (A 10)
        I_Axx = I_Hxx + I_Fxx + m_H * np.square(z_H - z_A) + m_F * np.square(r_F + z_A)
        # (A 11)
        I_Axz = I_Hxz - m_H * (x_H - x_A) * (z_H - z_A) + m_F * (w - x_A) * (r_F + z_A)
        # (A 12)
        I_Azz = I_Hzz + I_Fzz + m_H * np.square(x_H - x_A) + m_F * np.square(w - x_A)

        # (A 13)
        u_A = (x_A - w - c) * np.cos(lmbd) - z_A * np.sin(lmbd)

        # (A 14)
        I_All = m_A * np.square(u_A) + I_Axx * np.square(np.sin(lmbd)) + 2 * I_Axz * np.sin(lmbd) * np.cos(
            lmbd) + I_Azz * np.square(np.cos(lmbd))

        # (A 15)
        I_Alx = -m_A * u_A * z_A + I_Axx * np.sin(lmbd) + I_Axz * np.cos(lmbd)

        # (A 16)
        I_Alz = m_A * u_A * x_A + I_Axz * np.sin(lmbd) + I_Azz * np.cos(lmbd)

        # (A 17)
        miu = (c / w) * np.cos(lmbd)

        # (A 18)
        S_R = I_Ryy / r_R
        S_F = I_Fyy / r_F
        S_T = S_R + S_F

        # (A 19)
        S_A = m_A * u_A + miu * m_T * self.x_T

        # (A 20)
        M_pp = I_Txx
        M_pd = I_Alx + miu * I_Txz
        M_dp = M_pd
        M_dd = I_All + 2 * miu * I_Alz + np.square(miu) * I_Tzz

        # (A 21)

        self.M = np.matrix([[M_pp, M_pd], [M_dp, M_dd]])
        # self.M = [[ 80.81722, 2.31941332], [2.31941332, 0.29784188]]

        # (A 22)
        K_0pp = m_T * self.z_T
        K_0pd = -S_A
        K_0dp = K_0pd
        K_0dd = -S_A * np.sin(lmbd)

        # (A 23)
        self.K_0 = np.matrix([[K_0pp, K_0pd], [K_0dp, K_0dd]])
        # self.K_0 = np.matrix([[-80.95, -2.59951685], [-2.59951685, -0.80329488]])

        # (A 24)
        K_2pp = 0
        K_2pd = ((S_T - m_T * self.z_T) / w) * np.cos(lmbd)
        K_2dp = 0
        K_2dd = ((S_A + S_F * np.sin(lmbd)) / w) * np.cos(lmbd)

        # (A 25)
        self.K_2 = np.matrix([[K_2pp, K_2pd], [K_2dp, K_2dd]])
        # self.K_2 = np.matrix([[0.0, 76.5973459], [0.0, 2.65431524]])

        # (A 26)
        C_1pp = 0
        C_1pd = miu * S_T + S_F * np.cos(lmbd) + (I_Txz / w) * np.cos(lmbd) - miu * m_T * self.z_T
        C_1dp = -(miu * S_T + S_F * np.cos(lmbd))
        C_1dd = (I_Alz / w) * np.cos(lmbd) + miu * (S_A + (I_Tzz / w) * np.cos(lmbd))

        # (A 27)
        self.C_1 = np.matrix([[C_1pp, C_1pd], [C_1dp, C_1dd]])
        # self.C_1 = np.matrix([[0.0, 33.86641391], [-0.85035641, 1.68540397]])
