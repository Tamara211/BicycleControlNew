import numpy as np
from numpy.ma import cos, sin, tan
from BicycleParameters import BicycleParameters
from ConstantMatrices import ConstantMatrices

class EquationsCalculation:

    kp = 3.5 #TODO validate with bike
    kd = 0.035

    def __init__(self):

        self.params = BicycleParameters()
        self.matrices = ConstantMatrices()
        self.deltaTorque # Steering torque
        self.desiredDeltaAngle #TODO recieve from visualization


    def calculateTorque(self):
        self.deltaTorque = self.kp*(self.params.delta - self.desiredDeltaAngle) + self.kd * self.params.deltaDot

    def kinematicEquations(self):
        self.params.x = self.params.v * cos(self.params.psi) * self.params.rearWheelRadius
        self.params.y = self.params.v * sin(self.params.psi) * self.params.rearWheelRadius
        self.params.psi = ((self.params.v * self.params.delta)/self.params.wheelBase ) * cos(self.params.frontFrameTilt)


    def dynamicEquations(self):

        K = np.add(self.params.g * self.matrices.K_0, np.square(self.params.v) * self.matrices.K_2)
        C = self.params.v * self.matrices.C_1

        #TODO Not needed?
       # wheelAngularRate = input[2] / \
        #     (np.square(self.params.rearWheelRadius) * self.m_T + self.I_Ryy + \
         #    np.square(self.params.rearWheelRadius / self.params.frontWheelRadius) * \
          #   self.I_Fyy)

        M_inv = np.linalg.inv(self.matrices.M)

        c_term = np.dot(C, np.array([self.params.phiDot, self.params.deltaDot]))
        k_term = np.dot(K, np.array([self.params.phi, self.params.delta]))
        l_term = np.add(c_term, k_term)
        r_term = np.subtract(np.array(0, self.deltaTorque), l_term)
        ddq = np.dot(M_inv, np.array([r_term[0, 0], r_term[0, 1]]))


        phiDotDot = ddq[0, 0]
        deltaDotDot = ddq[0, 1]

        return (phiDotDot,deltaDotDot);

    def integrateEquations(self):
