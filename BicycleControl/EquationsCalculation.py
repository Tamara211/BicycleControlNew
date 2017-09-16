import numpy as np
from numpy.ma import cos, sin, tan
from ConstantMatrices import ConstantMatrices
from Controller import Controller

class EquationsCalculation:

    kp = 3.5 #TODO validate with bike
    kd = 0.035
    stepSize = 0.05 #check - step size for euler method

    def __init__(self):

        self.controller=Controller()
        self.matrices = ConstantMatrices()
        self.deltaTorque = 0# Steering torque
        self.desiredDeltaAngle = 0 #TODO recieve from visualization
        self.q = np.zeros(10)

        # q_0: x coordinates
        # q_1: y coordinates
        # q_2: yaw angle
        # q_3: wheel angle
        # q_4: wheel angular rate
        # q_5: roll angle
        # q_6: steer angle
        # q_7: roll anglular rate
        # q_8: steer anglular rate
        # q_9: yaw angular rate


    def calculateTorque(self):
        self.deltaTorque = self.kp*(self.controller.delta - self.desiredDeltaAngle) + self.kd * self.controller.deltaDot

    def kinematicEquations(self):

        dq = np.zeros(10)

        dq[0] = self.controller.v * cos(self.q[2]) * self.matrices.rearWheelRadius
        dq[1] = self.controller.v * sin(self.q[2]) * self.matrices.rearWheelRadius
        dq[2] = (self.controller.v * (self.q[6] / self.matrices.wheelBase)) * np.cos(self.matrices.frontFrameTilt)
        dq[3] = self.q[4]
        dq[9] = (self.controller.v * (self.q[8] / self.matrices.wheelBase)) * np.cos(self.matrices.frontFrameTilt)

        return dq


    def dynamicEquations(self):

        dq = np.zeros(10)

        K = np.add(self.matrices.g * self.matrices.K_0, np.square(self.controller.v) * self.matrices.K_2)
        C = self.controller.v * self.matrices.C_1

        dq[4] = input[2] / \
             (np.square(self.matrices.rearWheelRadius) * self.matrices.m_T + self.matrices.I_Ryy + \
             np.square(self.matrices.rearWheelRadius / self.matrices.frontWheelRadius) * \
             self.matrices.I_Fyy)

        M_inv = np.linalg.inv(self.matrices.M)

        c_term = np.dot(C, np.array([self.q[7], self.q[8]]))
        k_term = np.dot(K, np.array([self.q[5], self.q[6]]))
        l_term = np.add(c_term, k_term)
        r_term = np.subtract(np.array(0, self.deltaTorque), l_term)
        ddq = np.dot(M_inv, np.array([r_term[0, 0], r_term[0, 1]]))

        dq[5] = self.q[7]
        dq[6] = self.q[8]
        dq[7] = ddq[0, 0]
        dq[8] = ddq[0, 1]

        return dq

    # q_0: x coordinates
    # q_1: y coordinates
    # q_2: yaw angle
    # q_3: wheel angle
    # q_4: wheel angular rate
    # q_5: roll angle
    # q_6: steer angle
    # q_7: roll anglular rate
    # q_8: steer anglular rate
    # q_9: yaw angular rate

    def integrateEquations(self):
        self.updateValues()
        self.q = np.add(self.q, np.add(self.stepSize * self.kinematicEquations(), self.stepSize * self.dynamicEquations()))

    def calculateRollAngleAfterSteering(self, desiredSteeringAngle):

        self.desiredDeltaAngle = desiredSteeringAngle
        self.calculateTorque()
        self.integrateEquations()
        return self.q[5]


    def updateValues(self):
        self.controller.x = self.q[0]
        self.controller.y = self.q[1]
        self.controller.psi = self.q[2]
        self.controller.phi = self.q[5]
        self.controller.delta = self.q[6]
        self.controller.phiDot = self.q[7]
        self.controller.deltaDot = self.q[8]
        self.controller.psiDot = self.q[9]
