import numpy as np
from numpy.ma import cos, sin, tan
from BicycleParameters import BicycleParameters
from ConstantMatrices import ConstantMatrices

class EquationsCalculation:

    kp = 3.5 #TODO validate with bike
    kd = 0.035
    stepSize = 0.05 #check - step size for euler method

    def __init__(self):

        self.params = BicycleParameters()
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
        self.deltaTorque = self.kp*(self.params.delta - self.desiredDeltaAngle) + self.kd * self.params.deltaDot

    def kinematicEquations(self):

        dq = np.zeros(10)

        dq[0] = self.params.v * cos(self.q[2]) * self.params.rearWheelRadius
        dq[1] = self.params.v * sin(self.q[2]) * self.params.rearWheelRadius
        dq[2] = (self.params.v * (self.q[6] / self.params.wheelBase)) * np.cos(self.params.frontFrame_tilt)
        dq[3] = self.q[4]
        dq[9] = (self.params.v * (self.q[8] / self.params.wheelBase)) * np.cos(self.params.frontFrame_tilt)

        return dq


    def dynamicEquations(self):

        dq = np.zeros(10)

        K = np.add(self.params.g * self.matrices.K_0, np.square(self.params.v) * self.matrices.K_2)
        C = self.params.v * self.matrices.C_1

        dq[4] = input[2] / \
             (np.square(self.params.rearWheelRadius) * self.matrices.m_T + self.matrices.I_Ryy + \
             np.square(self.params.rearWheelRadius / self.params.frontWheelRadius) * \
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
        self.q = np.add(self.q, np.add(self.stepSize * self.kinematicEquations(), self.stepSize * self.dynamicEquations()))

    def requestSteer(self, desiredSteeringAngle):
        self.updateValues()
        self.desiredDeltaAngle = desiredSteeringAngle
        self.calculateTorque()
        self.integrateEquations()
        if(self.q[5] > 0.5 or self.q[5] < -0.5): #TODO validate values
            return 0 #Cannot steer
        else: #update values and allow steering
            self.updateValues()
        return 1 #Can steer

    def updateValues(self):
        self.params.x = self.q[0]
        self.params.y = self.q[1]
        self.params.psi = self.q[2]
        self.params.phi = self.q[5]
        self.params.delta = self.q[6]
        self.params.phiDot = self.q[7]
        self.params.deltaDot = self.q[8]
        self.params.psiDot = self.q[9]
