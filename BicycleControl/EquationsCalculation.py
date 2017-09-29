import numpy as np
from numpy.ma import cos, sin, tan
from ConstantMatrices import ConstantMatrices

class EquationsCalculation:

    kp = 3.5 #TODO validate with bike
    kd = 0.035
    stepSize = 0.05 #check - step size for euler method

    def __init__(self):

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


    def calculateTorque(self, delta, deltaDot):
        self.deltaTorque = self.kp*(delta - self.desiredDeltaAngle) + self.kd * deltaDot

    def kinematicEquations(self, velocity):

        dq = np.zeros(10)

        dq[0] = velocity * cos(self.q[2]) * self.matrices.rearWheelRadius
        dq[1] = velocity * sin(self.q[2]) * self.matrices.rearWheelRadius
        dq[2] = (velocity * (self.q[6] / self.matrices.wheelBase)) * np.cos(self.matrices.frontFrameTilt)
        dq[3] = self.q[4]
        dq[9] = (velocity * (self.q[8] / self.matrices.wheelBase)) * np.cos(self.matrices.frontFrameTilt)

        return dq


    def dynamicEquations(self, velocity):

        dq = np.zeros(10)

        K = np.add(self.matrices.g * self.matrices.K_0, np.square(velocity) * self.matrices.K_2)
        C = self.velocity * self.matrices.C_1

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

    def integrateEquations(self, velocity):
        #self.updateValues()
        self.q = np.add(self.q, np.add(self.stepSize * self.kinematicEquations(), self.stepSize * self.dynamicEquations(velocity)))

    def calculateRollAngleAfterSteering(self, desiredSteeringAngle, velocity, delta, deltaDot):

        self.desiredDeltaAngle = desiredSteeringAngle
        self.calculateTorque(delta, deltaDot)
        self.integrateEquations(velocity)
        return self.q[5]
<<<<<<< HEAD
=======


    def updateValues(self): #get current values from IMU
        self.controller.x = self.q[0]
        self.controller.y = self.q[1]
        self.controller.psi = self.q[2]
        self.controller.phi = self.q[5]
        self.controller.delta = self.q[6]
        self.controller.phiDot = self.q[7]
        self.controller.deltaDot = self.q[8]
        self.controller.psiDot = self.q[9]
>>>>>>> 71819082cbb4dc737554c00bd269ec7325fdcd1b
