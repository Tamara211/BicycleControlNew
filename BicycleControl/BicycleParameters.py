import numpy as np
class BicycleParameters:


    def __init__(self):

        # System state representation

        #TODO validate with IMU
        # Input from IMU
        self.x = 0 # X component of the contact point between rear wheel and the ground
        self.y = 0 # Y component of the contact point between rear wheel and the ground
        self.z = 0  # Z component of the contact point between rear wheel and the ground
        self.phi = np.arctan(self.x/self.y) #TODO check
        self.v = 0 # Forward velocity

        self.psi = 0 # Rear frame's yaw angle
        self.phi = 0 # Rear frame's roll angle
        self.delta = 0 # Steering angle (Can be controlled)
        self.thetaR = 0 # rear wheel angle relative to the rear frame
        self.thetaF = 0 # front wheel angle relative to the front frame

        #System parameters

        #TODO validate values

        self.wheelBase
        self.g = 9.81
        self.rearFrameMass # Kg
        self.rearWheelMass
        self.rearWheelRadius

        self.frontFrameMass # Kg
        self.frontFrameTilt
        self.frontFrameTrail
        self.frontWheelMass
        self.frontWheelRadius

        self.rearWheelInertialMatrix
        self.rearFrameInertialMatrix
        self.frontFrameInertialMatrix
        self.frontWheelInertialMatrix

        self.rearFrameCenterOfMass
        self.frontFrameCenterOfMass
