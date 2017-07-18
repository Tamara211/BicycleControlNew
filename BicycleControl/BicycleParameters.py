import numpy as np
class BicycleParameters:


    def __init__(self):

        # System state representation

        # Input from IMU
        self.x = 0 # X component of the contact point between rear wheel and the ground
        self.y = 0 # Y component of the contact point between rear wheel and the ground
        self.z = 0  # Z component of the contact point between rear wheel and the ground
        self.psi = 0 # Rear frame's yaw angle
        self.phi = 0 # Rear frame's roll angle
        self.phiDot = 0 #Yaw angular velocity
        self.psiDot = 0 #Roll angular velocity

        self.delta = 0 # Steering angle (Can be controlled)
        self.deltaDot = 0 # Steering angular velocity (Can be controlled)

        #TODO Delete?
        #self.thetaR = 0 # rear wheel angle relative to the rear frame
       # self.thetaF = 0 # front wheel angle relative to the front frame

        self.v = 0  # Forward velocity

        #System parameters

        #TODO validate values

        self.wheelBase = 1.16 # m
        self.g = 9.81
        self.rearFrameMass = 12.06 # Kg
        self.rearWheelMass = 2.56 # Kg
        self.rearWheelRadius = 0.34 #m

        self.frontFrameMass = 2.54 # Kg
        self.frontFrameTilt = 0.34 # Radians
        self.frontFrameTrail = 0.08 #TODO measure again
        self.frontWheelMass = 5.4 # Kg - including the motor?
        self.frontWheelRadius = 34 #cm

        self.rearWheelInertialMatrix
        self.rearFrameInertialMatrix
        self.frontFrameInertialMatrix
        self.frontWheelInertialMatrix

        self.rearFrameCenterOfMass
        self.frontFrameCenterOfMass
