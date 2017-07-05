
class BicycleParameters:


    def __init__(self):

        # System state representation

        #/validate with IMU
        self.x = 0 # X component of the contact point between rear wheel and the ground
        self.y = 0 # Y component of the contact point between rear wheel and the ground
        self.psi = 0 # Rear frame's yaw angle
        self.phi = 0 # Rear frame's roll angle
        self.delta = 0 # Steering angle (Can be controlled)
        self.thetaR = 0 # rear wheel angle relative to the rear frame
        self.thetaF = 0 # front wheel angle relative to the front frame

        #System parameters

        #validate values
        self.w # Wheel base
        self.c #trail
        self.lamda # Steer axis tilt
        self.g= 9.81 #gravity
        self.v # Forward speed (Can be controlled)
        self.m #total bicycle mass
        self.iRxx # Mass moments of inertia of the wear wheel
        self.iRyy # Mass moments of inertia of the wear wheel
        self.iFxx # Mass moments of inertia of the front wheel
        self.iFyy # Mass moments of inertia of the front wheel
        self.r #rear wheel raduis