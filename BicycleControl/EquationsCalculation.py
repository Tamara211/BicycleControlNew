import numpy as np
from numpy.ma import cos, sin, tan
from BicycleParameters import BicycleParameters
from ConstantMatrices import ConstantMatrices

class EquationsCalculation:

    kp = 4.28  # TODO validate value - Constant value in torque calculation

    def __init__(self):

        self.params = BicycleParameters()
        self.matrices = ConstantMatrices()
        self.deltaTorque # Steering torque
        self.desiredDeltaAngle #TODO recieve from visualization


    def calculateTorque(self):
        self.deltaTorque = self.kp*(self.params.delta - self.desiredDeltaAngle) #TODO is it ok to ignore the kd(deltaDot) ?

    def kinematicEquations(self):
        self.params.x = self.params.v * cos(self.params.psi) * self.params.rearWheelRadius
        self.params.y = self.params.v * sin(self.params.psi) * self.params.rearWheelRadius
        self.params.psi = ((self.params.v * self.params.delta)/self.params.wheelBase ) * cos(self.params.frontFrameTilt)


    def dynamicEquations(self):

