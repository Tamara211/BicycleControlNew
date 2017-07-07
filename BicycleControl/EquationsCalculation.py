import numpy as np
from numpy.ma import cos, sin, tan
from BicycleParameters import BicycleParameters
from ConstantMatrices import ConstantMatrices

class EquationsCalculation:
    def __init__(self):

        self.params = BicycleParameters()
        self.matrices = ConstantMatrices()

    def kinematicEquations(self):
        self.params.x = self.params.v * cos(self.params.psi) * self.params.rearWheelRadius
        self.params.y = self.params.v * sin(self.params.psi) * self.params.rearWheelRadius
        self.params.psi = ((self.params.v * self.params.delta)/self.params.wheelBase ) * cos(self.params.frontFrameTilt)


    def dynamicEquations(self):
