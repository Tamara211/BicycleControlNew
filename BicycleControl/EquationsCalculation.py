import numpy as np
from numpy.ma import cos, sin, tan
from BicycleParameters import BicycleParameters

class EquationsCalculation:
    def __init__(self):

        self.params = BicycleParameters()

    def kinematicEquations(self):
        self.params.x = self.params.v * cos(self.params.psi) * self.params.r
        self.params.y = self.params.v * sin(self.params.psi) * self.params.r
        self.params.psi = ((self.params.v * self.params.delta)/self.params.w ) / cos(self.params.lamda)


    def dynamicEquations(self):
