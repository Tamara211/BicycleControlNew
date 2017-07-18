from EquationsCalculation import EquationsCalculation

class Controller:


    def __init__(self):
        self.result = EquationsCalculation()
        #input from GUI
        self.maxRollAngle = 0
        self.minRollAngle = 0
        self.desiredSteerAngle = 0

    def requestSteer(self):
        resultedRollAngle = self.result.calculateRollAngleAfterSteering(self.desiredSteeringAngle)
        if (resultedRollAngle > self.maxRollAngle or resultedRollAngle < self.minRollAngle):
            return 0  # Cannot steer
        else:  #  allow steering
             return 1  # Can steer



