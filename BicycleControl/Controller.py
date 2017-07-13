from EquationsCalculation import EquationsCalculation

class Controller:


    def __init__(self):
        self.result = EquationsCalculation()

    def requestSteer(self, desiredSteeringAngle):
        resultedRollAngle = self.result.calculateRollAngleAfterSteering(desiredSteeringAngle)
        if (resultedRollAngle > 0.5 or resultedRollAngle < -0.5):  # TODO validate values
            return 0  # Cannot steer
        else:  #  allow steering
             return 1  # Can steer



