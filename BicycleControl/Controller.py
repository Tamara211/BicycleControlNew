from EquationsCalculation import EquationsCalculation

class Controller:
		
	def __init__(self):
		self.result = EquationsCalculation()
		#input from GUI
		self.maxRollAngle = 0
		self.minRollAngle = 0
		self.desiredSteerAngle = 0
             
    while True:
		if (fallingOver):
			#TODO write reaction method
			
		if (newGUIInput):
			#TODO new velocity input? adjust desired velocity and map motor values
			#TODO requestSteer check if steering possible or allowed
			#TODO map dynamixel values and send to dynamixel
			
		if (obstacleAhead):
			#TODO react to obstacle
			
		#TODO send motor value to motor
		
		
	def requestSteer(self):
		resultedRollAngle = self.result.calculateRollAngleAfterSteering(self.desiredSteeringAngle)
		if (resultedRollAngle > self.maxRollAngle or resultedRollAngle < self.minRollAngle):
			return 0  # Cannot steer
		else:  #  allow steering
			return 1  # Can steer
			
	def fallingOver(self):
		return True
		
	def obstacleAhead(self):
		return True



