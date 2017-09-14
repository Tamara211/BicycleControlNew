class Controller:
	
	def mapTargetVelocityToMotorValue(targetVelocity):
		global velocity
		#print('paramChanged: ', targetVelocity)
		#TODO: map motorValue correctly
		if targetVelocity > 0:
			velocity = b'\x69'			#100
		if targetVelocity > 10:
			velocity = b'\x69'			#105
		if targetVelocity > 11:
			velocity = b'\x6E'			#110
		if targetVelocity > 13:
			velocity = b'\x73'			#115
		if targetVelocity > 14:
			velocity = b'\x78'			#120
		if targetVelocity > 15:
			velocity = b'\x7D'			#125
		if targetVelocity > 17:
			velocity = b'\x82'			#130
		if targetVelocity > 18:
			velocity = b'\x87'			#135
		if targetVelocity > 19:
			velocity = b'\x8C'			#140
		if targetVelocity <= 10:
			velocity = b'\x00'
	
	
		return velocity
	
	def mapTargetAngleToMotorValue(targetAngle):
		global motorValue
	
		#TODO: map motorValue correctly
		if targetAngle > 0:
			motorValue = 2047 + 250
		elif targetAngle < 0:
			motorValue = 2047 - 250
		else:
			motorValue = 2047
		
		return motorValue
