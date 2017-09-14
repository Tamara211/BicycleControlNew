from EquationsCalculation import EquationsCalculation
import listenerCombined
import Helper
import Initializer

class Controller:
	
	def __init__(self):
		self.result = EquationsCalculation()
		
		#last input from GUI
		steerAngle = 0
		velocity = 0
		
		#input from GUI
		maxRollAngle = 0
		minRollAngle = 0
		desiredVeloSteer = [0,0]
		
		#output
		msgSteerDenied = "Steering denied!"
		
		#mapped motor values
		mappedSteerAngle = 0
		mappedVelocity = b'\x00'
		
		# Input from IMU
		self.x = 0 # X component of the contact point between rear wheel and the ground
		self.y = 0 # Y component of the contact point between rear wheel and the ground
		self.z = 0  # Z component of the contact point between rear wheel and the ground
		self.psi = 0 # Rear frame's yaw angleself.phi = 0 # Rear frame's roll angle
		self.phiDot = 0 #Yaw angular velocity
		self.psiDot = 0 #Roll angular velocity
		self.delta = 0 # Steering angle (Can be controlled)
		self.deltaDot = 0 # Steering angular velocity (Can be controlled)
		
		#TODO Delete?
		#self.thetaR = 0 # rear wheel angle relative to the rear frame
		# self.thetaF = 0 # front wheel angle relative to the front frame
		
		#Initialization
		#initialize components
		arduino = Initializer.initArduino()
		dynamixel = Initializer.initDynamixel()
		
		#initialize Listener
		L = listenerCombined.listenerCombined()
		listenerCombined.listener("CONTROL_LISTENER")

		pubSteerDenied = rospy.Publisher('SteeringDenied', String, queue_size=10)
		pubVelo = rospy.Publisher('CurrentVelocity', String, queue_size=10)
		pubSteerAngle = rospy.Publisher('CurrentSteeringAngle', String, queue_size=10)
		
	while True:
		
		desiredVeloSteer = L.getTargets()
		
		if (fallingOver):
			#TODO write reaction method
				
		#if new velocity input from GUI save desired velocity and map motor values
		if (velocity != desiredVeloSteer[0]):
			velocity = desiredVelocity
			mappedVelocity = Helper.mapTargetVelocityToMotorValue(desiredVelocity)
		if (steerAngle != desiredVeloSteer[1]):
			if(requestSteer()):
				steerAngle = desiredVeloSteer[1]
				setAngle()
			else:
				print("Steering not allowed. Steering angle to big")
				#TODO publish (maybe custom) steering denied message
				pubSteerDenied.publish(msgSteerDenied)
			
		if (obstacleAhead):
			#TODO react to obstacle
			
		#send motor value to motor (accelerate)
		setVelocity()
		
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
	
	#send velocity motor value to motor (apply acceleration)
	def setVelocity(self):
	
		global velocity
		global mappedVelocity
	
		try:
			print("targetVelocity", velocity)
			
			#send velocity/motorValue to motor
			arduino.flushInput()
			arduino.flushOutput()
			arduino.write(mappedVelocity)
			
			pubVelo.publish(velocity)
			
		#if keyboard input to stop
		except (KeyboardInterrupt):
			mappedVelocity = b'\x00'
			#send new velocity/motorValue to motor
			arduino.flushInput()
			arduino.flushOutput()
			arduino.write(mappedVelocity)

	#Set Angle, map it and send it to dynamixel (perform steer)
	def setAngle(self):
	
	print('setAngle begins')
	
	global steerAngle
	global mappedSteerAngle
		
	mappedSteerAngle = mapTargetAngleToMotorValue(steerAngle)

	if mappedSteerAngle <= 100 or mappedSteerAngle >= 4000:
		return

	# Write goal position
	dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_MOVING_SPEED, 200)
	dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, mappedSteerAngle)

	if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
		dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
	elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
		dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
		
	pubSteerAngle.publish(steerAngle)

