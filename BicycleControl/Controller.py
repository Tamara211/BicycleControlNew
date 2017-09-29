from EquationsCalculation import EquationsCalculation

sys.path.insert(0,"/home/bicycle/catkin_ws/src/communication/src")
import listenerCombined
import numpy as np
import Initializer
import Motor
import Steering

#initialize Controller
controller = Controller()

class Controller:

	#last input from GUI
	steerAngle = 0
	velocity = 0
		
	#input from GUI
	maxRollAngle = 0
	minRollAngle = 0
	desiredVeloSteer = [0,0]
	
	#components
	motor = None
	steering = None
	
	#Listener
	L = None
	
	#Publishers
	pubSteerDenied = None
	pubVelo = None
	pubSteerAngle = None
	
	def __init__(self):
		self.result = EquationsCalculation()

		#Limits on roll angle (tilt), can be changed to allow different stability strictness
		self.maxRollAngle=15
		self.minRollAngle=-15
		
		#output
		msgSteerDenied = "Steering denied!"
		
		# Input from IMU
		self.v = 0 #Velocity
		self.x = 0 # X component of the contact point between rear wheel and the ground
		self.y = 0 # Y component of the contact point between rear wheel and the ground
		self.z = 0  # Z component of the contact point between rear wheel and the ground
		self.psi = 0 # Rear frame's yaw angle
        self.phi = 0 # Rear frame's roll angle
		self.phiDot = 0 #Yaw angular velocity
		self.psiDot = 0 #Roll angular velocity
		self.delta = 0 # Steering angle (Can be controlled)
		self.deltaDot = 0 # Steering angular velocity (Can be controlled)
		
		#Initialization
		#initialize components
		motor = Motor()
		steering = Steering()
		
		#initialize Listener
		L = listenerCombined.listenerCombined()
		listenerCombined.listener("CONTROL_LISTENER")

		#Initialize Publishers
		pubSteerDenied = rospy.Publisher('SteeringDenied', String, queue_size=10)
		pubVelo = rospy.Publisher('CurrentVelocity', String, queue_size=10)
		pubSteerAngle = rospy.Publisher('CurrentSteeringAngle', String, queue_size=10)


    def requestSteer(self, desiredAngle):
        resultedRollAngle = self.result.calculateRollAngleAfterSteering(desiredAngle)
        if (resultedRollAngle > self.maxRollAngle or resultedRollAngle < self.minRollAngle):
            return False  # Cannot steer
        else:  # allow steering
            return True  # Can steer


    def fallingOver(self):
        while True:
            if (self.phi> self.maxRollAngle | self.phi < self.minRollAngle):  # TODO adjust values
                return True


    def obstacleAhead(self):
        return True
		
	while True:

		orientation = L.getOrientation
		angles = L.getYawPitchRoll
        angularVelocity = L.getAngularVelocity
        self.x=orientation[0]
        self.y=orientation[1]
        self.z=orientation[2]
        self.phi = angles[2]
        self.psi = angles[0]
        self.phiDot = angularVelocity[2]
        self.psiDot = angularVelocity[0]
        #TODO how do we get delta and deltaDot?

		desiredVeloSteer = L.getTargets()
		
		if (fallingOver):
            stabilize(self)
			#TODO implement stabilize
				
		#if new velocity input from GUI save desired velocity and map motor values
		if (velocity != desiredVeloSteer[0]):
			velocity = desiredVelocity
			motor.mapTargetVelocityToMotorValue(desiredVelocity)
			
		if (steerAngle != desiredVeloSteer[1]):
			if(requestSteer(desiredVeloSteer[1])):
				steerAngle = desiredVeloSteer[1]
				steering.setAngle(steerAngle)
				pubSteerAngle.publish(steerAngle)
			else:
				print("Steering not allowed. Steering angle to big")
				#TODO publish (maybe custom) steering denied message
				pubSteerDenied.publish(msgSteerDenied)
			
		if (obstacleAhead):
			#TODO react to obstacle
			
		#send motor value to motor (accelerate)
		motor.setVelocity()
		pubVelo.publish(velocity)
