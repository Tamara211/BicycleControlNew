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
		
		#output
		msgSteerDenied = "Steering denied!"
		
		# Input from IMU
		self.x = 0 # X component of the contact point between rear wheel and the ground
		self.y = 0 # Y component of the contact point between rear wheel and the ground
		self.z = 0  # Z component of the contact point between rear wheel and the ground
		self.psi = 0 # Rear frame's yaw angle
        self.phi = 0 # Rear frame's roll angle
		self.phiDot = 0 #Yaw angular velocity
		self.psiDot = 0 #Roll angular velocity
		self.delta = 0 # Steering angle (Can be controlled)
		self.deltaDot = 0 # Steering angular velocity (Can be controlled)
		
		#TODO Delete?
		#self.thetaR = 0 # rear wheel angle relative to the rear frame
		# self.thetaF = 0 # front wheel angle relative to the front frame
		
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
            return 0  # Cannot steer
        else:  # allow steering
            return 1  # Can steer


    def fallingOver(self):
        while True:
            if (self.phi> 3 | self.phi < 3):  # TODO adjust values
                return True


    def obstacleAhead(self):
        return True
		
	while True:
		
		orientation = L.getOrientation
        self.x=orientation[0]
        self.y=orientation[1]
        self.z=orientation[2]
        #TODO figure out how to calculate angles, perform calculation here and assign values to attributes

		desiredVeloSteer = L.getTargets()
		
		if (fallingOver):
            stabilize(self)
			#TODO implement stabilize
				
		#if new velocity input from GUI save desired velocity and map motor values
		if (velocity != desiredVeloSteer[0]):
			velocity = desiredVelocity
			motor.mapTargetVelocityToMotorValue(desiredVelocity)
			
		if (steerAngle != desiredVeloSteer[1]):
			if(requestSteer()):
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
