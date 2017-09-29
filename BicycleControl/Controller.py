from EquationsCalculation import EquationsCalculation

import sys

sys.path.insert(0,"/home/bicycle/catkin_ws/src/ControllerFinal/BicycleControl")
import listenerCombined
import numpy as np
from Motor import Motor
from Steering import Steering
import rospy
from std_msgs.msg import String
from thread import start_new_thread

class Controller:

	#last input from GUI
	steerAngle = 0
	velocity = 0
		
	#input from GUI
	maxRollAngle = 0
	minRollAngle = 0
	desiredVeloSteer = [0,0]
	
	#components
	motor = Motor()
	steering = Steering()

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
		self.motor.initArduino()
		self.steering.initDynamixel()
		
		#initialize Listener
		#L = listenerCombined.listenerCombined()
		#listenerCombined.listener("CONTROL_LISTENER")

		#Initialize Publishers
		pubSteerDenied = rospy.Publisher('SteeringDenied', String, queue_size=10)
		pubVelo = rospy.Publisher('CurrentVelocity', String, queue_size=10)
		pubSteerAngle = rospy.Publisher('CurrentSteeringAngle', String, queue_size=10)


	def requestSteer(self, desiredAngle):
		resultedRollAngle = self.result.calculateRollAngleAfterSteering(desiredAngle, self.velocity, self.delta, self.deltaDot)
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
		
	def updateValues(self):
		self.x = result.q[0]
		self.y = result.q[1]
		self.psi = result.q[2]
		self.phi = result.q[5]
		self.delta = result.q[6]
		self.phiDot = result.q[7]
		self.deltaDot = result.q[8]
		self.psiDot = result.q[9]
		
	def runListener(self):
		L = listenerCombined.listenerCombined()
		listenerCombined.listener('listener')
		
		while True:
			self.orientation = L.getOrientation()
			self.angles = L.getYawPitchRoll()
			self.angularVelocity = L.getAngularVelocity()
			self.x=self.orientation[0]
			self.y=self.orientation[1]
			self.z=self.orientation[2]
			self.phi = self.angles[2]
			self.psi = self.angles[0]
			self.phiDot = self.angularVelocity[2]
			self.psiDot = self.angularVelocity[0]
			#TODO how do we get delta and deltaDot?

			self.desiredVeloSteer = L.getTargets()
		
	def run(self):
		
		print('Controller running...')	
		
		while True:

			
			print(self.desiredVeloSteer)
			
			if (self.fallingOver()):
				stabilize(self)
				#TODO implement stabilize
					
			#if new velocity input from GUI save desired velocity and map motor values
			if (self.velocity != self.desiredVeloSteer[0]):
				self.velocity = self.desiredVelocity
				self.motor.mapTargetVelocityToMotorValue(desiredVelocity)
				self.motor.setVelocity()
				print('Velocity: ', self.velocity)
				
			if (self.steerAngle != self.desiredVeloSteer[1]):
				if(requestSteer(self.desiredVeloSteer[1])):
					self.steerAngle = self.desiredVeloSteer[1]
					self.steering.setAngle(self.steerAngle)
					print('Steering angle: ', self.steerAngle)
					self.pubSteerAngle.publish(self.steerAngle)
				else:
					print("Steering not allowed. Steering angle to big")
					#TODO publish (maybe custom) steering denied message
					self.pubSteerDenied.publish(msgSteerDenied)
				
			if (self.obstacleAhead()):
				#TODO react to obstacle
				
				#send motor value to motor (accelerate)
				self.motor.setVelocity()
				self.pubVelo.publish(velocity)

def main():
	#initialize Controller
	controller = Controller()
	#start_new_thread(controller.runListener(), ("",))
	controller.run()

if __name__ == '__main__':
	main()
