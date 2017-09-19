import serial
from Motor import Motor

motor = Motor()
motor.initArduino()

#take user input and set velocity
while True:
	velocity = input('Velocity? (Integer[0-20])')
	velocity = int(velocity)
	motor.mapTargetVelocityToMotorValue(velocity)
	motor.setVelocity()
