import serial
import Motor

motor = Motor()

#take user input and set velocity
while True:
	velocity = input('Velocity? (Integer[0-20])')
	velocity = int(velocity)
	motor.mapTargetVelocityToMotorValue(velocity)
	motor.setVelocity()
