import sys

from Steering import Steering

steering = Steering()
steering.initDynamixel()

#take user input and set angle
while True:
	angle = input('Angle? (Integer)')
	angle = int(angle)
	steering.setAngle(angle)
