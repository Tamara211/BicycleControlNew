import Steering

sys.path.insert(0, '/home/bicycle/catkin_ws/src/bike_control/src/DynamixelSDK-master/python/dynamixel_functions_py')             # Path setting

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

steering = Steering()

#take user input and set angle
while True:
	angle = input('Angle? (Integer)')
	angle = int(angle)
	steering.setAngle(angle)