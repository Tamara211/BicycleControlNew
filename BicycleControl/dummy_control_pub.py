#! /usr/bin/env python2
from __future__ import print_function

import rospy
from beginner_tutorials.msg import controller
import time
from random import randint

pub = rospy.Publisher('GUIData', controller, queue_size=10)

def talker(values):
	global pub
	msg = controller()
	msg.velocity_angle = values
	#hello_str = str(distance)
	rospy.loginfo(msg)
	pub.publish(msg)

def dummyCoord():
	#t = 2
	while (True):
		
		waitTime = 5
		
		angle = -10
		
		velocity = 0
		
		values = [velocity, angle]
		
		print(values)
		talker(values)
		time.sleep(waitTime)
		
		angle = 10
		
		values = [velocity, angle]
		
		print(values)
		talker(values)
		time.sleep(waitTime)
		
		angle = 0
		
		values = [velocity, angle]
		
		print(values)
		talker(values)
		time.sleep(waitTime)
		
		velocity = 12
		
		values = [velocity, angle]
		
		print(values)
		talker(values)
		time.sleep(waitTime)	

		time.sleep(waitTime)
		
		velocity = 0
		
		values = [velocity, angle]
		
		print(values)
		talker(values)
		time.sleep(waitTime)	

		time.sleep(waitTime)

def main():
	try:
		rospy.init_node('controlTalker', anonymous=True)
		print('calling dummy')
		dummyCoord()

	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
