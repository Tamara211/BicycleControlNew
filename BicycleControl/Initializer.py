import sys
import time
from operator import itemgetter, attrgetter
from itertools import count, starmap
from pyglet import event
import math
from datetime import tzinfo, timedelta, datetime
from thread import start_new_thread
import struct
import rospy

import os




class Initializer:

    def __init__(self):
		
		if os.name == 'nt':
			import msvcrt
			def getch():
			return msvcrt.getch().decode()
		else:
			import sys, tty, termios
			fd = sys.stdin.fileno()
			old_settings = termios.tcgetattr(fd)
			def getch():
				try:
					tty.setraw(sys.stdin.fileno())
					ch = sys.stdin.read(1)
				finally:
					termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
				return ch