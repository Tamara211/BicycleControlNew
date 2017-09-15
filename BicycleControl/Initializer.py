import sys
import time
from operator import itemgetter, attrgetter
from itertools import count, starmap
from pyglet import event
import math
from datetime import tzinfo, timedelta, datetime
import csv
from thread import start_new_thread
import struct
import rospy
sys.path.insert(0,"/home/bicycle/catkin_ws/src/communication/src")
import listenerCombined

import os

sys.path.insert(0, '/home/bicycle/catkin_ws/src/bike_control/src/DynamixelSDK-master/python/dynamixel_functions_py')             # Path setting

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library
import serial

class Initializer:

    def __init__(self):
        self.arduino=None
		
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


	# Control table address for Dynymixel MX-106R
	ADDR_MX_MODEL_NUMBER        = 0                             # (0x00) Model Number(L) Lowest byte of model number R 64 (0X40)
																# (0X01) Model Number(H) Highest byte of model number R 1 (0X01)
	ADDR_MX_FIRMWARE_VERSION    = 2                             # (0X02) Version of Firmware Information on the version of firmware R -
	ADDR_MX_ID                  = 3                             # (0X03) ID ID of Dynamixel RW 1 (0X01)
	ADDR_MX_BAUD_RATE           = 4                             # (0X04) Baud Rate Baud Rate of Dynamixel RW 34 (0X22)
	ADDR_MX_RETURN_DELAY        = 5                             # (0X05) Return Delay Time Return Delay Time RW 250 (0XFA)
	ADDR_MX_CW_ANGLE            = 6                             # (0X06) CW Angle Limit(L) Lowest byte of clockwise Angle Limit RW 0 (0X00)
																# (0X07) CW Angle Limit(H) Highest byte of clockwise Angle Limit RW 0 (0X00)
	ADDR_MX_CCW_ANGLE           = 8                             # (0X08) CCW Angle Limit(L) Lowest byte of counterclockwise Angle Limit RW 255 (0XFF)
																# (0X09) CCW Angle Limit(H) Highest byte of counterclockwise Angle Limit RW 15 (0X0F)
	ADDR_MX_DRIVE_MODE          = 10                            # (0X0A) Drive Mode Dual Mode Setting RW 0(0X00)
	ADDR_MX_TEMPERTURE_LIMIT    = 11                            # (0X0B) the Highest Limit Temperature Internal Limit Temperature RW 80 (0X50)
	ADDR_MX_LOW_LIMIT_VOLTAGE   = 12                            # (0X0C) the Lowest Limit Voltage Lowest Limit Voltage RW 60 (0X3C)
	ADDR_MX_HIGH_LIMIT_VOLTAGE  = 13                            # (0X0D) the Highest Limit Voltage Highest Limit Voltage RW 160 (0XA0)
	ADDR_MX_MAX_TORQUE          = 14                            # (0X0E) Max Torque(L) Lowest byte of Max. Torque RW 255 (0XFF)
																# (0X0F) Max Torque(H) Highest byte of Max. Torque RW 3 (0X03)
	ADDR_MX_STATUS_RETURN_LVL   = 16                            # (0X10) Status Return Level Status Return Level RW 2 (0X02)
	ADDR_MX_ALARM_LED           = 17                            # (0X11) Alarm LED LED for Alarm RW 36 (0X24)
	ADDR_MX_ALARM_SHUTDOWN      = 18                            # (0X12) Alarm Shutdown Shutdown for Alarm RW 36 (0X24)
	ADDR_MX_MULTI_TURN_OFFSET   = 20                            # (0X14) Multi Turn Offset(L) multi-turn offset least significant byte (LSB) RW 0 (0X00)
																# (0X15) Multi Turn Offset(H) multi-turn offset most significant byte (MSB) RW 0 (0X00)
	ADDR_MX_RESOLUTION_DIVIDER  = 22                            # (0X16) Resolution Divider Resolution divider RW 1 (0X01)
	ADDR_MX_TORQUE_ENABLE       = 24                            # (0X18) Torque Enable Torque On/Off RW 0 (0X00)
	ADDR_MX_ALARM_LED           = 25                            # (0X19) LED LED On/Off RW 0 (0X00)
	ADDR_MX_D_GAIN              = 26                            # (0X1A) D Gain Derivative Gain RW 0 (0X00)
	ADDR_MX_I_GAIN              = 27                            # (0X1B) I Gain Integral Gain RW 0 (0X00)
	ADDR_MX_P_GAIN              = 28                            # (0X1C) P Gain Proportional Gain RW 32 (0X20)
	ADDR_MX_GOAL_POSITION       = 30                            # (0X1E) Goal Position(L) Lowest byte of Goal Position RW -
																# (0X1F) Goal Position(H) Highest byte of Goal Position RW -
	ADDR_MX_MOVING_SPEED        = 32                            # (0X20) Moving Speed(L) Lowest byte of Moving Speed RW -
																# (0X21) Moving Speed(H) Highest byte of Moving Speed RW -
	ADDR_MX_TORQUE_LIMIT        = 34                            # (0X22) Torque Limit(L) Lowest byte of Torque Limit RW ADD14
																# (0X23) Torque Limit(H) Highest byte of Torque Limit RW ADD15
	ADDR_MX_PRESENT_POSITION    = 36                            # (0X24) Present Position(L) Lowest byte of Current Position R -
																# (0X25) Present Position(H) Highest byte of Current Position R -
	ADDR_MX_PRESENT_SPEED       = 38                            # (0X26) Present Speed(L) Lowest byte of Current Speed R -
																# (0X27) Present Speed(H) Highest byte of Current Speed R -
	ADDR_MX_PRESENT_LOAD        = 40                            # (0X28) Present Load(L) Lowest byte of Current Load R -
																# (0X29) Present Load(H) Highest byte of Current Load R -
	ADDR_MX_PRESENT_VOLTAGE     = 42                            # (0X2A) Present Voltage Current Voltage R -
	ADDR_MX_PRESENT_TEMPERATURE = 43                            # (0X2B) Present Temperature Current Temperature R -
	ADDR_MX_REGISTERED          = 44                            # (0X2C) Registered Means if Instruction is registered R 0 (0X00)
	ADDR_MX_MOVING              = 46                            # (0X2E) Moving Means if there is any movement R 0 (0X00)
	ADDR_MX_LOCK                = 47                            # (0X2F) Lock Locking EEPROM RW 0 (0X00)
	ADDR_MX_PUNCH               = 48                            # (0X30) Punch(L) Lowest byte of Punch RW 0 (0X00)
																# (0X31) Punch(H) Highest byte of Punch RW 0 (0X00)
	ADDR_MX_CURRENT             = 68                            # (0X44) Current(L) Lowest byte of Consuming Current RW 0 (0X00)
																# (0X45) Current(H) Highest byte of Consuming Current RW 0 (0X00)
	ADDR_MX_TORQUE_CTRL_MODE    = 70                            # (0X46) Torque Control Mode Enable Torque control mode on/off RW 0 (0X00)
	ADDR_MX_GOAL_TORQUE         = 71                            # (0X47) Goal Torque(L) Lowest byte of goal torque value RW 0 (0X00)
																# (0X48) Goal Torque(H) Highest byte of goal torque value RW 0 (0X00)
	ADDR_MX_GOAL_ACCELERATION   = 73                            # (0X49) Goal Acceleration Goal Acceleration RW 0 (0X00)

	# Protocol version
	PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

	# Default setting
	DXL_ID                      = 1                             # Dynamixel ID: 1
	BAUDRATE                    = 1000000                       # 1000000
	DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')     # Check which port is being used on your controller
																# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

	TORQUE_ENABLE               = 1                             # Value for enabling the torque
	TORQUE_DISABLE              = 0                             # Value for disabling the torque
	DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
	DXL_MAXIMUM_POSITION_VALUE  = 4000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
	DXL_MOVING_STATUS_THRESHOLD = 10                            # Dynamixel moving status threshold

	ESC_ASCII_VALUE             = 0x1b

	COMM_SUCCESS                = 0                             # Communication Success result value
	COMM_TX_FAIL                = -1001                         # Communication Tx Failed

	global motorValue
	scale = 0.5
	expo = 2
	expoVelocity = 4
	velocity = 0
	velocityKMH = 0
	targetAngle = 0

	@staticmethod
	def initArduino(self):
		self.arduino = serial.Serial(
		   port='/dev/ttyACM2',
		 baudrate=115200,
		 parity=serial.PARITY_ODD,
		 stopbits=serial.STOPBITS_ONE,
		 bytesize=serial.EIGHTBITS,
		 writeTimeout = 10
		)

		print('arduino setting up')

		if self.arduino.isOpen() == False:
			self.arduino.open()

		self.arduino.write(b'A')
		
		return self.arduino

	@staticmethod
	def initDynamixel(self):

		# DIFFERENT MODES
		# JOIN Mode
		#       MOVING SPEED 0-1023 (0.114 rpm) (1023 - 117.07 rpm)
		#       GOAL POSITION 0-4095 (0.088 degree)
		# MULTI-TURN Mode
		#       MOVING SPEED 0-1023
		#       GOAL POSITION -28672 - 28672 (0.088 * RESOLUTION DIVIDER)
		# WHEEL Mode
		#       MOVING SPEED 0-2047 (1024-2047 is CW, 0-1023 is CCW) 10th bit flips direction
		#       NO GOAL POSITION

		# Initialize PortHandler Structs
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		port_num = dynamixel.portHandler(DEVICENAME)

		# Initialize PacketHandler Structs
		dynamixel.packetHandler()

		index = 0
		dxl_comm_result = COMM_TX_FAIL                              # Communication result
		dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

		dxl_error = 0                                               # Dynamixel error
		dxl_present_position = 0                                    # Present position

		# Open port
		if dynamixel.openPort(port_num):
			print("Succeeded to open the port!")
		else:
			print("Failed to open the port!")
			print("Press any key to terminate...")
			#getch()
			#quit()

		# Set port baudrate
		if dynamixel.setBaudRate(port_num, BAUDRATE):
			print("Succeeded to change the baudrate!")
		else:
			print("Failed to change the baudrate!")
			print("Press any key to terminate...")
			#getch()
			#quit()


		# Enable Dynamixel Torque
		dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
			dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
		elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
			dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
		else:
			print("Dynamixel has been successfully connected")

		# Set Speed
		dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_MOVING_SPEED, 200)
		if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
			dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
		elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
			dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
		else:
			print("Dynamixel speed has been set successfully")
			
		return dynamixel