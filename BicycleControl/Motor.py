import serial

class Motor:

	mappedVelocity = b'\x00'
	velocity = 0
	arduino = None

	def _init_(self):
		initArduino()
	#send velocity motor value to motor (apply acceleration)
	def setVelocity(self):
	
		global arduino
		global mappedVelocity
		global velocity
		
		try:
			print("targetVelocity", velocity)
			print("mappedVelocity", mappedVelocity)
			
			#send velocity/motorValue to motor
			arduino.flushInput()
			arduino.flushOutput()
			arduino.write(mappedVelocity)
			
		#if keyboard input to stop
		except (KeyboardInterrupt):
			mappedVelocity = b'\x00'
			#send new velocity/motorValue to motor
			arduino.flushInput()
			arduino.flushOutput()
			arduino.write(mappedVelocity)
	
	#maps km/h to motor value
	def mapTargetVelocityToMotorValue(self, targetVelocity):
		
		global mappedVelocity
		global velocity

		#print('Mapping km/h to motor value.')
		if targetVelocity > 0:
			mappedVelocity = b'\x69'			#100
			velocity = 10
		if targetVelocity > 10:
			mappedVelocity = b'\x69'			#105
			velocity = 11
		if targetVelocity > 11:
			mappedVelocity = b'\x6E'			#110
			velocity = 13
		if targetVelocity > 13:
			mappedVelocity = b'\x73'			#115
			velocity = 14
		if targetVelocity > 14:
			mappedVelocity = b'\x78'			#120
			velocity = 15
		if targetVelocity > 15:
			mappedVelocity = b'\x7D'			#125
			velocity = 17
		if targetVelocity > 17:
			mappedVelocity = b'\x82'			#130
			velocity = 18
		if targetVelocity > 18:
			mappedVelocity = b'\x87'			#135
			velocity = 19
		if targetVelocity > 19:
			mappedVelocity = b'\x8C'			#140
			velocity = 20
		if targetVelocity <= 9:
			mappedVelocity = b'\x00'
			velocity = 0
			
	#initialize arduino connection
	def initArduino(self):
		
		global arduino
		
		arduino = serial.Serial(
			port='/dev/ttyACM0',
			baudrate=115200,
			parity=serial.PARITY_ODD,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			writeTimeout = 10
		)

		print('arduino setting up')

		if arduino.isOpen() == False:
			arduino.open()
			print("Arduino connection openned")

		arduino.write(b'\x00')
		print('Arduino initialized')
		
