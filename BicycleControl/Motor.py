import serial

class Motor:

	mappedVelocity = b'\x00'
	arduino = None

	def _init_(self):
		initArduino()
	#send velocity motor value to motor (apply acceleration)
	def setVelocity(self, velocity):
	
		try:
			print("targetVelocity", velocity)
			
			#send velocity/motorValue to motor
			arduino.flushInput()
			arduino.flushOutput()
			arduino.write(mappedVelocity)
			
			pubVelo.publish(velocity)
			
		#if keyboard input to stop
		except (KeyboardInterrupt):
			mappedVelocity = b'\x00'
			#send new velocity/motorValue to motor
			arduino.flushInput()
			arduino.flushOutput()
			arduino.write(mappedVelocity)
	
	#maps km/h to motor value
	def mapTargetVelocityToMotorValue(targetVelocity):

		#print('Mapping km/h to motor value.')
		if targetVelocity > 0:
			mappedVelocity = b'\x69'			#100
		if targetVelocity > 10:
			mappedVelocity = b'\x69'			#105
		if targetVelocity > 11:
			mappedVelocity = b'\x6E'			#110
		if targetVelocity > 13:
			mappedVelocity = b'\x73'			#115
		if targetVelocity > 14:
			mappedVelocity = b'\x78'			#120
		if targetVelocity > 15:
			mappedVelocity = b'\x7D'			#125
		if targetVelocity > 17:
			mappedVelocity = b'\x82'			#130
		if targetVelocity > 18:
			mappedVelocity = b'\x87'			#135
		if targetVelocity > 19:
			mappedVelocity = b'\x8C'			#140
		if targetVelocity <= 10:
			mappedVelocity = b'\x00'
			
	#initialize arduino connection
	def initArduino(self):
		arduino = serial.Serial(
		   port='/dev/ttyACM2',
		 baudrate=115200,
		 parity=serial.PARITY_ODD,
		 stopbits=serial.STOPBITS_ONE,
		 bytesize=serial.EIGHTBITS,
		 writeTimeout = 10
		)

		print('arduino setting up')

		if arduino.isOpen() == False:
			arduino.open()

		arduino.write(b'A')
		