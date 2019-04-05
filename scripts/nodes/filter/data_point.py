class DataPoint:
	"""
		Sensor measurements handle to store all the desired data in a standard way
		Handles tag position data and imu data. Imu data should be integrated to represent velocities
	"""
	
	def __init__(self):
		self.ID        = None
		self.timeStamp = None
		self.Z         = None

	def setID(self, dataID):
		self.ID = dataID

	def setZ(self, sensorMeasurement):
		self.Z = sensorMeasurement

	def setTime(self, measurementTime):
		self.timeStamp = measurementTime
	
	def getID(self):
		return self.ID

	def getZ(self):
		return self.Z

	def getTime(self):
		return self.timeStamp