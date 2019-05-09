"""
MIT License

Copyright(c) 2018 Brennan Cain and Michail Kalaitzakis(Unmanned Systems and Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author: Michail Kalaitzakis

"""

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
