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
import numpy as np
from kalman_filter import KalmanFilter
from sensor import Sensor

from quaternion import Quaternion
from quaternion import quatMultiply
from quaternion import quatInverse
from quaternion import quat2rpy

class FusionEKF:
	"""
		Gets inputs from multiple sources and uses a Kalman Filter to estimate the state of the system

		The state of the system is: 
			X = {pD, dotpD, attD, dotattD, pJ, dotpJ, headJ, GSPoffsetJ, compassOffsetJ}
		
		The inputs are:
			- Drone's position in local ENU
			- Drone's velocity in local ENU
			- Drone's attitude
			- Drone's angular velocities
			- Jetyak's position in local ENU
			- Jetyak's heading
			- Jetyak's position in the Drone's body frame
	"""
	
	def __init__(self, F, P, N, rate):
		self.kalmanF   = KalmanFilter()
		self.F         = F
		self.P         = P
		self.N         = N
		self.n         = np.shape(F)[0]
		self.X         = np.matrix(np.zeros((self.n, 1)))
		self.dt        = 1 / rate
		self.isInit    = False		

		self.X[0:3]    = np.nan  # Drone's position
		self.X[14:17]  = np.nan  # Jetyak's position
		self.X[19:24]  = np.nan  # Jetyak's GPS and heading offset

	def initialize(self, dataPoint):
		if dataPoint.getID() == 'dgps':
			self.X[0] = dataPoint.getZ().item(0)
			self.X[1] = dataPoint.getZ().item(1)
			self.X[2] = dataPoint.getZ().item(2)
		elif dataPoint.getID() == 'jgps':
			self.X[6] = dataPoint.getZ().item(0)
			self.X[7] = dataPoint.getZ().item(1)
			self.X[8] = dataPoint.getZ().item(2)
		elif dataPoint.getID() == 'tag':
			if (not (np.isnan(self.X[0]) or 
					 np.isnan(self.X[6]))):

				self.X[11] = self.X[6] - self.X[0] - dataPoint.getZ().item(0)
				self.X[12] = self.X[7] - self.X[1] - dataPoint.getZ().item(1)
				self.X[13] = self.X[8] - self.X[2] - dataPoint.getZ().item(2)
				self.X[14] = dataPoint.getZ().item(3)

				self.kalmanF.initialize(self.X, self.F, self.P, self.N)
				self.kalmanF.updateF(self.dt)
				self.kalmanF.updateQ()
				self.kalmanF.predict()
				self.isInit = True

				print 'Colocalization filter initialized'

	def process(self, dataPoint):
		if not self.isInit:
			self.initialize(dataPoint)

			return None, None
		else:
			r = None
			P = None

			# KF Correction Step
			r, P = self.kalmanF.correct(dataPoint.getZ(), dataPoint.getH(), dataPoint.getR(), dataPoint.getChi())
			
			if r is None:
				print "A ", dataPoint.getID(), " measurement was rejected"

			self.X = self.kalmanF.getState()

			return r, P

	def getState(self):
		if self.isInit:
			self.kalmanF.predict()
			return self.X
		else:
			return None

	def resetFilter(self):
		self.isInit    = False

		self.X         = np.matrix(np.zeros((self.n, 1)))
		self.X[0:3]    = np.nan  # Drone's position
		self.X[14:17]  = np.nan  # Jetyak's position
		self.X[19:24]  = np.nan  # Jetyak's GPS and heading offset
