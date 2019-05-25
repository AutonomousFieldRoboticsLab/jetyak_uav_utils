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
from data_point import DataPoint

from quaternion import Quaternion
from quaternion import quatMultiply
from quaternion import quatInverse

class FusionEKF:
	"""
		Gets inputs from multiple sources and uses a Kalman Filter to estimate the state of the system

		The state of the system is: 
			X = {pD, dotpD, attD, dotattD, pJ, dotpJ, headJ, GSPoffsetJ, GPSbiasD}
		
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
		self.X[6:10]   = np.nan  # Drone's attitude
		self.X[14:17]  = np.nan  # Jetyak's position
		self.X[19:23]  = np.nan  # Jetyak's heading and GPS offset

	def initialize(self, dataPoint):
		if dataPoint.getID() == 'dgps':
			self.X[0] = dataPoint.getZ().item(0)
			self.X[1] = dataPoint.getZ().item(1)
			self.X[2] = dataPoint.getZ().item(2)
		elif dataPoint.getID() == 'jgps':
			self.X[14] = dataPoint.getZ().item(0)
			self.X[15] = dataPoint.getZ().item(1)
			self.X[16] = dataPoint.getZ().item(2)
		elif dataPoint.getID() == 'atti':
			self.X[6] = dataPoint.getZ().item(0)
			self.X[7] = dataPoint.getZ().item(1)
			self.X[8] = dataPoint.getZ().item(2)
			self.X[9] = dataPoint.getZ().item(3)
		elif dataPoint.getID() == 'jhdg':
			self.X[19] = dataPoint.getZ().item(0)
		elif dataPoint.getID() == 'tag':
			if (not (np.isnan(self.X[0]) or 
					 np.isnan(self.X[6]) or
					 np.isnan(self.X[14]) or
					 np.isnan(self.X[19]))):

				q = Quaternion(self.X[6], self.X[7], self.X[8], self.X[9])

				tagPos = Quaternion(dataPoint.getZ().item(0),
									dataPoint.getZ().item(1),
									dataPoint.getZ().item(2),
									0)
				
				posWq = quatMultiply(quatMultiply(q, tagPos), quatInverse(q))

				self.X[20] = self.X[14] - self.X[0] - posWq.x
				self.X[21] = self.X[15] - self.X[1] - posWq.y
				self.X[22] = self.X[16] - self.X[2] - posWq.z

				self.kalmanF.initialize(self.X, self.F, self.P, self.N)
				self.kalmanF.updateF(self.dt)
				self.kalmanF.updateQ()
				self.isInit = True
				
				print 'Colocalization filter initialized'

	def process(self, dataPoint, H, R):
		if not self.isInit:
			self.initialize(dataPoint)
		else:
			# KF Correction Step
			if dataPoint.getID() == 'tag':
				q = Quaternion(self.X.item(6),
							   self.X.item(7),
							   self.X.item(8),
							   self.X.item(9))

				tagPos = Quaternion(dataPoint.getZ().item(0),
									dataPoint.getZ().item(1),
									dataPoint.getZ().item(2),
									0)

				posWq = quatMultiply(quatMultiply(q, tagPos), quatInverse(q))
				posW = np.matrix([[posWq.x],
								  [posWq.y],
								  [posWq.z]])

				self.kalmanF.correct(posW, H, R)
			elif dataPoint.getID() == 'imu':
				q = Quaternion(self.X.item(6),
							   self.X.item(7),
							   self.X.item(8),
							   self.X.item(9))

				qOmega = Quaternion(dataPoint.getZ().item(0),
									dataPoint.getZ().item(1),
									dataPoint.getZ().item(2),
									0)

				dq = quatMultiply(q, qOmega)

				dqm = np.matrix([[0.5 * dq.x],
								 [0.5 * dq.y],
								 [0.5 * dq.z],
								 [0.5 * dq.w]])
					
				self.kalmanF.correct(dqm, H, R)
			else:
				self.kalmanF.correct(dataPoint.getZ(), H, R)
			
			self.X = self.kalmanF.getState()

	def getState(self):
		if self.isInit:
			self.kalmanF.predict()
			return self.X
		else:
			return None