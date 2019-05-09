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

class FusionEKF:
	"""
		Gets inputs from multiple sources and uses a Kalman Filter to estimate the state of the system
		
		The state of the system is X = [x y z xdot ydot zdot xddot yddot zddot qX qY qZ qW qXdot qYdot qZdot qWdot]
		the position and velocity of the tag in the drone's body frame. Model is assuming constant acceleration.
		
		The inputs are:
			- IMU Accelerometer data.
			- Tag position data in the drone's body frame
	"""
	
	def __init__(self, n, F, P, Htag, Himu, Rtag, Rimu, N):
		self.kalmanF   = KalmanFilter(n)
		self.n         = n
		self.F         = F
		self.P         = P
		self.Htag      = Htag
		self.Himu      = Himu
		self.Rtag      = Rtag
		self.Rimu      = Rimu
		self.N         = N
		self.timeStamp = None
		self.isInit    = False

	def initialize(self, dataPoint):
		if dataPoint.getID() == 'tagPose':
			self.timeStamp = dataPoint.getTime()
			initialState = np.matrix(np.zeros((self.n, 1)))
			initialState[0] = dataPoint.getZ().item(0)
			initialState[1] = dataPoint.getZ().item(1)
			initialState[2] = dataPoint.getZ().item(2)
			initialState[9] = dataPoint.getZ().item(3)
			initialState[10] = dataPoint.getZ().item(4)
			initialState[11] = dataPoint.getZ().item(5)
			initialState[12] = dataPoint.getZ().item(6)
			self.kalmanF.initialize(initialState, self.F, self.P, self.N)
			self.isInit = True
			print("Filter initialized")

	def process(self, dataPoint):
		if not self.isInit:
			self.initialize(dataPoint)
		else:
			# Update time
			dt = dataPoint.getTime() - self.timeStamp
			self.timeStamp = dataPoint.getTime()
			
			# Update F and Q Matrices
			self.kalmanF.updateF(dt)
			self.kalmanF.updateQ(dt)
			
			# KF Prediction Step
			self.kalmanF.predict()
			
			# KF Correction Step
			if dataPoint.getID() == 'tagPose':
				self.kalmanF.correct(dataPoint.getZ(), self.Htag, self.Rtag)
			elif dataPoint.getID() == 'imuAcc':
				# Initialize quaternion
				state = self.kalmanF.getState()
				qState = Quaternion(state.item(9), state.item(10), state.item(11), state.item(12))
				qOmega = Quaternion(-1 * dataPoint.getZ().item(3),
									-1 * dataPoint.getZ().item(4),
									-1 * dataPoint.getZ().item(5),
									0)

				dq = quatMultiply(qState, qOmega)

				measurement = np.matrix([[dataPoint.getZ().item(0)],
										 [dataPoint.getZ().item(1)],
										 [dataPoint.getZ().item(2)],
										 [0.5 * dq.x],
										 [0.5 * dq.y],
										 [0.5 * dq.z],
										 [0.5 * dq.w]])

				self.kalmanF.correct(measurement, self.Himu, self.Rimu)
			else:
				print("Bad ID")

	def resetFilter(self, P):
		self.P = P
		self.isInit = False
		print("Filter reset")

	def getState(self):
		return self.kalmanF.getState()
