#!/usr/bin/python
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
ROS Integration: Brennan Cain

"""

import numpy as np
import rospy as rp

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped
from jetyak_uav_utils.msg import ObservedState

from data_point import DataPoint
from fusion_ekf import FusionEKF
from gps_utils import GPS_utils
from quaternion import Quaternion
from quaternion import quatMultiply
from quaternion import quatInverse
from quaternion import quat2rpy

import threading

class FilterNode():

	def __init__(self,):
		rp.init_node("state")

		# Create GPS_utils instance
		self.myENU = GPS_utils()
		self.originSet = False

		# Create handle for last tag
		self.lastTag = DataPoint()

		# Set rate of publisher
		self.rate = 50

		# Number of States
		n = 24

		# Initial State Transition Matrix
		F = np.asmatrix(np.eye(n))

		# Initial Process Matrix
		P = np.asmatrix(1.0e3 * np.eye(n))

		# Transition Matrix for Tag measurements
		self.Htag = np.matrix(np.zeros((4, n)))
		self.Htag[0:3,   0:3] = np.matrix(-1 * np.eye(3))
		self.Htag[0:3, 14:17] = np.matrix(np.eye(3))
		self.Htag[0:3, 20:23] = np.matrix(-1 * np.eye(3))
		self.Htag[3,      19] =  1
		self.Htag[3,      23] = -1

		# Covariance Matrix for Tag measurements
		self.Rtag = np.asmatrix(1.0e-6 * np.eye(4))

		# Transition Matrix for Attitude measurements
		self.Hatt = np.matrix(np.zeros((4, n)))
		self.Hatt[0:4, 6:10] = np.matrix(np.eye(4))

		# Covariance Matrix for Attitude measurements
		self.Ratt = np.asmatrix(1.0e-9 * np.eye(4))

		# Transition Matrix for IMU measurements
		self.Himu = np.matrix(np.zeros((4, n)))
		self.Himu[0:4, 10:14] = np.matrix(np.eye(4))

		# Covariance Matrix for IMU measurements
		self.Rimu = np.asmatrix(1.0e-6 * np.eye(4))

		# Transition Matrix for Drone velocity measurements
		self.HvelD = np.matrix(np.zeros((3, n)))
		self.HvelD[0:3, 3:6] = np.matrix(np.eye(3))

		# Covariance Matrix for Drone velocity measurements
		self.RvelD = np.asmatrix(1.0e-5 * np.eye(3))

		# Transition Matrix for Drone GPS measurements
		self.HgpsD = np.matrix(np.zeros((3, n)))
		self.HgpsD[0:3, 0:3] = np.matrix(np.eye(3))

		# Covariance Matrix for Drone GPS measurements
		self.RgpsD = np.matrix(np.zeros((3, 3)))
		self.RgpsD[0:3, 0:3] = np.asmatrix(1.0e-3 * np.eye(3))
		#self.RgpsD[0:2, 0:2] = np.asmatrix(1.0e-1 * np.eye(2))
		#self.RgpsD[2, 2] = 1.0e-5

		# Transition Matrix for Jetyak GPS measurements
		self.HgpsJ = np.matrix(np.zeros((3, n)))
		self.HgpsJ[0:3, 14:17] = np.matrix(np.eye(3))

		# Covariance Matrix for Jetyak GPS measurements
		self.RgpsJ = np.asmatrix(1.0e-3 * np.eye(3))
		self.RgpsJ[2, 2] = 1.0e-1

		# Transition Matrix for Jetyak GPS heading
		self.HhdgJ = np.matrix(np.zeros((1, n)))
		self.HhdgJ[0, 19] = 1

		# Covariance Matrix for Jetyak GPS heading
		self.RhdgJ = np.matrix(1.0e-12)

		# Process Noise Level
		N = 1.0e-7

		# Initialize Kalman Filter
		self.fusionF = FusionEKF(F, P, N, self.rate)

		# Set up Subscribers
		self.dGPS_sub = rp.Subscriber("/dji_sdk/gps_position", NavSatFix, self.dGPS_callback)
		self.dAtti_sub = rp.Subscriber("/dji_sdk/attitude", QuaternionStamped, self.dAtti_callback)
		self.dIMU_sub = rp.Subscriber("/dji_sdk/imu", Imu, self.dIMU_callback)
		self.dVel_sub = rp.Subscriber("/dji_sdk/velocity", Vector3Stamped, self.dVel_callback)
		self.jGPS_sub = rp.Subscriber("/jetyak2/global_position/global", NavSatFix, self.jGPS_callback)
		self.jCompass_sub = rp.Subscriber("/jetyak2/global_position/compass_hdg", Float64, self.jCompass_callback)
		self.tag_sub = rp.Subscriber("/jetyak_uav_vision/tag_pose", PoseStamped, self.tag_callback)

		# Set up Publisher
		self.state_pub = rp.Publisher("/jetyak_uav_vision/state", ObservedState, queue_size = 1)

		# Create thread for publisher
		t = threading.Thread(target=self.statePublisher)
		t.start()

		rp.spin()			

	def dGPS_callback(self, msg):
		if self.originSet:
			enu = self.myENU.geo2enu(msg.latitude, msg.longitude, msg.altitude)

			dGPSpoint = DataPoint()
			dGPSpoint.setID('dgps')
			dGPSpoint.setZ(np.matrix([[enu.item(0)],[enu.item(1)],[enu.item(2)]]))
			self.fusionF.process(dGPSpoint, self.HgpsD, self.RgpsD)
		else:
			self.myENU.setENUorigin(msg.latitude, msg.longitude, msg.altitude)
			self.originSet = True

	def dAtti_callback(self, msg):
		dAttiPoint = DataPoint()
		dAttiPoint.setID('atti')
		dAttiPoint.setZ(np.matrix([[msg.quaternion.x],
								   [msg.quaternion.y],
								   [msg.quaternion.z],
								   [msg.quaternion.w]]))
		self.fusionF.process(dAttiPoint, self.Hatt, self.Ratt)

	def dIMU_callback(self, msg):
		dIMUPoint = DataPoint()
		dIMUPoint.setID('imu')
		dIMUPoint.setZ(np.matrix([[msg.angular_velocity.x],
								  [msg.angular_velocity.y],
								  [msg.angular_velocity.z]]))
		self.fusionF.process(dIMUPoint, self.Himu, self.Rimu)
	
	def dVel_callback(self, msg):
		dVelPoint = DataPoint()
		dVelPoint.setID('dvel')
		dVelPoint.setZ(np.matrix([[msg.vector.x],
								  [msg.vector.y],
								  [msg.vector.z]]))
		self.fusionF.process(dVelPoint, self.HvelD, self.RvelD)

	def jGPS_callback(self, msg):
		if self.originSet:
			enu = self.myENU.geo2enu(msg.latitude, msg.longitude, msg.altitude)

			jGPSpoint = DataPoint()
			jGPSpoint.setID('jgps')
			jGPSpoint.setZ(np.matrix([[enu.item(0)],[enu.item(1)],[enu.item(2)]]))
			self.fusionF.process(jGPSpoint, self.HgpsJ, self.RgpsJ)

	def jCompass_callback(self, msg):
		jHeadingPoint = DataPoint()
		jHeadingPoint.setID('jhdg')
		if msg.data < 270:
			jHeadingPoint.setZ(np.matrix([np.deg2rad(90 - msg.data)]))
		else:
			jHeadingPoint.setZ(np.matrix([np.deg2rad(450 - msg.data)]))
		self.fusionF.process(jHeadingPoint, self.HhdgJ, self.RhdgJ)

	def tag_callback(self, msg):
		tagPoint = DataPoint()
		tagPoint.setID('tag')
		tagPoint.setZ(np.matrix([[msg.pose.position.x],
								 [msg.pose.position.y],
								 [msg.pose.position.z],
								 [msg.pose.orientation.x],
								 [msg.pose.orientation.y],
								 [msg.pose.orientation.z],
								 [msg.pose.orientation.w]]))
		tagPoint.setTime(msg.header.stamp.to_sec())
		if self.tagFilter(tagPoint):
			self.fusionF.process(tagPoint, self.Htag, self.Rtag)
			self.lastTag = tagPoint
	
	def statePublisher(self):
		r = rp.Rate(self.rate)
		while not rp.is_shutdown():
			X = self.fusionF.getState()

			if X != None:
				q = Quaternion(X.item(6), X.item(7), X.item(8), X.item(9))
				dq = Quaternion(X.item(10), X.item(11), X.item(12), X.item(13))

				omega = quatMultiply(quatInverse(q), dq)
				rpy = quat2rpy(q)

				stateMsg = ObservedState()
				stateMsg.header.stamp = rp.Time.now()
				stateMsg.header.frame_id = 'local_ENU'
			
				stateMsg.drone_p.x = X.item(0)
				stateMsg.drone_p.y = X.item(1)
				stateMsg.drone_p.z = X.item(2)

				stateMsg.drone_pdot.x = X.item(3)
				stateMsg.drone_pdot.y = X.item(4)
				stateMsg.drone_pdot.z = X.item(5)

				stateMsg.drone_q.x = rpy[0]
				stateMsg.drone_q.y = rpy[1]
				stateMsg.drone_q.z = rpy[2]

				stateMsg.drone_qdot.x = 2 * omega.x
				stateMsg.drone_qdot.y = 2 * omega.y
				stateMsg.drone_qdot.z = 2 * omega.z

				stateMsg.boat_p.x = X.item(14) - X.item(20)
				stateMsg.boat_p.y = X.item(15) - X.item(21)
				stateMsg.boat_p.z = X.item(16) - X.item(22)

				stateMsg.boat_pdot.x = X.item(17)
				stateMsg.boat_pdot.y = X.item(18)
				stateMsg.boat_pdot.z = 0

				stateMsg.heading = X.item(19) - X.item(23)

				stateMsg.origin.x = self.myENU.latZero
				stateMsg.origin.y = self.myENU.lonZero
				stateMsg.origin.z = self.myENU.hgtZero

				self.state_pub.publish(stateMsg)
			
			r.sleep()
	
	def tagFilter(self, newTag):
		if (self.lastTag.getTime() == None):
			return True
		else:
			dt = newTag.getTime() - self.lastTag.getTime()
			vX = (newTag.getZ().item(0) - self.lastTag.getZ().item(0)) / dt
			vY = (newTag.getZ().item(1) - self.lastTag.getZ().item(1)) / dt
			vZ = (newTag.getZ().item(2) - self.lastTag.getZ().item(2)) / dt

			v = np.sqrt(pow(vX, 2) + pow(vY, 2) + pow(vZ, 2))
		
			if v < 5.0:
				return True
			else:
				return False

# Start Node
filtered = FilterNode()
