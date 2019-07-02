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

from sensor import Sensor
from data_point import DataPoint
from fusion_ekf import FusionEKF
from gps_utils import GPS_utils
from quaternion import Quaternion
from quaternion import quatMultiply
from quaternion import quatInverse
from quaternion import quat2rpy
from setupSensors import setupSensors

import threading

class FilterNode():

	def __init__(self,):
		rp.init_node("state")

		# Create GPS_utils instance
		self.myENU = GPS_utils()
		self.originSet = False

		# Critical chi-squared values for P = 0.001 and different degrees of freedom
		self.chiSquared_1 = 10.828
		self.chiSquared_3 = 16.266

		# Set rate of publisher
		self.rate = 50.0

		# Number of states
		n = 24

		# Initial State Transition Matrix
		F = np.asmatrix(np.eye(n))

		# Initial Process Matrix
		P = np.asmatrix(1.0e3 * np.eye(n))

		# Process Noise Level
		N = 5.0e-3

		# Setup Sensors
		self.tagS, self.attiS, self.imuS, self.velDS, self.gpsDS, self.gpsJS, self.hdgJS = setupSensors(n)

		# Initialize Kalman Filter
		self.fusionF = FusionEKF(F, P, N, self.rate)

		# Set up Subscribers
		self.dGPS_sub = rp.Subscriber("/dji_sdk/gps_position", NavSatFix, self.dGPS_callback, queue_size = 1)
		self.dAtti_sub = rp.Subscriber("/dji_sdk/attitude", QuaternionStamped, self.dAtti_callback, queue_size = 1)
		self.dIMU_sub = rp.Subscriber("/dji_sdk/imu", Imu, self.dIMU_callback, queue_size = 1)
		self.dVel_sub = rp.Subscriber("/dji_sdk/velocity", Vector3Stamped, self.dVel_callback, queue_size = 1)
		self.jGPS_sub = rp.Subscriber("/jetyak2/global_position/global", NavSatFix, self.jGPS_callback, queue_size = 1)
		self.jCompass_sub = rp.Subscriber("/jetyak2/global_position/compass_hdg", Float64, self.jCompass_callback, queue_size = 1)
		self.tag_sub = rp.Subscriber("/jetyak_uav_vision/tag_pose", PoseStamped, self.tag_callback, queue_size = 1)

		# Set up Publisher
		self.state_pub = rp.Publisher("/jetyak_uav_vision/state", ObservedState, queue_size = 1)

		# Create thread for publisher
		t = threading.Thread(target=self.statePublisher)
		t.start()

		rp.spin()			

	def dGPS_callback(self, msg):
		if self.originSet:
			enu = self.myENU.geo2enu(msg.latitude, msg.longitude, msg.altitude)

			self.gpsDS.setZ(np.matrix([[enu.item(0)],[enu.item(1)],[enu.item(2)]]))
			r, P = self.fusionF.process(self.gpsDS)
			self.gpsDS.updateR(r, P)
		else:
			self.myENU.setENUorigin(msg.latitude, msg.longitude, msg.altitude)
			self.originSet = True

	def dAtti_callback(self, msg):
		self.attiS.setZ(np.matrix([[msg.quaternion.x],
								   [msg.quaternion.y],
								   [msg.quaternion.z],
								   [msg.quaternion.w]]))
		r, P = self.fusionF.process(self.attiS)
		self.attiS.updateR(r, P)

	def dIMU_callback(self, msg):
		self.imuS.setZ(np.matrix([[msg.angular_velocity.x],
								  [msg.angular_velocity.y],
								  [msg.angular_velocity.z]]))
		r, P = self.fusionF.process(self.imuS)
		self.imuS.updateR(r, P)
	
	def dVel_callback(self, msg):
		self.velDS.setZ(np.matrix([[msg.vector.x],
								  [msg.vector.y],
								  [msg.vector.z]]))
		r, P = self.fusionF.process(self.velDS)
		self.velDS.updateR(r, P)

	def jGPS_callback(self, msg):
		if self.originSet:
			enu = self.myENU.geo2enu(msg.latitude, msg.longitude, msg.altitude)

			self.gpsJS.setZ(np.matrix([[enu.item(0)],[enu.item(1)],[enu.item(2)]]))
			r, P = self.fusionF.process(self.gpsJS)
			self.gpsJS.updateR(r, P)

	def jCompass_callback(self, msg):
		if msg.data < 270:
			self.hdgJS.setZ(np.matrix([np.deg2rad(90 - msg.data)]))
		else:
			self.hdgJS.setZ(np.matrix([np.deg2rad(450 - msg.data)]))
		
		r, P = self.fusionF.process(self.hdgJS)
		self.hdgJS.updateR(r, P)

	def tag_callback(self, msg):
		self.tagS.setZ(np.matrix([[msg.pose.position.x],
								 [msg.pose.position.y],
								 [msg.pose.position.z],
								 [msg.pose.orientation.x],
								 [msg.pose.orientation.y],
								 [msg.pose.orientation.z],
								 [msg.pose.orientation.w]]))
		
		r, P = self.fusionF.process(self.tagS)
		self.tagS.updateR(r, P)
	
	def statePublisher(self):
		r = rp.Rate(self.rate)
		while not rp.is_shutdown():
			X = self.fusionF.getState()

			if not (X is None):
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

# Start Node
filtered = FilterNode()