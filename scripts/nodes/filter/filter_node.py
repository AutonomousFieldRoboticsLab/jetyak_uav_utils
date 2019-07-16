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

		# Set rate of publisher
		self.rate = 50.0

		# Number of states
		n = 15

		# Initial State Transition Matrix
		F = np.asmatrix(np.eye(n))

		# Initial Process Matrix
		P = np.asmatrix(1.0e3 * np.eye(n))

		# Process Noise Level
		N = 1e-3

		# Setup Sensors
		self.updateSensorR = True
		self.tagS, self.velDS, self.gpsDS, self.gpsJS = setupSensors(n)

		# Attitude handles
		self.droneAtti     = None
		self.droneARates   = None
		self.jetyakHeading = None

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

		# Set up Services
		self.resetFilter_srv = rp.Service("/jetyak_uav_vision/resetFilter", Trigger, self.resetFilter)

		# Set up Publisher
		self.state_pub = rp.Publisher("/jetyak_uav_vision/state", ObservedState, queue_size = 1)

		# Create thread for publisher
		t = threading.Thread(target=self.statePublisher)
		t.start()

		rp.spin()

	def resetFilter(self, srv):
		self.fusionF.resetFilter()

		return TriggerResponse(True, "Filter reset")

	def checkAngle(self, q):
		while q < -np.pi:
			q += 2 * np.pi

		while q >= np.pi:
			q -= 2 * np.pi
		
		return q

	def dGPS_callback(self, msg):
		if self.originSet:
			enu = self.myENU.geo2enu(msg.latitude, msg.longitude, msg.altitude)

			self.gpsDS.setZ(np.matrix([[enu.item(0)],[enu.item(1)],[enu.item(2)]]))
			
			r, P = self.fusionF.process(self.gpsDS)
			
			if self.updateSensorR:
				self.gpsDS.updateR(r, P)
		else:
			self.myENU.setENUorigin(msg.latitude, msg.longitude, msg.altitude)
			self.originSet = True
	
	def dVel_callback(self, msg):
		self.velDS.setZ(np.matrix([[msg.vector.x],
								  [msg.vector.y],
								  [msg.vector.z]]))
		
		r, P = self.fusionF.process(self.velDS)
		
		if self.updateSensorR:
			self.velDS.updateR(r, P)

	def jGPS_callback(self, msg):
		if self.originSet:
			enu = self.myENU.geo2enu(msg.latitude, msg.longitude, msg.altitude)

			self.gpsJS.setZ(np.matrix([[enu.item(0)],[enu.item(1)],[enu.item(2)]]))
			
			r, P = self.fusionF.process(self.gpsJS)
			
			if self.updateSensorR:
				self.gpsJS.updateR(r, P)

	def tag_callback(self, msg):
		if (self.droneAtti is not None) and (self.jetyakHeading is not None):
			tagPos = Quaternion(msg.pose.position.x,
								msg.pose.position.y,
								msg.pose.position.z,
								0)
			
			qTag = Quaternion(msg.pose.orientation.x,
							  msg.pose.orientation.y,
							  msg.pose.orientation.z,
							  msg.pose.orientation.w)
			
			posWq = quatMultiply(quatMultiply(self.droneAtti, tagPos), quatInverse(self.droneAtti))				
			rpyT = quat2rpy(qTag)
			rpyD = quat2rpy(self.droneAtti)

			offq = self.checkAngle(self.jetyakHeading - rpyT[2] - rpyD[2])
					
			self.tagS.setZ(np.matrix([[posWq.x],
									  [posWq.y],
									  [posWq.z],
									  [offq]]))
			
			r, P = self.fusionF.process(self.tagS)
			
			if self.updateSensorR:
					self.tagS.updateR(r, P)
	
	def dAtti_callback(self, msg):
		self.droneAtti = Quaternion(msg.quaternion.x,
									msg.quaternion.y,
									msg.quaternion.z,
									msg.quaternion.w)

	def dIMU_callback(self, msg):
		self.droneARates = np.matrix([[msg.angular_velocity.x],
								  	  [msg.angular_velocity.y],
								  	  [msg.angular_velocity.z]])

	def jCompass_callback(self, msg):
		if msg.data < 270:
			self.jetyakHeading = np.matrix([np.deg2rad(90 - msg.data)])
		else:
			self.jetyakHeading = np.matrix([np.deg2rad(450 - msg.data)])
	
	def statePublisher(self):
		r = rp.Rate(self.rate)
		while not rp.is_shutdown():
			X = self.fusionF.getState()

			if not (X is None):
				rpy = quat2rpy(self.droneAtti)

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

				stateMsg.drone_qdot.x = self.droneARates[0]
				stateMsg.drone_qdot.y = self.droneARates[1]
				stateMsg.drone_qdot.z = self.droneARates[2]

				stateMsg.boat_p.x = X.item(6) - X.item(11)
				stateMsg.boat_p.y = X.item(7) - X.item(12)
				stateMsg.boat_p.z = X.item(8) - X.item(13)

				stateMsg.boat_pdot.x = X.item(9)
				stateMsg.boat_pdot.y = X.item(10)
				stateMsg.boat_pdot.z = 0

				stateMsg.heading = self.checkAngle(self.jetyakHeading - X.item(14))

				stateMsg.gps_offset.x = X.item(11)
				stateMsg.gps_offset.y = X.item(12)
				stateMsg.gps_offset.z = X.item(13)

				stateMsg.heading_offset = X.item(14)

				stateMsg.origin.x = self.myENU.latZero
				stateMsg.origin.y = self.myENU.lonZero
				stateMsg.origin.z = self.myENU.hgtZero

				self.state_pub.publish(stateMsg)
			
			r.sleep()

# Start Node
filtered = FilterNode()
