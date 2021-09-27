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

		self.dronep = [0,0,0]
		self.dronepdot = [0,0,0]
		self.droneq = [0,0,0]
		self.droneqdot = [0,0,0]

		# Set up Subscribers
		self.dGPS_sub = rp.Subscriber("/dji_sdk/gps_position", NavSatFix, self.dGPS_callback, queue_size = 1)
		self.dAtti_sub = rp.Subscriber("/dji_sdk/attitude", QuaternionStamped, self.dAtti_callback, queue_size = 1)
		self.dIMU_sub = rp.Subscriber("/dji_sdk/imu", Imu, self.dIMU_callback, queue_size = 1)
		self.dVel_sub = rp.Subscriber("/dji_sdk/velocity", Vector3Stamped, self.dVel_callback, queue_size = 1)

		# Set up Publisher
		self.state_pub = rp.Publisher("/jetyak_uav_vision/state", ObservedState, queue_size = 1)

		# Create thread for publisher
		t = threading.Thread(target=self.statePublisher)
		t.start()


	def dGPS_callback(self, msg):
		if self.originSet:
			enu = self.myENU.geo2enu(msg.latitude, msg.longitude, msg.altitude)

			self.dronep = [enu.item(0),enu.item(1),enu.item(2)]
		else:
			self.myENU.setENUorigin(msg.latitude, msg.longitude, msg.altitude)
			self.originSet = True
	
	def dVel_callback(self, msg):
		self.dronepdot = [msg.vector.x,
											msg.vector.y,
											msg.vector.z]

	def dAtti_callback(self, msg):
		self.droneq = quat2rpy(Quaternion(msg.quaternion.x,
									msg.quaternion.y,
									msg.quaternion.z,
									msg.quaternion.w))

	def dIMU_callback(self, msg):
		self.droneqdot = np.matrix([[msg.angular_velocity.x],
								  	  [msg.angular_velocity.y],
								  	  [msg.angular_velocity.z]])

	def statePublisher(self):
		r = rp.Rate(self.rate)
		while not rp.is_shutdown():
			if(self.originSet):
				stateMsg = ObservedState()
				stateMsg.header.stamp = rp.Time.now()
				stateMsg.header.frame_id = 'local_ENU'
			
				stateMsg.drone_p.x = self.dronep[0]
				stateMsg.drone_p.y = self.dronep[1]
				stateMsg.drone_p.z = self.dronep[2]

				stateMsg.drone_pdot.x = self.dronepdot[0]
				stateMsg.drone_pdot.y = self.dronepdot[1]
				stateMsg.drone_pdot.z = self.dronepdot[2]

				stateMsg.drone_q.x = self.droneq[0]
				stateMsg.drone_q.y = self.droneq[1]
				stateMsg.drone_q.z = self.droneq[2]

				stateMsg.drone_qdot.x = self.droneqdot[0]
				stateMsg.drone_qdot.y = self.droneqdot[1]
				stateMsg.drone_qdot.z = self.droneqdot[2]

				stateMsg.boat_p.x = 0
				stateMsg.boat_p.y = 0
				stateMsg.boat_p.z = 0

				stateMsg.boat_pdot.x = 0
				stateMsg.boat_pdot.y = 0
				stateMsg.boat_pdot.z = 0

				stateMsg.heading = 0

				stateMsg.gps_offset.x = 0
				stateMsg.gps_offset.y = 0
				stateMsg.gps_offset.z = 0

				stateMsg.heading_offset = 0

				stateMsg.origin.x = self.myENU.latZero
				stateMsg.origin.y = self.myENU.lonZero
				stateMsg.origin.z = self.myENU.hgtZero

				self.state_pub.publish(stateMsg)
			
			r.sleep()

# Start Node
filtered = FilterNode()
