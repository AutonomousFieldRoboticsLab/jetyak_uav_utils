#!/usr/bin/python

import numpy as np
import rospy as rp
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from data_point import DataPoint
from fusion_ekf import FusionEKF

class FilterNode():

	def __init__(self,):
		rp.init_node("tag_pose_filter")
		self.lastTag = DataPoint()
		self.gravityConst = 9.832
		self.biasX = -0.149
		self.biasY = 0.041

		# Number of States
		n = 17

		# Initial State Transition Matrix
		F = np.asmatrix(np.eye(n))

		# Process Matrix
		self.P = np.asmatrix(1.0e4 * np.eye(n))

		# Transition Matrix for Tag measurements
		Htag = np.matrix(np.zeros((7, n)))
		Htag[0:3, 0:3] = np.matrix(np.eye(3))
		Htag[3:7, 9:13] = np.matrix(np.eye(4))

		# Covariance Matrix for Tag measurements
		Rtag = np.asmatrix(1.0e-6 * np.eye(7))

		# Transition Matrix for IMU measurements
		Himu = np.matrix(np.zeros((7, n)))
		Himu[0:3, 6:9] = np.matrix(np.eye(3))
		Himu[3:7, 13:17] = np.matrix(np.eye(4))

		# Covariance Matrix for IMU measurements
		Rimu = np.asmatrix(1.0e-3 * np.eye(7))

		# Process Noise Level
		N = 1.0

		# Initialize Kalman Filter
		self.fusionF = FusionEKF(n, F, self.P, Htag, Himu, Rtag, Rimu, N)

		# Set up Subscribers
		self.imu_sub = rp.Subscriber("/dji_sdk/imu", Imu, self.imu_callback)
		self.tag_sub = rp.Subscriber("tag_pose", PoseStamped, self.tag_callback)

		# Set up Publishers
		self.tagVel_pub = rp.Publisher("tag_velocity", Vector3Stamped, queue_size = 1)
		self.tag_pub = rp.Publisher("filtered_tag", PoseStamped, queue_size = 1)

		# Set up Service Servers
		self.reset_service = rp.Service("reset_filter", Trigger, self.reset_callback)

		rp.spin()

	def tag_callback(self, msg):
		# Prepare msg for process
		tagD = DataPoint()
		tagD.setID('tagPose')
		tagD.setZ(np.matrix([[msg.pose.position.x],
							[msg.pose.position.y],
							[msg.pose.position.z],
							[msg.pose.orientation.x],
							[msg.pose.orientation.y],
							[msg.pose.orientation.z],
							[msg.pose.orientation.w]]))
		tagD.setTime(msg.header.stamp.to_sec())

		# Process Data
		if self.checkOutliers(tagD):
			self.fusionF.process(tagD)
		
		# Get Filtered State
		if self.fusionF.isInit:
			fState = self.fusionF.getState()

			# Publish Filtered State
			pubMsg = PoseStamped()
			pubMsg.header.stamp = msg.header.stamp
			pubMsg.header.frame_id = msg.header.frame_id
			pubMsg.pose.position.x = fState.item(0)
			pubMsg.pose.position.y = fState.item(1)
			pubMsg.pose.position.z = fState.item(2)
			pubMsg.pose.orientation.x = fState.item(9)
			pubMsg.pose.orientation.y = fState.item(10)
			pubMsg.pose.orientation.z = fState.item(11)
			pubMsg.pose.orientation.w = fState.item(12)
			self.tag_pub.publish(pubMsg)

			# Publish Tag's Velocity
			velMsg = Vector3Stamped()
			velMsg.header.stamp = msg.header.stamp
			velMsg.vector.x = fState.item(3)
			velMsg.vector.y = fState.item(4)
			velMsg.vector.z = fState.item(5)
			self.tagVel_pub.publish(velMsg)		

	def imu_callback(self, msg):
		# Prepare msg for process
		imuD = DataPoint()
		imuD.setID('imuAcc')
		imuD.setZ(np.matrix([[(msg.linear_acceleration.x - self.biasX) / self.gravityConst],
							[(msg.linear_acceleration.y - self.biasY) / self.gravityConst],
							[(msg.linear_acceleration.z - self.gravityConst) / self.gravityConst],
							[msg.angular_velocity.x],
							[msg.angular_velocity.y],
							[msg.angular_velocity.z]]))
		imuD.setTime(msg.header.stamp.to_sec())

		# Process Data
		self.fusionF.process(imuD)

	def checkOutliers(self, newTag):
		if (self.lastTag.getTime() == None):
			self.lastTag = newTag
			return True
		else:
			dt = newTag.getTime() - self.lastTag.getTime()
			vX = (newTag.getZ().item(0) - self.lastTag.getZ().item(0)) / dt
			vY = (newTag.getZ().item(1) - self.lastTag.getZ().item(1)) / dt
			vZ = (newTag.getZ().item(2) - self.lastTag.getZ().item(2)) / dt

			v = np.sqrt(pow(vX, 2) + pow(vY, 2) + pow(vZ, 2))
			
			if v < 5.0:
				self.lastTag = newTag
				return True
			else:
				return False

	def reset_callback(self, req):
		print("Resetting")
		self.fusionF.resetFilter(self.P)
		successful = True
		
		return TriggerResponse(successful,"Successfully reset filter")

# Start Node
filtered = FilterNode()
