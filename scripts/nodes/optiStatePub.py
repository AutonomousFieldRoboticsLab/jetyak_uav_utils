#!/usr/bin/python

import numpy as np
import rospy as rp

from tf.transformations import *
from sensor_msgs.msg import NavSatFix,Joy,Imu
from geometry_msgs.msg import QuaternionStamped,Vector3Stamped,PoseStamped
from jetyak_uav_utils.msg import ObservedState
import math

class StatePub():
	def __init__(self,):

		rp.init_node("lqr_test")

		self.optiSub = rp.Subscriber("vrpn_client_node/Matrice/pose",PoseStamped,self.optiCB,queue_size=1)
		self.boatOptiSub = rp.Subscriber("vrpn_client_node/MasterTag/pose",PoseStamped,self.boatCB,queue_size=1)
		self.velSub=rp.Subscriber("/dji_sdk/velocity",Vector3Stamped,self.velCB,queue_size=1)
		self.imuSub=rp.Subscriber("/dji_sdk/imu",Imu,self.imuCB,queue_size=1)
		#self.rcSub=rp.Subscriber("/dji_sdk/rc",Joy,self.rcCB,queue_size=1)
		self.statePub=rp.Publisher("/jetyak_uav_vision/state",ObservedState,queue_size=1)
		self.state = ObservedState()
		self.lastOpti=[0,0,0,0]
		self.lastOptiT=rp.Time.now().to_sec()
		rp.spin()

	def publish(self,):
		#self.lastOpti= [self.state.drone_p.x, self.state.drone_p.y, self.state.drone_p.z, self.lastOptiT]

		self.state.header.stamp = rp.Time.now()
		self.statePub.publish(self.state)
		
	#def rcCB(self,msg):
	#	self.publish()
	def imuCB(self,msg):
		self.state.drone_qdot.x=msg.angular_velocity.x
		self.state.drone_qdot.y=msg.angular_velocity.y
		self.state.drone_qdot.z=msg.angular_velocity.z

	
	def velCB(self,msg):
		self.state.drone_pdot.x=msg.vector.x
		self.state.drone_pdot.y=msg.vector.y
		self.state.drone_pdot.z=msg.vector.z
		self.publish()

	def boatCB(self,msg):
		self.state.boat_p.x=msg.pose.position.x
		self.state.boat_p.y=msg.pose.position.y
		self.state.boat_p.z=msg.pose.position.z

		(r,p,y) =euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
		self.state.heading=y


	def optiCB(self,msg):
		(r,p,y) =euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
		self.state.drone_q.x=r
		self.state.drone_q.y=p
		self.state.drone_q.z=y
		self.yaw=y
		"""
		dt=msg.header.stamp.to_sec()-self.lastOpti[3]
		self.lastOptiT=msg.header.stamp.to_sec()
		dx=(msg.pose.position.x-self.lastOpti[0])/dt
		dy=(msg.pose.position.y-self.lastOpti[1])/dt
		dz=(msg.pose.position.z-self.lastOpti[2])/dt

		tx=dx*math.cos(-y)-dy*math.sin(-y)
		ty=dx*math.sin(-y)+dy*math.cos(-y)
		#print("%1.8f,\t%1.8f,\t%1.8f"%(tx,ty,dt))
		
		self.state.drone_pdot.x=tx
		self.state.drone_pdot.y=ty
		self.state.drone_pdot.z=dz
		"""
		self.state.drone_p.x=msg.pose.position.x
		self.state.drone_p.y=msg.pose.position.y
		self.state.drone_p.z=msg.pose.position.z


lll = StatePub()
