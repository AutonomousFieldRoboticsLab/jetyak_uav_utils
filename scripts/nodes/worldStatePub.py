#!/usr/bin/python

from LQR import LQR
from GPS_utils import GPS_utils

import numpy as np
import threading
from time import sleep

import rospy as rp

from tf.transformations import *
from sensor_msgs.msg import NavSatFix,Joy,Imu
from geometry_msgs.msg import QuaternionStamped,Vector3Stamped
from jetyak_uav_utils.msg import ObservedState

from math import pi,sin,cos,atan2

#36 second looping jetyak trajectory at 2m/s
def getJetyakPose(t):
	t=t%36

	if(t<10):
		x=0
		y=2*t
		xd = 0
		yd=2
		w=pi/2
		return [x,y,xd,yd,w]
	elif(t<18):
		T=t-10
		x=5-5*cos(T*pi/8)
		y=20+5*sin(T*pi/8)
		xd=5*pi*sin(pi*T/8)/8
		yd=5*pi*cos(pi*T/8)/8
		w = atan2(yd,xd)
		return [x,y,xd,yd,w]
	elif(t<28):
		T=t-18
		x=10
		y=20-2*T
		xd=0
		yd=-2
		w=-pi/2
		return [x,y,xd,yd,w]
	else:
		T=t-28
		x=5+5*cos(T*pi/8)
		y=-5*sin(T*pi/8)
		xd=-5*pi*sin(pi*T/8)/8
		yd=-5*pi*cos(pi*T/8)/8
		w = atan2(yd,xd)
		return [x,y,xd,yd,w]
		

class StatePub():
	def __init__(self,):
		self.doJetyak=True
		self.firstAtt=True
		self.firstGPS=True
		self.GPSU = GPS_utils()

		rp.init_node("lqr_test")
		self.gpsSub=rp.Subscriber("/dji_sdk/gps_position",NavSatFix,self.gpsCB)
		self.attSub=rp.Subscriber("/dji_sdk/attitude",QuaternionStamped,self.attCB)
		self.velSub=rp.Subscriber("/dji_sdk/velocity",Vector3Stamped,self.velCB)
		self.imuSub=rp.Subscriber("/dji_sdk/imu",Imu,self.imuCB)
		self.statePub=rp.Publisher("/jetyak_uav_vision/state",ObservedState,queue_size=1)
		self.state = ObservedState()
		self.startTime=rp.Time.now().to_sec()
		rp.spin()
	def publish(self,):
		self.state.header.stamp = rp.Time.now()
		x,y,xd,yd,w = getJetyakPose(rp.Time.now().to_sec()-self.startTime)
		if(self.doJetyak):
			self.state.boat_p.x=x
			self.state.boat_p.y=y
			self.state.boat_pdot.x=xd
			self.state.boat_pdot.y=yd
			self.state.heading=w
		self.statePub.publish(self.state)
	def attCB(self,msg):
		if(self.firstAtt):
			self.firstAtt=False
		(r,p,y) =euler_from_quaternion([msg.quaternion.x,msg.quaternion.y,msg.quaternion.z,msg.quaternion.w])
		self.state.drone_q.x=r
		self.state.drone_q.y=p
		self.state.drone_q.z=y
		self.yaw=y
		self.publish()

	def imuCB(self,msg):
		self.state.drone_qdot.x=msg.angular_velocity.x
		self.state.drone_qdot.y=msg.angular_velocity.y
		self.state.drone_qdot.z=msg.angular_velocity.z
		self.publish()

	def velCB(self,msg):
		if(not self.firstAtt):
			x=msg.vector.x
			y=msg.vector.y
			self.state.drone_pdot.x=msg.vector.x
			self.state.drone_pdot.y=msg.vector.y
			self.state.drone_pdot.z=msg.vector.z
			self.publish()
	
	def gpsCB(self,msg):
		if(self.firstGPS):
			self.GPSU.setENUorigin(0,0,0)
		p=self.GPSU.geo2enu(msg.latitude,msg.longitude,msg.altitude)
		self.state.drone_p.x=p[0]
		self.state.drone_p.y=p[1]
		self.state.drone_p.z=p[2]
		self.firstGPS=False
		self.publish()

lll = StatePub()
