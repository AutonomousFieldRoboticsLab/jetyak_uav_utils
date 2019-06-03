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


		self.optiSub = rp.Subscriber("/vrpn_client_node/Matrice/pose",PoseStamped,self.optiCB)
		self.velSub=rp.Subscriber("/dji_sdk/velocity",Vector3Stamped,self.velCB)
		self.imuSub=rp.Subscriber("/dji_sdk/imu",Imu,self.imuCB)
		self.statePub=rp.Publisher("/jetyak_uav_vision/state",ObservedState,queue_size=1)
		self.state = ObservedState()
		rp.spin()
	def publish(self,):
		self.state.header.stamp = rp.Time.now()
		self.statePub.publish(self.state)


	def imuCB(self,msg):
		self.state.drone_qdot.x=msg.angular_velocity.x
		self.state.drone_qdot.y=msg.angular_velocity.y
		self.state.drone_qdot.z=msg.angular_velocity.z
		self.publish()
	def velCB(self,msg):
		self.state.drone_pdot.x=msg.vector.x
		self.state.drone_pdot.y=msg.vector.y
		self.state.drone_pdot.z=msg.vector.z
		self.publish()
	
	def optiCB(self,msg):
		self.state.drone_p.x=msg.pose.position.x
		self.state.drone_p.y=msg.pose.position.y
		self.state.drone_p.z=msg.pose.position.z

		(r,p,y) =euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
		self.state.drone_q.x=r
		self.state.drone_q.y=p
		self.state.drone_q.z=y
		self.yaw=y
		self.publish()

lll = StatePub()
