import rospy
from tf.transformations import *
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import NavSatFix,NavSatStatus, Imu
from geometry_msgs.msg import QuaternionStamped,PoseStamped
from math import atan2, cos, sin, pi, sqrt,asin

def latlon_from_bearing(lat1,lon1,dist,brng):
	d=dist/6378100.0
	lat2 = asin( sin(lat1)*cos(d) + cos(lat1)*sin(d)*cos(brng));
	lon2 = lon1 + atan2(sin(brng)*sin(d)*cos(lat1),cos(d)-sin(lat1)*sin(lat2));
	return lat2,lon2
def toRad(deg): 
	return deg*pi/180.0
def toDeg(rad): 
	return rad*180.0/pi

class Coloc():
	def __init__(self,):
		rospy.init_node("coloc")

		self.tagSub = rospy.Subscriber("/jetyak_uav_vision/filtered_tag", PoseStamped, self.tag_callback)

		#UAV Subs
		self.uavAttSub = rospy.Subscriber("/dji_sdk/attitude", QuaternionStamped, self.uav_att_callback)
		self.uavGpsSub = rospy.Subscriber("/dji_sdk/gps_position" , NavSatFix , self.uav_gps_callback)
		self.uavGpsHealthSub = rospy.Subscriber("/dji_sdk/gps_health" , UInt8 , self.uav_gps_health_callback)

		#boat Subs
		self.boatGpsSub = rospy.Subscriber("/jetyak2/global_position/global" , NavSatFix , self.boat_gps_callback)
		
		self.boatPub = rospy.Publisher("/boat_gps_estimate",NavSatFix, queue_size=10)
		self.uavPub = rospy.Publisher("/uav_gps_estimate",NavSatFix, queue_size=10)

		self.boat_last3=[]
		self.boat_health=0
		self.uav_last3=[]
		self.uav_health=0

	def tag_callback(self,msg):
		self.tag=[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,0]

	def uav_att_callback(self,msg):

		q = [msg.quaternion.x,msg.quaternion.y,msg.quaternion.z,msg.quaternion.w]
		q_i = quaternion_inverse(q)
		self.uav_to_boat = quaternion_multiply(quaternion_multiply(q,self.tag),q_i)
		#print("E: %.2f, N: %.3f, A: %.2f" % (self.uav_to_boat[0],self.uav_to_boat[1],self.uav_to_boat[2]))

		self.uav_boat_heading = atan2(self.uav_to_boat[0],self.uav_to_boat[1])
		while(self.uav_boat_heading>pi):
			self.uav_boat_heading = self.uav_boat_heading-2*pi
		while(self.uav_boat_heading<=-pi):
			self.uav_boat_heading = self.uav_boat_heading+2*pi

		self.boat_uav_heading = 180+self.uav_boat_heading
		while(self.boat_uav_heading>pi):
			self.boat_uav_heading = self.boat_uav_heading-2*pi
		while(self.boat_uav_heading<=-pi):
			self.boat_uav_heading = self.boat_uav_heading+2*pi

		self.dist = sqrt(pow(self.uav_to_boat[0],2)+pow(self.uav_to_boat[1],2))
	def uav_gps_health_callback(self,msg):
		if(msg.data >= 3):
			self.uav_health=0
		else:
			self.uav_health=-1

	def uav_gps_callback(self,msg):
		
		lat1 = toRad(msg.latitude)
		lon1 = toRad(msg.longitude)
		lat2,lon2 = latlon_from_bearing(lat1,lon1,self.uav_boat_heading,self.dist)
		
		self.uav_last3.append((toDeg(lat2),toDeg(lon2),rospy.get_time()))
		if(len(self.uav_last3)>3):
			self.uav_last3.pop(0)
		if(len(self.uav_last3)==3):
			d1 = [(self.uav_last3[1][0]-self.uav_last3[0][0])/(self.uav_last3[1][2]-self.uav_last3[0][2]),(self.uav_last3[1][1]-self.uav_last3[0][1])/(self.uav_last3[1][2]-self.uav_last3[0][2])]
			d2 = [(self.uav_last3[2][0]-self.uav_last3[1][0])/(self.uav_last3[2][2]-self.uav_last3[1][2]),(self.uav_last3[2][1]-self.uav_last3[1][1])/(self.uav_last3[2][2]-self.uav_last3[1][2])]
			a  = [(d2[0]-d1[0])*2/(self.uav_last3[2][2]-self.uav_last3[0][2]),(d2[1]-d1[1])*2/(self.uav_last3[2][2]-self.uav_last3[0][2])]
		#print("UAV: %.8f, %.8f"%(toDeg(lat1),toDeg(lon1)))
		#print("ASV: %.8f, %.8f"%(toDeg(lat2),toDeg(lon2)))
		#print("Acc: %.8f"%sqrt(pow(a[0],2)+pow(a[1],2)))
		nsf = NavSatFix()
		nsf.header.stamp = rospy.get_rostime()
		nsf.latitude = toDeg(lat2)
		nsf.longitude= toDeg(lon2)
		status = NavSatStatus()
		status.status = self.uav_health
		status.service= 0b11
		nsf.status = status
		self.boatPub.publish(nsf)

	def boat_gps_callback(self,msg):
		lat1 = toRad(msg.latitude)
		lon1 = toRad(msg.longitude)
		lat2,lon2 = latlon_from_bearing(lat1,lon1,self.boat_uav_heading,self.dist)
		
		self.boat_last3.append((toDeg(lat2),toDeg(lon2),rospy.get_time()))
		if(len(self.boat_last3)>3):
			self.boat_last3.pop(0)
		if(len(self.boat_last3)==3):
			d1 = [(self.boat_last3[1][0]-self.boat_last3[0][0])/(self.boat_last3[1][2]-self.boat_last3[0][2]),
				(self.boat_last3[1][1]-self.boat_last3[0][1])/(self.boat_last3[1][2]-self.boat_last3[0][2])]

			d2 = [(self.boat_last3[2][0]-self.boat_last3[1][0])/(self.boat_last3[2][2]-self.boat_last3[1][2]),
				(self.boat_last3[2][1]-self.boat_last3[1][1])/(self.boat_last3[2][2]-self.boat_last3[1][2])]

			a  = [(d2[0]-d1[0])*2/(self.boat_last3[2][2]-self.boat_last3[0][2]),
				(d2[1]-d1[1])*2/(self.boat_last3[2][2]-self.boat_last3[0][2])]
		print("ASV: %.8f, %.8f"%(toDeg(lat1),toDeg(lon1)))
		print("UAV: %.8f, %.8f"%(toDeg(lat2),toDeg(lon2)))
		print("Acc: %.8f"%sqrt(pow(a[0],2)+pow(a[1],2)))
		nsf = NavSatFix()
		nsf.header.stamp = rospy.get_rostime()
		nsf.latitude = toDeg(lat2)
		nsf.longitude= toDeg(lon2)
		nsf.altitude = -self.uav_to_boat[2]
		nsf.status = msg.status
		self.uavPub.publish(nsf)
		
coloc = Coloc()
rospy.spin()
