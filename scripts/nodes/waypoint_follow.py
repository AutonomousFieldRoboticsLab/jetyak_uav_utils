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


Provides spiral and rectangular coverage patterns and waypoint following.

Author: Brennan Cain

"""
import rospy
from jetyak_uav_utils.msg import Waypoint, WaypointArray
from jetyak_uav_utils.srv import SetWaypoints,SetWaypointsResponse,Int,IntResponse
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Trigger,TriggerResponse
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from math import atan2,cos,sin,pi,sqrt
import numpy as np

def lat_lon_to_m(lat1,lon1,lat2,lon2):
	R = 6378137
	rLat1 = lat1 * pi / 180
	rLat2 = lat2 * pi / 180
	rLon1 = lon1 * pi / 180
	rLon2 = lon2 * pi / 180

	dLat = rLat2 - rLat1
	dLon = rLon2 - rLon1
	a = pow(sin(dLat / 2), 2) + cos(rLat1) * cos(rLat2) * pow(sin(dLon / 2), 2)
	return R * 2 * atan2(sqrt(a), sqrt(1 - a))

def clip(val,low,high):
	return min(high,max(val,low))


def frange(start, stop, parts):
	L = []
	step = (stop - start) / parts
	for i in range(parts):
		L.append(start + step * i)
	return L


def angle_dist(a1,a2):
	"""
	Takes 2 angles in rads. 
	diff
	normalize to (-pi,pi]
	"""
	diff=a1-a2
	while(diff>pi):
		diff-=2*pi
	while(diff<=-pi):
		diff+=2*pi
	return diff



class WaypointFollow():	
	def __init__(self,):
		self.hover_alt=10
		self.flag = 0b100  # ,yaw angle, world frame, and velocity commands
		self.wps = []
		self.corners = {1:None, 2:None, 3: None}
		self.yaw = 0
		self.lat = 0
		self.lon = 0
		self.height = 0
		self.max_speed=5
		self.in_waypoint = False
		self.time_entered_wp = 0

		rospy.init_node("waypoint_follower")
		self.cmd_pub = rospy.Publisher("/jetyak_uav_utils/extCommand", Joy, queue_size=1)
		self.yaw_sub = rospy.Subscriber("/dji_sdk/attitude",QuaternionStamped,self.att_callback,queue_size=1)
		self.gps_sub = rospy.Subscriber("/dji_sdk/gps_position", NavSatFix, self.gps_callback, queue_size=1)
		self.height_sub = rospy.Subscriber("/dji_sdk/height_above_takeoff", Float32, self.height_callback, queue_size=1)
		self.wps_service = rospy.Service("set_waypoints", SetWaypoints, self.wps_callback)
		self.spiral_srv = rospy.Service("create_spiral", Trigger, self.spiral_callback)
		self.mark_corner_srv = rospy.Service("mark_corner", Int, self.mark_corner_callback)
		self.spiral_srv = rospy.Service("create_rect", Int, self.rectangle_callback)


	def att_callback(self, msg):
		q = msg.quaternion
		orientation_list=[q.x,q.y,q.z,q.w]
		self.yaw = euler_from_quaternion(orientation_list)[-1]

	def gps_callback(self, msg):
		self.lat = msg.latitude
		self.lon = msg.longitude
		self.do_controls()

	def height_callback(self, msg):
		self.height = msg.data

	def mark_corner_callback(self, req):
		if req.data in [1, 2, 3]:
			self.corners[req.data] = (self.lat, self.lon, self.height,self.yaw)
			return IntResponse(True,"Corner set")
		else:
			return IntResponse(False,"Not a valid ID")

	
		
	def rectangle_callback(self, req):
		intermediate = 5 # TODO: Make this a parameter
		n=req.data
		if (n <= 1):
			return IntResponse(False, "number must be greater than 1")
		for i in [1, 2, 3]:
			if (self.corners[i] == None):
				return IntResponse(False, "Missing corner %i" % i)
				
		def norm(L): return math.sqrt(sum(i ** 2 for i in L))
		def add(v1, v2): return [(v1[i] + v2[i]) for i in range(len(v1))]
 		def diff(v1,v2): 
			ov= [(v2[i]-v1[i]) for i in [0,1,2]]
			ov.append(angle_dist(v2[3],v1[3]))
			return ov
		def scale(v, l): return [v[i]*l for i in range(len(v))]
			

		v12 = diff(self.corners[1],self.corners[2])
		
		v23 = diff(self.corners[2],self.corners[3])
		self.wps = []

		step = scale(v23, 1/float(n-1))
		middle = [add(self.corners[1], scale(v12, i/(intermediate+1.0)))
                    for i in range(1, intermediate+1)]

		left = [add(self.corners[1], scale(step, i)) for i in range(n)]
		right = [add(self.corners[2], scale(step, i)) for i in range(n)]
		mid = [[add(middle[j], scale(step, i)) for i in range(n)]
                    for j in range(intermediate)]
		print mid
		path = []
		for i in range(len(left)):
			if(i % 2 == 0):
				path.append(left.pop(0))
				for j in range(intermediate):
					path.append(mid[j].pop(0))
				path.append(right.pop(0))
			else:
				path.append(right.pop(0))
				for j in range(intermediate)[::-1]:
					path.append(mid[j].pop(0))
				path.append(left.pop(0))

		for p in path:
			wp = Waypoint()
			wp.lat = p[0]
			wp.lon = p[1]
			wp.alt = p[2]
			wp.heading=p[3]


			wp.radius = 1
			wp.loiter_time = 0
			self.wps.append(wp)
			
	def wps_callback(self, req):
		del self.wps[:]
		for i in req.waypoints.waypoints:
			print i
			self.wps.append(i)
		return SetWaypointsResponse(True)

	def spiral_callback(self, srv):
		self.wps = []

		wp_radius = 5
		end_radius_m = 10
		turns = 4
		altitude = 10

		#dont modify
		maxR = 1
		r = .01
		rmult = r/turns
		endradius = end_radius_m*.00001

		x = []
		y = []

		while(r < maxR):
			r = r+(1.0/r)*rmult
			t = (r/maxR)*(turns+1)*2*pi
			x.append(r*cos(t))
			y.append(r*sin(t))

		waypoints = []
		for i in range(len(x)):
			wp = Waypoint()
			wp.alt = altitude
			wp.lat = self.lat + (x[i] / maxR) * endradius
			wp.lon = self.lon + (y[i] / maxR) * endradius

			wp.radius = wp_radius
			wp.loiter_time = 0
			self.wps.append(wp)
		return TriggerResponse(True,"Spiral created")

	def do_controls(self,):
		if len(self.wps) == 0:
			return

		if not self.in_waypoint:
			self.in_waypoint = self.check_if_in_wp()
			
		#If we are still in a waypoint
		if self.in_waypoint:
			#If this is our first time noticing we are in a waypoint
			if self.time_entered_wp == 0:
				self.time_entered_wp = rospy.Time.now().to_sec()
				print("Entered waypoint")
			elif rospy.Time.now().to_sec() - self.time_entered_wp > self.wps[0].loiter_time:
				self.wps.pop(0)
				self.time_entered_wp = 0
				self.in_waypoint = False
				if(len(self.wps)==0):
					cmd = Joy()
					cmd.axes = [0,0, 0, 0, 0b010]
					#cmd.axes=[east,0,height_diff,0,self.flag]
					self.cmd_pub.publish(cmd)
					print("Mission Complete")
				else:
					print("Moving to next objective: %i left"%len(self.wps))
				return
		
		""" 
		if len(wps) >1:
			let v0 <- wps[0]
			let v1 <- wps[1]
			if(v0 dot v1 < 0):
				vg = v0
			else
				vg = interpolation(v1,v0)
		else:
			vg <- wps[0]
		"""
		vg=np.array([0,0,0])
		if(len(self.wps)>1):
			v0=np.array([0,0,0])
			v1=np.array([0,0,0])

			v0[0] = lat_lon_to_m(self.lat, self.wps[0].lon, self.lat, self.lon)
			if(self.wps[0].lon<self.lon):
				v0[0] = -v0[0]
			v0[1] = lat_lon_to_m(self.wps[0].lat, self.lon, self.lat, self.lon)
			if(self.wps[0].lat<self.lat):
				v0[1] = -v0[1]

			v1[0] = lat_lon_to_m(self.lat, self.wps[1].lon, self.lat, self.lon)
			if(self.wps[1].lon<self.lon):
				v1[0] = -v1[0]
			v1[1] = lat_lon_to_m(self.wps[1].lat, self.lon, self.lat, self.lon)
			if(self.wps[1].lat<self.lat):
				v1[1] = -v1[1]
			
			if(np.dot(v0,v1)<0):
				vg=v0
			else:
				d = v1-v0
				y = v0*(d/(np.linalg.norm(d)**2))
				vg=v0+y*(v1-v0)
		else:
			vg[0] = lat_lon_to_m(self.lat, self.wps[0].lon, self.lat, self.lon)
			if(self.wps[0].lon<self.lon):
				vg[0] = -vg[0]
			vg[1] = lat_lon_to_m(self.wps[0].lat, self.lon, self.lat, self.lon)
			if(self.wps[0].lat<self.lat):
				vg[1] = -vg[1]

		self.hover_alt = self.wps[0].alt

		height_diff=self.wps[0].alt-self.height
		
		mag= sqrt(vg[0]**2+vg[1]**2)
		if(mag>self.max_speed):
			vg[0]=vg[0]*self.max_speed/mag
			vg[1]=vg[1]*self.max_speed/mag

		cmd = Joy()

		cmd.axes = [vg[0], vg[1], height_diff, self.wps[0].heading, self.flag]
		#cmd.axes=[east,0,height_diff,0,self.flag]
		self.cmd_pub.publish(cmd)


	def check_if_in_wp(self,):
		dist = lat_lon_to_m(self.lat, self.lon, self.wps[0].lat, self.wps[0].lon)
		print(dist)
		return pow(dist, 2) + pow(self.height - self.wps[0].alt, 2) < pow(self.wps[0].radius, 2)



wpfollow = WaypointFollow()
rospy.spin()
