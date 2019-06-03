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
from LQR import LQR
from jetyak_uav_utils.msg import Waypoint, WaypointArray, ObservedState
from jetyak_uav_utils.srv import SetWaypoints,SetWaypointsResponse,Int,IntResponse
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Trigger,TriggerResponse
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from math import atan2,cos,sin,pi,sqrt
from time import sleep
import numpy as np

import threading

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
	diffself.lat
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
		self.flag = 0b00  # LQR
		self.wps = []
		self.corners = {1:None, 2:None, 3: None}
		self.lqr = LQR()
		self.max_speed=5
		self.in_waypoint = False
		self.time_entered_wp = 0

		rospy.init_node("waypoint_follower")
		self.cmd_pub = rospy.Publisher("/jetyak_uav_utils/extCommand", Joy, queue_size=1)
		self.stat_sub= rospy.Subscriber("/jetyak_uav_vision/state", ObservedState, self.state_callback)
		self.wps_service = rospy.Service("set_waypoints", SetWaypoints, self.wps_callback)
		self.spiral_srv = rospy.Service("create_spiral", Trigger, self.spiral_callback)
		self.mark_corner_srv = rospy.Service("mark_corner", Int, self.mark_corner_callback)
		self.spiral_srv = rospy.Service("create_rect", Int, self.rectangle_callback)
		rospy.on_shutdown(self.die)

		self.running=True
		t=threading.Thread(target=self.do_control)
		t.start()
		rospy.spin()

	def die(self,):
		print("dying")
		self.running=False

	def mark_corner_callback(self, req):
		if req.data in [1, 2, 3]:
			self.corners[req.data] = [i for i in self.pose]
			return IntResponse(True,"Corner set")
		else:
			return IntResponse(False,"Not a valid ID")

	def state_callback(self,msg):
		velx,vely= msg.drone_pdot.x,msg.drone_pdot.y
		yaw = msg.drone_q.z
		tx = velx*cos(yaw)+vely*sin(yaw)
		ty = -velx*sin(yaw)+vely*cos(yaw)
		
		self.pose=[msg.drone_p.x,msg.drone_p.y,msg.drone_p.z,msg.drone_q.z]
		state = [0,0,0,\
						tx,tx,msg.drone_pdot.z,\
						msg.drone_q.x,msg.drone_q.y,0,\
						msg.drone_qdot.x,msg.drone_qdot.y,msg.drone_qdot.z]
		self.lqr.setState(state)

		
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
		#middle = [add(self.corners[1], scale(v12, i/(intermediate+1.0))) for i in range(1, intermediate+1)]

		left = [add(self.corners[1], scale(step, i)) for i in range(n)]
		right = [add(self.corners[2], scale(step, i)) for i in range(n)]
		#mid = [[add(middle[j], scale(step, i)) for i in range(n)] for j in range(intermediate)]

		path = []
		for i in range(len(left)):
			if(i % 2 == 0):
				path.append(left.pop(0))
				#for j in range(intermediate):
				#	path.append(mid[j].pop(0))
				path.append(right.pop(0))
			else:
				path.append(right.pop(0))
				#for j in range(intermediate)[::-1]:
				#	path.append(mid[j].pop(0))
				path.append(left.pop(0))

		for p in path:
			print(p)
			wp = Waypoint()
			wp.lon = p[0]
			wp.lat = p[1]
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

		wp_radius = .5
		turns = 4

		#dont modify
		maxR = 10
		r = .1

		x = []
		y = []

		while(r < maxR):
			r = r+(1.0/r)
			t = (r/maxR)*(turns+1)*2*pi
			x.append(r*cos(t))
			y.append(r*sin(t))

		waypoints = []
		for i in range(len(x)):
			wp = Waypoint()
			wp.alt = self.pose[2]
			wp.lat = self.pose[1] + (y[i] / maxR)
			wp.lon = self.pose[0] + (x[i] / maxR)

			wp.radius = wp_radius
			wp.loiter_time = 0
			self.wps.append(wp)
		return TriggerResponse(True,"Spiral created")

	def do_control(self,):
		print("thread started")
		while(self.running):
			sleep(1/25.0)
			if len(self.wps) != 0:
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
							cmd.axes = [0,0,0, 0, 0b01]
							self.cmd_pub.publish(cmd)
							print("Mission Complete")
							continue
						else:
							print("Moving to next objective: %i left"%len(self.wps))
							continue
		
				goal=np.array([[0] for i in range(12)])
				x = self.wps[0].lon - self.pose[0]
				y = self.wps[0].lat - self.pose[1]
				z = self.wps[0].alt - self.pose[2]
				w = self.wps[0].heading - self.pose[3]

				goal[0] = x*cos(self.pose[3])+y*sin(self.pose[3])
				goal[1] = -x*sin(self.pose[3])+y*cos(self.pose[3])
				goal[2] = z
				goal[8] = w
			
				cmd = self.lqr.getCmd(goal)
		
				cmdM = Joy()
				r = float(cmd[0])#clip(float(cmd[0]),-.1,.1)
				p = float(cmd[1])#clip(float(cmd[1]),-.1,.1)
				cmdM.axes = [r,p,float(cmd[2]),float(cmd[3]), self.flag]

				self.cmd_pub.publish(cmdM)



	def check_if_in_wp(self,):
		dist = sqrt((self.pose[0]-self.wps[0].lon)**2+(self.pose[1]-self.wps[0].lat)**2+(self.pose[2]-self.wps[0].alt)**2)
		print(dist)
		return dist < self.wps[0].radius



wpfollow = WaypointFollow()

