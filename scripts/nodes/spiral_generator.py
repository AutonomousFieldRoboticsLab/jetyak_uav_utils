import rospy
from jetyak_uav_utils.msg import Waypoint, WaypointArray
from jetyak_uav_utils.srv import SetWaypoints, SetWaypointsResponse
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from math import atan2, cos, sin, pi, sqrt

def frange(start, stop, parts):
	L=[]
	step = (stop - start) / parts
	for i in range(parts):
		L.append(start + step * i)
	return L
		
class SpiralGenerator():
	def __init__(self, wp_radius, endradius, turns, points,altitude):
		self.wp_radius = wp_radius
		self.endradius = endradius
		self.turns = turns
		self.points = points
		self.altitude = altitude

		
		self.lon=0
		self.lat=0
		
		
		rospy.init_node("spiral_generator")

		self.gps_sub = rospy.Subscriber("/dji_sdk/gps_position", NavSatFix, self.gps_callback)
		self.spiral_srv = rospy.Service("create_spiral", Trigger, self.spiral_callback)
		self.set_wps_service = rospy.ServiceProxy("set_waypoints", SetWaypoints)

	def spiral_callback(self,srv):
		maxR = 10.0

		r = 1.0
		rmult = .3

		x=[]
		y=[]

		while(r<maxR):
			r=r+(1.0/r)*rmult
			t = (r/maxR)*self.turns*2*pi
			x.append(r*cos(t))
			y.append(r*sin(t))

		waypoints=[]
		f = open("spiral_dump.csv","w+")
		for i in range(len(x)):
			wp = Waypoint()
			wp.alt = self.altitude
			wp.lat = self.lat + (x[i]/maxR)*self.endradius
			wp.lon = self.lon + (y[i]/maxR)*self.endradius

			f.write("%f,%f\n"%(wp.lat,wp.lon))
			wp.radius = 1
			wp.loiter_time = 0
			waypoints.append(wp)
		f.close()
		waypoints_srv = SetWaypoints()
		waypoints_srv.waypoints=waypoints
		self.set_wps_service(waypoints_srv)
		print("Spiral Pattern Sent")
		return True

	def gps_callback(self, msg):
		self.lat=msg.latitude
		self.lon=msg.longitude
		
		

wp_radius = .1
endradius = .00005
turns = 3
points = 30
altitude = 10

spiraler = SpiralGenerator(wp_radius, endradius, turns, points, altitude)
rospy.spin()
