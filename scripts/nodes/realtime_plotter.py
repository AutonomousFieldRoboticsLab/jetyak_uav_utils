#!/usr/bin/python
from matplotlib import pyplot as plt
import rospy as rp

from jetyak_uav_utils.msg import ObservedState

from math import pi,sin,cos,atan2

class RealtimePlotter():
		
	
	def plot(self,state):
		t = state.header.stamp.to_sec()
		self.drone_p.append((t,state.drone_p.x,state.drone_p.y))
		self.boat_p.append((t,state.boat_p.x,state.boat_p.y))
		while(t-self.boat_p[0,0]>self.frame):
			self.boat_p.pop(0)
			self.boat_p.pop(0)
		plt.plot([x[1] for x in self.boat_p], [x[2] for x in self.boat_p],".b")
		plt.plot([x[1] for x in self.boat_p], [x[2] for x in self.boat_p],".g")
		plt.pause(.001)

	def __init__(self,):
		self.drone_p = []
		self.boat_p = []
		self.frame = 36
		self.statePub=rp.Publisher("/jetyak_uav_vision/state",ObservedState,self.plot)
		rp.spin()

lll = StatePub()
