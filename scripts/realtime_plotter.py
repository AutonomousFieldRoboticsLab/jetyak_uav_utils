#!/usr/bin/python
from matplotlib import pyplot as plt
import rospy as rp

from jetyak_uav_utils.msg import ObservedState

from math import pi,sin,cos,atan2

class RealtimePlotter():
	def plot(self,state):
		t = state.header.stamp.to_sec()
		if(t-self.last<1.0/self.rate):
			return
		self.last = t
		self.drone_p.append((t,state.drone_p.x,state.drone_p.y))
		self.boat_p.append((t,state.boat_p.x,state.boat_p.y))
		while(t-self.boat_p[0][0]>self.frame):
			self.boat_p.pop(0)
			self.drone_p.pop(0)
		plt.clf()
		plt.plot([x[1] for x in self.boat_p], [x[2] for x in self.boat_p],"b")
		plt.plot([x[1] for x in self.drone_p], [x[2] for x in self.drone_p],"g")
		plt.gca().set_aspect('equal',adjustable='box')
		plt.gca().set(xlim=(-2,12),ylim=(-7,27))
		plt.pause(.001)

	def __init__(self,):
		rp.init_node("plotter")
		self.drone_p = []
		self.boat_p = []
		self.frame = 36
		self.rate=1.0
		self.last=0
		self.statePub=rp.Subscriber("/jetyak_uav_vision/state",ObservedState,self.plot)
		rp.spin()

lll = RealtimePlotter()
