#!/usr/bin/python

import rospy as rp
from mavros_msgs.msg import RCIn
from jetyak_uav_utils.srv import SetString


class RC_Interpreter():

	def __init__(self, minD, maxD, parts):

		self.modeTable = ["land", "takeoff", "leave",
				  "", "hover", "",
				  "land", "follow", "return"]

		self.lastPublished = self.modeTable[0]
		# parameter stuff
		self.minD = minD
		self.maxD = maxD
		self.range = maxD-minD
		self.parts = parts

		# ros stuff
		rp.init_node("rc_interpreter")
		self.joySub = rp.Subscriber("/jetyak2/rc/in", RCIn, self.joyCallback)
		rp.wait_for_service("/jetyak_uav_utils/setMode")
		self.modeClient=rp.ServiceProxy("/jetyak_uav_utils/setMode", SetString)
		rp.spin()

	def joyCallback(self, msg):
		if(len(msg.channels) <8):
			rp.logwarn("Too few RC Channels: %i",len(msg.channels))
			return 
		part=int((float(msg.channels[7]-self.minD)/self.range)*self.parts)

		if self.modeTable[part] != self.lastPublished and self.modeTable[part] !="":
			self.lastPublished=self.modeTable[part]
			#print("Mode: %s, \tpart: %i" % (self.modeTable[part],part))
			while(not self.modeClient(self.modeTable[part])):
				print("Trying to change to %s"%self.modeTable[part])

if __name__ == "__main__":
	rc=RC_Interpreter(900, 2000, 8)
