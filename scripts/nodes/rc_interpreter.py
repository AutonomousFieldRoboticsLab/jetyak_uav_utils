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


Interprets signals from a telemetry radio and calls rosservices based on defined rules.
Author: Brennan Cain

"""

import rospy as rp
from mavros_msgs.msg import RCIn
from jetyak_uav_utils.srv import SetString
from std_srvs.srv import Trigger


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
		self.parts = parts-1

		# ros stuff
		rp.init_node("rc_interpreter")
		self.joySub = rp.Subscriber("/jetyak2/rc/in", RCIn, self.joyCallback)
		rp.wait_for_service("/jetyak_uav_utils/setMode")
		self.modeClient=rp.ServiceProxy("/jetyak_uav_utils/setMode", SetString)
		rp.wait_for_service("/jetyak_uav_utils/create_spiral")
		self.spiralClient=rp.ServiceProxy("/jetyak_uav_utils/create_spiral", Trigger)
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
		if part==5:
			self.spiralClient()
			print("spiralGen")

if __name__ == "__main__":
	rc=RC_Interpreter(900, 2000, 9) # min, max, sections
