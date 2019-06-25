#!/usr/bin/python
import sys

import rospy
from std_msgs.msg import UInt8
from jetyak_uav_utils.srv import SetString
from jetyak_uav_utils.msg import ObservedState

followingReceived=False
firstLandingTime = 0
firstLandedTime = 0
maxX=-200
minX=200
maxY=-200
minY=200
lastX=0
lastY=0

def state(msg):
	global maxX, maxY, minX, minY, lastX, lastY
	x=msg.drone_p.x
	y=msg.drone_p.y
	if(y>maxY):
		maxY=y
	if(y<minY):
		minY=y
	if(x>maxX):
		maxX=x
	if(x<minX):
		minX=x
	lastX=x
	lastY=y

def mode(msg):
	global followingReceived
	global firstLandingTime
	if msg.data==1:
		modeClient("land")
		print("Calling land")
	if(firstLandingTime==0 and msg.data==4):
		firstLandingTime=rospy.Time.now().to_sec()
		print("Landing: %f"%firstLandingTime)
	

def status(msg):
	global firstLandingTime
	global f
	global firstLandedTime
	if(msg.data==5 and firstLandedTime==0):
		firstLandedTime=rospy.Time.now().to_sec()

		f.write("%s,%f,%f,%f,%f\n"%(trial,(firstLandedTime-firstLandingTime),lastX,lastY,maxX))
		print("Python trying to die")
		rospy.signal_shutdown("Completed")



directory = sys.argv[1]
f = open(directory+".csv","a+")
trial=sys.argv[2]
rospy.init_node("land_timer")
fs = rospy.Subscriber("/dji_sdk/flight_status",UInt8,status,queue_size=1)
ms = rospy.Subscriber("/jetyak_uav_utils/behavior_mode",UInt8,mode,queue_size=1)
ss = rospy.Subscriber("/jetyak_uav_vision/state",ObservedState,state,queue_size=1)
rospy.wait_for_service("/jetyak_uav_utils/setMode")
modeClient=rospy.ServiceProxy("/jetyak_uav_utils/setMode", SetString)
rospy.spin()
