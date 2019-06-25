#!/usr/bin/python

"""
Provides high level control for the drone to:
* takeoff from the boat,
* follow the boat,
* leave to do pattern,
* return to the boat,
* and land back on the boat
"""
import sys

import rospy
from std_msgs.msg import UInt8,UInt16
from std_srvs.srv import Trigger
from jetyak_uav_utils.srv import SetString
from jetyak_uav_utils.msg import ObservedState

"""
Callbacks and methods
"""
def state_cb(msg):
	global lastState,inCorner,timeCornerFlip
	if(lastState==None):
		lastState=msg
	else:
		Dw = (lastState.heading-msg.heading)
		Dt = (lastState.header.stamp.to_sec()-msg.header.stamp.to_sec())
		dw = Dw/Dt
		tSinceFlip = msg.header.stamp.to_sec()-timeCornerFlip
		if abs(dw) > .25 and tSinceFlip>1:
			inCorner=True
			timeCornerFlip=msg.header.stamp.to_sec()
		elif abs(dw) < .25 and tSinceFlip>1:
			inCorner=False
			timeCornerFlip=msg.header.stamp.to_sec()

def mode_cb(msg):
	global mode
	mode=msg.data

def wp_count_cb(msg):
	global wpLeft
	wpLeft = msg.data

def wait_for_corner():
	rate = rospy.Rate(100)
	while(ros::ok()):
		if(inCorner):
			return True
		rate.sleep()

def wait_leave_corner():
	rate = rospy.Rate(100)
	while(ros::ok()):
		if(not inCorner):
			return True
		rate.sleep()

def wait_for_follow():
	rate = rospy.Rate(100)
	while(ros::ok()):
		if(mode == 1):
			return True
		rate.sleep()

def wait_for_waypoint_gen():
	rate = rospy.Rate(100)
	while(ros::ok():
		if(wpLeft != 0):
			return True
		rate.sleep()

def wait_for_waypoint_fin():

	rate = rospy.Rate(100)
	while(ros::ok()):
		if(wpLeft == 0):
			return True
		rate.sleep()

def wait(sec):
	start = rospy.Time.now().to_sec()
	rate = rospy.Rate(100)
	while(ros::ok() and rospy.Time.now().to_sec()-start<sec):
		rate.sleep()
	return True

def takeoff():
	modeClient("takeoff")
def leave():
	modeClient("leave")
def comeback():#ha reserved keyword
	modeClient("return")
def land():
	modeClient("land")
def spiral():
	spiralClient()
	
"""
Begin serial section
"""
lastState= None
inCorner = False
timeCornerFlip = 0
wpLeft=0
mode =0

#initialize ros
rospy.init_node("mission_controller")

ms = rospy.Subscriber("/jetyak_uav_utils/behavior_mode",UInt8,mode,queue_size=1)
ss = rospy.Subscriber("/jetyak_uav_vision/state",ObservedState,state,queue_size=1)
ss = rospy.Subscriber("/jetyak_uav_vision/wp_remaining",UInt16,state,queue_size=1)
rospy.wait_for_service("/jetyak_uav_utils/setMode")
modeClient=rospy.ServiceProxy("/jetyak_uav_utils/setMode", SetString)
rospy.wait_for_service("/jetyak_uav_utils/create_spiral")
spiralClient=rospy.ServiceProxy("/jetyak_uav_utils/create_spiral", Trigger)

wait_for_corner()
wait_leave_corner()
takeoff()
wait_for_follow()
wait(10)
leave()
spiral()
wait_for_waypoint_gen()
wait_for_waypoint_fin()
comeback()
wait_for_follow()
wait_for_corner()
wait_leave_corner()
land()

