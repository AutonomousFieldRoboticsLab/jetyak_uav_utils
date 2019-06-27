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
from math import pi
from time import sleep

"""
Callbacks and methods
"""

#dist from w1 to w2
def ang_dist(w1, w2):
	dW = w2-w1
	if(dW>=pi):
		dW-=2*pi
	if(dW<-pi):
		dW+=2*pi
	return dW

def state_cb(msg):
	global lastState,inCorner,wList
	if(lastState!=None):
		dW = ang_dist(msg.heading,lastState.heading)
		#print(dW)
		wList.append(dW)
		wList=wList[-75:]
		#print(abs(sum(wList)))
		if(dW>.1):
			inCorner=True
		else:
			inCorner=False
	lastState=msg

def mode_cb(msg):
	global mode
	mode=msg.data

def wp_count_cb(msg):
	global wpLeft
	wpLeft = msg.data

def wait_for_corner():
	rate = rospy.Rate(100)
	while(not rospy.is_shutdown()):
		if(inCorner):
			return True
		rate.sleep()

def wait_leave_corner():
	rate = rospy.Rate(100)
	while(not rospy.is_shutdown()):
		if(not inCorner):
			return True
		rate.sleep()

def wait_for_follow():
	rate = rospy.Rate(100)
	while(not rospy.is_shutdown()):
		if(mode == 1):
			return True
		rate.sleep()

def wait_for_waypoint_gen():
	rate = rospy.Rate(100)
	while(not rospy.is_shutdown()):
		if(wpLeft != 0):
			return True
		rate.sleep()

def wait_for_waypoint_fin():

	rate = rospy.Rate(100)
	while(not rospy.is_shutdown()):
		if(wpLeft == 0):
			return True
		rate.sleep()

def wait(sec):
	start = rospy.Time.now().to_sec()
	rate = rospy.Rate(100)
	while(not rospy.is_shutdown() and rospy.Time.now().to_sec()-start<sec):
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
wList = []

#initialize ros
rospy.init_node("mission_controller")

ms = rospy.Subscriber("/jetyak_uav_utils/behavior_mode",UInt8,mode_cb,queue_size=1)
ss = rospy.Subscriber("/jetyak_uav_vision/state",ObservedState,state_cb,queue_size=1)
ss = rospy.Subscriber("/jetyak_uav_utils/wp_remaining",UInt16,wp_count_cb,queue_size=1)
rospy.wait_for_service("/jetyak_uav_utils/setMode")
modeClient=rospy.ServiceProxy("/jetyak_uav_utils/setMode", SetString)
rospy.wait_for_service("/jetyak_uav_utils/create_spiral")
spiralClient=rospy.ServiceProxy("/jetyak_uav_utils/create_spiral", Trigger)
"""
print("Waiting for corner")
wait_for_corner()
print("Waiting leave corner")
wait_leave_corner()
print("takeoff")
takeoff()
print("follow?")
wait_for_follow()
print("Following for 30...")
wait(30)
print("Leave")
leave()
print("Spiral")
spiral()
print("waiting for waypoint population")
wait_for_waypoint_gen()
print("wait for no more waypoints")
wait_for_waypoint_fin()
print("return")
comeback()
print("wait for follow")
wait_for_follow()
print("Wait for corner")
wait_for_corner()
print("wait to leave corner")
wait_leave_corner()
print("land")
land()
"""
while(not rospy.is_shutdown()):
	print(inCorner)
	sleep(1)
