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
from std_msgs.msg import UInt8
from jetyak_uav_utils.srv import SetString
from jetyak_uav_utils.msg import ObservedState



def wait_for_corner():

def wait_leave_corner():

def wait_for_return();

def wait_for_waypoint();

def takeoff():

def land():
