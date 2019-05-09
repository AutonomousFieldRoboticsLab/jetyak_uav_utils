#!/usr/bin/python

import numpy as np
import rospy as rp


class LQRNode():
    """

    """

    def __init__(self):
        rp.init_node("lqr_controller")

        # Set LQR gains
        self.xK = np.matrix([-0.2879, -0.2708, 0.5150, 0.0937])
        self.yK = np.matrix([ 0.2889,  0.2614, 0.4997, 0.0938])

        # Initialize state
        self.xState = np.matrix([0],[0],[0],[0]])
        self.yState = np.matrix([0],[0],[0],[0]])
        self.zPos = 0
        self.Orientation = 0

        # Initialize setpoint
        self.xSetpoint = np.matrix([0],[0],[0],[0]])
        self.ySetpoint = np.matrix([0],[0],[0],[0]])
        self.zSetpoint = 0
        self.orientationSetpoint = 0

        # Set input limit
        self.uLimit = 0.2

        # Set Subscribers
        # TO DO: add Kalman Filter subscriber
        
        # Set Publishers
        self.cmd_pub = rp.Publisher("")

    def getState_callback(self, msg):
        # TO DO: Change Kalman Filter msg to contain full state
    
    def clipCommand(self, u):
        if u > self.uLimit:
            u = self.uLimit
        elif u < -self.uLimit:
            u = -self.uLimit
        
        return u
    
    def sendCommand(self):
        xU = self.xK * (self.xSetpoint - self.xState)
        yU = self.yK * (self.ySetpoint - self.yState)
        # TO DO: Get LQR for z position as well
        # TO DO: Add PID implementation for yaw commands

        # Clip x-y inputs if out of bounds
        xU = self.clipCommand(xU)
        yU = self.clipCommand(yU)
