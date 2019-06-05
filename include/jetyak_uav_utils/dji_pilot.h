/**
MIT License

Copyright (c) 2018 Brennan Cain and Michail Kalaitzakis (Unmanned Systems and Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
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
*/

/**
 * This node provides a robust interface to the DJI hardware, allowing abstraction for higher level controllers.
 * 
 * Author: Michail Kalaitzakis, Brennan Cain
 */

#ifndef DJI_PILOT_H
#define DJI_PILOT_H

// System includes
#include <string>

// ROS
#include <ros/ros.h>

// ROS includes
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include "std_srvs/SetBool.h"

// DJI SDK includes
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/dji_sdk.h>

#include "jetyak_uav_utils/jetyak_uav_utils.h"

class dji_pilot
{
public:
	/** dji_pilot
	 * Constructs an instance of this node
	 */
	dji_pilot(ros::NodeHandle &nh);

	~dji_pilot();

	/** publishCommand
	 * Pushes a command through that was previously added through a service or subscriber
	 */
	void publishCommand();
	
	/** checkRCconnection
	 * Returns true if the RC is properly communicating with the SDK
	*/
	bool checkRCconnection();

protected:
	// ROS Subscribers
	ros::Subscriber extCmdSub;
	ros::Subscriber djiRCSub;

	// ROS Publishers
	ros::Publisher controlPub;

	// ROS Services
	ros::ServiceClient sdkCtrlAuthorityServ;
	ros::ServiceClient armServ, taskServ;
	ros::ServiceServer propServServer, takeoffServServer, landServServer;

	// Callback functions

	/** extCallback
	 * Allows other ros nodes to publish to this node. This callback is for a higher level controller.
	 */
	void extCallback(const sensor_msgs::Joy::ConstPtr &msg);

	/** rcCallback
	 * Allows the RC inputs to be received. This topic has a higher priority than extCallback.
	 */
	void rcCallback(const sensor_msgs::Joy::ConstPtr &msg);

	// Service Servers
	/** propServCallback
	 * Handles starting the propellors
	 */
	bool propServCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

	/** landServCallback
	 * Handles landing using the built in service of DJI_SDK
	 */
	bool landServCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	/** takeoffServCallback
	 * Handles takeoff using the built in service of DJI_SDK
	 */
	bool takeoffServCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	// Functions
	/** setupRCCallbak
	 * Sets up platform specific constants.
	 */
	void setupRCCallback();

	/** requestControl
	 * calls the DJI SDK to ask for control.
	 *
	 * @param requestFlag Flag to use to request control
	 *
	 * @return True if control is granted
	 */
	bool requestControl(int requestFlag);

	/** loadPilotParameters
	 * Loads parameters needed by this node from the rosparam server.
	 */
	void loadPilotParameters();

	/** adaptiveClipping
	 * Clips the values for velocities based on the flag that is passed in.
	 *
	 * @param msg Joy message in rpty convention with a DJI_SDK flag at [4]
	 *
	 * @return Joy message with clipped velocities.
	 */
	sensor_msgs::Joy adaptiveClipping(sensor_msgs::Joy msg);

	// Data
	sensor_msgs::Joy extCommand, rcCommand;
	bool autopilotOn;
	bool bypassPilot;
	uint8_t commandFlag;

	int modeFlag, pilotFlag;
	double hVelocityMaxBody, hVelocityMaxGround, hAngleRateCmdMax, hAngleCmdMax;
	double vVelocityMaxBody, vVelocityMaxGround, vPosCmdMax, vPosCmdMin, vThrustCmdMax;
	double yAngleRateMax, yAngleMax;
	double rcStickThresh;
	double rcVelocityMultiplierH, rcVelocityMultiplierV;

	bool isM100;

	// If connection to RC is lost set to panic mode
	bool panicMode;
	bool rcReceived;
	ros::Time lastRCmsg;

private:
	/** buildFlag
	 * Builds a DJI_SDK flag using the simpler JETYAK_UAV_UTILS flag
	 *
	 * @param flag Enumerated integer defined in jetyak_uav.h
	 *
	 * @return DJI_SDK flag
	 */
	uint8_t buildFlag(JETYAK_UAV_UTILS::Flag flag);
};

#endif
