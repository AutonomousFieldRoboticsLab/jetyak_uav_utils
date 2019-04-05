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

#include "jetyak_uav_utils/jetyak_uav.h"

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
	double rcVelocityMultiplier;

	bool isM100;

private:
	/** buildFlag
	 * Builds a DJI_SDK flag using the simpler JETYAK_UAV flag
	 *
	 * @param flag Enumerated integer defined in jetyak_uav.h
	 *
	 * @return DJI_SDK flag
	 */
	uint8_t buildFlag(JETYAK_UAV::Flag flag);
};

#endif
