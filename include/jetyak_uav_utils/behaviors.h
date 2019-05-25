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
 * Base behavioral controller for the uav
 * Manages the high level methods of the system.
 * Sends commands to the UAV
 * implements a joystick ovveride
 * implements a safety controller for if the drone is too close to an object
 * Listens to topic to determine when to switch modes
 * 
 * Author: Brennan Cain, Michail Kalaitzakis
 */

#ifndef JETYAK_UAV_UTILS_BEHAVIORS_H_
#define JETYAK_UAV_UTILS_BEHAVIORS_H_

// C includes
#include <cmath>
#include <cstdlib>
#include <vector>

// ROS
#include <ros/ros.h>

// ROS Core includes
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Trigger.h>
#include "std_srvs/SetBool.h"

// Jetyak UAV Includes
#include "jetyak_uav_utils/ObservedState.h"
#include "jetyak_uav_utils/FourAxes.h"
#include "jetyak_uav_utils/GetString.h"
#include "jetyak_uav_utils/SetString.h"
#include "jetyak_uav_utils/jetyak_uav_utils.h"

// Lib includes
#include "../lib/bsc_common/include/lqr.h"
#include "../lib/bsc_common/include/types.h"
#include "../lib/bsc_common/include/util.h"

class Behaviors
{
private:
	/*********************************************
	 * ROS PUBLISHERS, SUBSCRIBERS, AND SERVICES
	 *********************************************/
	ros::Subscriber stateSub_, tagSub_, extCmdSub_;
	ros::Publisher cmdPub_, modePub_;
	ros::ServiceClient propSrv_, takeoffSrv_, landSrv_, lookdownSrv_, resetKalmanSrv_, enableGimbalSrv_;
	ros::ServiceServer setModeService_, getModeService_, setFollowPosition_, setLandPosition_;
	ros::NodeHandle nh;

	/**********************
	 * INSTANCE VARIABLES
	 **********************/
	int integral_size = 0;
	bsc_common::LQR *lqr_; // pid controllers
	std::string k_matrix_path;
	bool behaviorChanged_ = false;
	JETYAK_UAV_UTILS::Mode currentMode_;
	bool propellorsRunning = false;
	double resetFilterTimeThresh;

	/************************************
	 * STATE VARIABLES
	 ************************************/
	double uavHeight_ = 0;
	double lastSpotted = 0;
	jetyak_uav_utils::ObservedState state;

	/*********************************************
	 * BEHAVIOR SPECIFIC VARIABLES AND CONSTANTS
	 **********************************************/
	struct
	{
		double boatz;
		double height;
		double threshold;
	} takeoff_;

	// Land specific constants
	struct
	{
		bsc_common::pose4d_t goal_pose; // landing goal
		double heightGoal;
		double heightThresh;
		double xThresh,yThresh,zThresh;
		double velThreshSqr;
		double angleThresh;
	} land_;

	// follow specific constants
	struct
	{
		bsc_common::pose4d_t goal_pose; // follow goal
		double deadzone_radius;
	} follow_;

	// follow specific constants
	struct
	{
		bsc_common::pose4d_t kp, kd, ki;
		double gotoHeight;
		double finalHeight;
		double downRadius;
		double settleRadiusSquared = 1;
		double tagTime;
		double tagLossThresh;
		double maxVel;
		enum Stage
		{
			UP,
			OVER,
			DOWN,
			SETTLE
		} stage;
	} return_;

	struct
	{
		sensor_msgs::Joy input;
	} leave_;
	/*********************
	 * SERVICE CALLBACKS
	 *********************/
	/** setModeCallback
	 * Callback for the mode service. Changes the current mode.
	 *
	 * @param req mode to set
	 * @param res successful
	 */
	bool setModeCallback(jetyak_uav_utils::SetString::Request &req, jetyak_uav_utils::SetString::Response &res);

	/** getModeCallback
	 * Callback for the mode service. Changes the current mode.
	 *
	 * @param req empty
	 * @param res contains the mode
	 */
	bool getModeCallback(jetyak_uav_utils::GetString::Request &req, jetyak_uav_utils::GetString::Response &res);

	/** setFollowPositionCallback
	 * Change the constants in the follow position
	 *
	 * @param req 4 arrays of a single value for position
	 * @param res boolean indicating a success or failure
	 */
	bool setFollowPositionCallback(jetyak_uav_utils::FourAxes::Request &req, jetyak_uav_utils::FourAxes::Response &res);

	/** setFollowPositionCallback
	 * Change the constants in the land position
	 *
	 * @param req 4 arrays of a single value for position
	 * @param res boolean indicating a success or failure
	 */
	bool setLandPositionCallback(jetyak_uav_utils::FourAxes::Request &req, jetyak_uav_utils::FourAxes::Response &res);

	/****************************
	 * SUBSCRIPTION CALLBACKS
	 *****************************/

	/** stateCallback
	 * *
	 */
	void stateCallback(const jetyak_uav_utils::ObservedState::ConstPtr &msg);

	/** tagCallback
	 * 
	 */
	void tagCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

	/** extCallback
	 * listens for external commands, pass through when in LEAVE mode.
	 * 
	 * @param msg gets a command using the rpty flag setup
	 */
	void extCmdCallback(const sensor_msgs::Joy::ConstPtr &msg);

	/******************************
	 * BEHAVIOR METHODS
	 ******************************/
	/** takeoffBehavior
	 * Take off and change to follow mode if successful
	 */
	void takeoffBehavior();
	/** followBehavior
	 * Follow the boat using tags (later fused sensors)
	 */
	void followBehavior();

	/** leaveBehavior
	 * Pass through commands from an external controller to pilot
	 */
	void leaveBehavior();

	/** returnBehavior
	 * Safely return to the boat
	 */
	void returnBehavior();

	/** landBehavior
	 * Safely land on the boat
	 */
	void landBehavior();

	/** rideBehavior
	 * Safely ride on the boat
	 */
	void rideBehavior();

	/** hoverBehavior
	 * Safely hover
	 */
	void hoverBehavior();

	/*****************
	 * Common Methods
	 *****************/

	/** downloadParams
	 * Download params from the namespaces's ros param server
	 *
	 * @param ns name of the namespace
	 */
	void downloadParams(std::string ns = "");

	/** boat_to_drone
	 * Converts a 4d posein the boat FLU frame to the UAV FLU frame. Useful in translating boat relative setpoints to the drone's frame.
	 * @param pos position in the boat frame
	 * @param yaw offset relative to boat frame
	 * */
	Eigen::Vector4d boat_to_drone(Eigen::Vector4d pos);

	/***********************
	 * Constructor Methods
	 **********************/

	/** assignPublishers
	 * Initialize the ROS Publishers
	 */
	void assignPublishers();

	/** assignService Clients
	 * Initialize the ROS Service clients
	 */
	void assignServiceClients();

	/** assignServiceServers
	 * Initialize the ROS service servers
	 */
	void assignServiceServers();

	/** assignSubscribers
	 * Initialize the ROS Subscribers
	 */
	void assignSubscribers();

public:
	/** Constructor
	 * Start up the Controller Node
	 * Create publishers, subscribers, services
	 *
	 * @param nh node handler
	 */
	Behaviors(ros::NodeHandle &nh);

	~Behaviors();

	/** doBehaviorAction
	 * calls the behavior designated by the mode service
	 */
	void doBehaviorAction();
};

#endif
