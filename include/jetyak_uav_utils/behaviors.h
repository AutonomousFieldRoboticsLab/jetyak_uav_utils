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
	ros::Subscriber stateSub_, boatGPSSub_, boatIMUSub_, uavGPSSub_, uavAttSub_, uavHeightSub_, uavVelSub,
			extCmdSub_;
	ros::Publisher cmdPub_, modePub_;
	ros::ServiceClient propSrv_, takeoffSrv_, landSrv_, lookdownSrv_, resetKalmanSrv_, enableGimbalSrv_;
	ros::ServiceServer setModeService_, getModeService_, setBoatNSService_, setFollowPIDService_, setLandPIDService_,
			setFollowPosition_, setLandPosition_;
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
	sensor_msgs::NavSatFix uavGPS_, boatGPS_;
	geometry_msgs::QuaternionStamped uavAttitude_;
	sensor_msgs::Imu uavImu_, boatImu_;
	double uavHeight_ = 0;

	jetyak_uav_utils::ObservedState state;
	bsc_common::vel3d_t uavWorldVel_ = {0, 0, 0, 0}; // world
	bsc_common::vel3d_t uavBodyVel_ = {0, 0, 0, 0};	// world

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
		bsc_common::pose4d_t kp, kd, ki;
		bsc_common::pose4d_t goal_pose; // landing goal
		double lastSpotted;
		double heightGoal;
		double heightThresh;
		double lowX, highX, lowY, highY, lowZ, highZ;
		double velThreshSqr;
		double angleThresh;
	} land_;

	// follow specific constants
	struct
	{
		bsc_common::pose4d_t kp, kd, ki;
		bsc_common::pose4d_t goal_pose; // follow goal
		double lastSpotted;
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

	/** setBoatNSCallback
	 * Callback for the boat namespace service. This changes the topics for the
	 * boat gps and imu subscriptions.
	 *
	 * @param req string of the root NS of the boat (ex. "/jetyak1")
	 * @param res boolean indicating a success or failure
	 */
	bool setBoatNSCallback(jetyak_uav_utils::SetString::Request &req, jetyak_uav_utils::SetString::Response &res);

	/** setFollowPIDCallback
	 * Change the constants in the follow pid controller
	 *
	 * @param req 4 arrays of new constants in P,I,D order for each axis
	 * @param res boolean indicating a success or failure
	 */
	bool setFollowPIDCallback(jetyak_uav_utils::FourAxes::Request &req, jetyak_uav_utils::FourAxes::Response &res);

	/** setLandPIDCallback
	 * Change the constants in the land pid controller
	 *
	 * @param req 4 arrays of new constants in P,I,D order for each axis
	 * @param res boolean indicating a success or failure
	 */
	bool setLandPIDCallback(jetyak_uav_utils::FourAxes::Request &req, jetyak_uav_utils::FourAxes::Response &res);

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

	/** uavGPSCallback
	 * Listens for updates from the UAVs GPS
	 *
	 * @param msg gets the global position of the UAV
	 */
	void uavGPSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

	/** uavHeightCallback
	 * Listens for updates on the UAVs height
	 *
	 * @param msg gets the ground relative altitude of the UAV
	 */
	void uavHeightCallback(const std_msgs::Float32::ConstPtr &msg);

	/** boatGPSCallback
	 * Listens for updates from the boat's GPS
	 *
	 * @param msg gets the global position of the boat
	 */
	void boatGPSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

	/** uavAttitudeCallback
	 * Listens for updates in the UAVs Attitude
	 *
	 * @param msg gets the global attitude of the UAV
	 */
	void uavAttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

	/** uavImuCallback
	 * Listens for updates in the UAVs IMU
	 *
	 * @param msg gets the Imu reading of the UAV
	 */
	void uavImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

	/** uavVelVallback
	 * Receive the velocity of the UAv in the world frame
	 *
	 * @param msg gets the UAV velocity reading
	 */
	void uavVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

	/** boatIMUCallback
	 * Listens for updates in the boat's IMU
	 *
	 * @param msg gets the IMU reading of the boat
	 */
	void boatIMUCallback(const sensor_msgs::Imu::ConstPtr &msg);

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

	/** uploadParams
	 * Upload params to the namespaces's ros param server
	 *
	 * @param ns name of the namespace
	 */
	void uploadParams(std::string ns = "");

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
