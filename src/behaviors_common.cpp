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
 * This file implements general functions of the behaviors node.
 * 
 * Author: Brennan Cain
 */

#include "jetyak_uav_utils/behaviors.h"

void Behaviors::downloadParams(std::string ns_param)
{
	/** getP
	 * Gathers the parameter and executes a ROS_WARN if unable to gather
	 *
	 * @param ns namespace of the parameter
	 * @param name name of the parameter
	 *
	 * @return None
	 */
	auto getP = [](std::string ns, std::string name, double &param) {
		if (!ros::param::get(ns + name, param))
			ROS_WARN("FAILED: %s", name.c_str());
	};

	std::string ns = ns_param;

	/******************
	 * MISC PARAMETERS *
	 ******************/
	if (!ros::param::get(ns + "integral_size", integral_size))
		ROS_WARN("FAILED: %s", "integral_size");

	if (!ros::param::get(ns + "k_matrix_path", k_matrix_path))
		ROS_WARN("FAILED: %s", "k_matrix_path");

	getP(ns, "reset_kalman_threshold", resetFilterTimeThresh);

	getP(ns, "land_x", land_.goal_pose.x);
	getP(ns, "land_y", land_.goal_pose.y);
	getP(ns, "land_z", land_.goal_pose.z);
	getP(ns, "land_w", land_.goal_pose.w);

	double velMag;
	getP(ns, "land_vel_mag", velMag);
	land_.velThreshSqr = velMag * velMag;
	getP(ns, "land_xThresh", land_.xThresh);
	getP(ns, "land_yThresh", land_.yThresh);
	getP(ns, "land_zThresh", land_.zThresh);
	getP(ns, "land_angle_thresh", land_.angleThresh);

	/**********************
	 * TAKEOFF PARAMETERS *
	 *********************/
	getP(ns, "takeoff_height", takeoff_.height);
	getP(ns, "takeoff_threshold", takeoff_.threshold);

	/*********************
	 * FOLLOW PARAMETERS *
	 ********************/
	getP(ns, "follow_x", follow_.goal_pose.x);
	getP(ns, "follow_y", follow_.goal_pose.y);
	getP(ns, "follow_z", follow_.goal_pose.z);
	getP(ns, "follow_w", follow_.goal_pose.w);

	/**********************
	 * LANDING PARAMETERS *
	 *********************/
	double settleRadius;
	getP(ns, "return_gotoHeight", return_.gotoHeight);
	getP(ns, "return_finalHeight", return_.finalHeight);
	getP(ns, "return_downRadius", return_.downRadius);
	getP(ns, "return_settleRadius", settleRadius);
	getP(ns, "return_tagTime", return_.tagTime);
	getP(ns, "return_tagLossThresh", return_.tagLossThresh);
	return_.settleRadiusSquared = settleRadius * settleRadius;
}

void Behaviors::assignPublishers()
{
	cmdPub_ = nh.advertise<sensor_msgs::Joy>("behavior_cmd", 1);
	modePub_ = nh.advertise<std_msgs::UInt8>("behavior_mode", 1);
}

void Behaviors::assignServiceClients()
{
	propSrv_ = nh.serviceClient<std_srvs::SetBool>("prop_enable");
	takeoffSrv_ = nh.serviceClient<std_srvs::Trigger>("takeoff");
	landSrv_ = nh.serviceClient<std_srvs::Trigger>("land");
	enableGimbalSrv_ = nh.serviceClient<std_srvs::SetBool>("setGimbalTracking");
	lookdownSrv_ = nh.serviceClient<std_srvs::Trigger>("/jetyak_uav_vision/facedown");
	resetKalmanSrv_ = nh.serviceClient<std_srvs::Trigger>("/jetyak_uav_vision/reset_filter");
}

void Behaviors::assignServiceServers()
{
	setModeService_ = nh.advertiseService("setMode", &Behaviors::setModeCallback, this);
	getModeService_ = nh.advertiseService("getMode", &Behaviors::getModeCallback, this);
	setFollowPosition_ = nh.advertiseService("setFollowPosition", &Behaviors::setFollowPositionCallback, this);
	setLandPosition_ = nh.advertiseService("setLandPosition", &Behaviors::setLandPositionCallback, this);
}

void Behaviors::assignSubscribers()
{
	stateSub_ = nh.subscribe("/jetyak_uav_vision/state", 1, &Behaviors::stateCallback, this);
	tagSub_ = nh.subscribe("/jetyak_uav_vision/tag_pose", 1, &Behaviors::tagCallback, this);
	extCmdSub_ = nh.subscribe("extCommand", 1, &Behaviors::extCmdCallback, this);
}

Eigen::Vector4d Behaviors::boat_to_drone(Eigen::Vector4d pos)
{
	//vertical setpoint
	double vDiff = pos(2) + state.boat_p.z - state.drone_p.z;
	double wDiff = pos(3) + state.heading - state.drone_q.z;
	if(wDiff>=M_PI)
		wDiff-=2*M_PI;
	else if(wDiff<=-M_PI)
		wDiff+=2*M_PI;

	//get setpoint in world frame
	Eigen::Vector2d hGoal(pos(0), pos(1));														//setpoint relative to boat frame
	hGoal = bsc_common::util::rotation_matrix(state.heading) * hGoal; //relative to boat in world frame
	Eigen::Vector2d hBoat(state.boat_p.x, state.boat_p.y);						//boat position in world
	hGoal = hBoat + hGoal;																						// setpoint in world frame

	//transform to drone frame
	Eigen::Vector2d hDrone(state.drone_p.x, state.drone_p.y);						 // drone position in world
	hGoal = hGoal - hDrone;																							 //world frame vector from drone to setpoint
	hGoal = bsc_common::util::rotation_matrix(-state.drone_q.z) * hGoal; // transform into the drone frame

	Eigen::Vector4d dronePos;
	dronePos << hGoal(0), hGoal(1), vDiff, wDiff;
	
	return dronePos;
}
