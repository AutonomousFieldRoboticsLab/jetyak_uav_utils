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

void Behaviors::createPID(bsc_common::pose4d_t &kp, bsc_common::pose4d_t &ki, bsc_common::pose4d_t &kd)
{
	xpid_ = new bsc_common::PID(follow_.kp.x, follow_.ki.x, follow_.kd.x, integral_size);
	ypid_ = new bsc_common::PID(follow_.kp.y, follow_.ki.y, follow_.kd.y, integral_size);
	zpid_ = new bsc_common::PID(follow_.kp.z, follow_.ki.z, follow_.kd.z, integral_size);
	wpid_ = new bsc_common::PID(follow_.kp.w, follow_.ki.w, follow_.kd.w, integral_size);
}

void Behaviors::resetPID()
{
	xpid_->reset();
	ypid_->reset();
	zpid_->reset();
	wpid_->reset();
}

void Behaviors::setPID(bsc_common::pose4d_t &kp, bsc_common::pose4d_t &ki, bsc_common::pose4d_t &kd)
{
	xpid_->updateParams(kp.x, ki.x, kd.x);
	ypid_->updateParams(kp.y, ki.y, kd.y);
	zpid_->updateParams(kp.z, ki.z, kd.z);
	wpid_->updateParams(kp.w, ki.w, kd.w);
}

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

	getP(ns, "reset_kalman_threshold", resetFilterTimeThresh);

	/**********************
	 * LANDING PARAMETERS *
	 *********************/
	getP(ns, "land_x_kp", land_.kp.x);
	getP(ns, "land_y_kp", land_.kp.y);
	getP(ns, "land_z_kp", land_.kp.z);
	getP(ns, "land_w_kp", land_.kp.w);

	getP(ns, "land_x_kd", land_.kd.x);
	getP(ns, "land_y_kd", land_.kd.y);
	getP(ns, "land_z_kd", land_.kd.z);
	getP(ns, "land_w_kd", land_.kd.w);

	getP(ns, "land_x_ki", land_.ki.x);
	getP(ns, "land_y_ki", land_.ki.y);
	getP(ns, "land_z_ki", land_.ki.z);
	getP(ns, "land_w_ki", land_.ki.w);

	getP(ns, "land_x", land_.goal_pose.x);
	getP(ns, "land_y", land_.goal_pose.y);
	getP(ns, "land_z", land_.goal_pose.z);
	getP(ns, "land_w", land_.goal_pose.w);

	double velMag;
	getP(ns, "land_velMag", velMag);
	land_.velThreshSqr = velMag * velMag;
	getP(ns, "land_xTopThresh", land_.xTopThresh);
	getP(ns, "land_yTopThresh", land_.yTopThresh);
	getP(ns, "land_xBottomThresh", land_.xBottomThresh);
	getP(ns, "land_yBottomThresh", land_.yBottomThresh);
	getP(ns, "land_bottom",land_.bottom);
	getP(ns, "land_top", land_.top);
	getP(ns, "land_angleThresh", land_.angleThresh);

	/**********************
	 * TAKEOFF PARAMETERS *
	 *********************/
	getP(ns, "takeoff_height", takeoff_.height);
	getP(ns, "takeoff_threshold", takeoff_.threshold);

	/*********************
	 * FOLLOW PARAMETERS *
	 ********************/
	getP(ns, "follow_x_kp", follow_.kp.x);
	getP(ns, "follow_y_kp", follow_.kp.y);
	getP(ns, "follow_z_kp", follow_.kp.z);
	getP(ns, "follow_w_kp", follow_.kp.w);

	getP(ns, "follow_x_kd", follow_.kd.x);
	getP(ns, "follow_y_kd", follow_.kd.y);
	getP(ns, "follow_z_kd", follow_.kd.z);
	getP(ns, "follow_w_kd", follow_.kd.w);

	getP(ns, "follow_x_ki", follow_.ki.x);
	getP(ns, "follow_y_ki", follow_.ki.y);
	getP(ns, "follow_z_ki", follow_.ki.z);
	getP(ns, "follow_w_ki", follow_.ki.w);

	getP(ns, "follow_x", follow_.goal_pose.x);
	getP(ns, "follow_y", follow_.goal_pose.y);
	getP(ns, "follow_z", follow_.goal_pose.z);
	getP(ns, "follow_w", follow_.goal_pose.w);

	/*********************
	 * RETURN PARAMETERS *
	 ********************/
	double settleRadius;
	getP(ns, "return_gotoHeight", return_.gotoHeight);
	getP(ns, "return_heightThresh", return_.heightThresh);
	getP(ns, "return_finalHeight", return_.finalHeight);
	getP(ns, "return_downRadius", return_.downRadius);
	getP(ns, "return_settleRadius", settleRadius);
	getP(ns, "return_tagTime", return_.tagTime);
	getP(ns, "return_tagLossThresh", return_.tagLossThresh);
	getP(ns, "return_heightThresh", return_.heightThresh);
	return_.settleRadiusSquared = settleRadius * settleRadius;
	getP(ns, "return_settle_x", return_.goal.x);
	getP(ns, "return_settle_y", return_.goal.y);
	getP(ns, "return_settle_z", return_.goal.z);
	getP(ns, "return_settle_w", return_.goal.w);
}

void Behaviors::assignPublishers()
{
	cmdPub_ = nh.advertise<sensor_msgs::Joy>("behavior_cmd", 1);
	modePub_ = nh.advertise<std_msgs::UInt8>("behavior_mode", 1);
	gimbalCmdPub_ = nh.advertise<geometry_msgs::Vector3>("/jetyak_uav_vision/gimbal_cmd", 1);
}

void Behaviors::assignServiceClients()
{
	propSrv_ = nh.serviceClient<std_srvs::SetBool>("prop_enable");
	takeoffSrv_ = nh.serviceClient<std_srvs::Trigger>("takeoff");
	landSrv_ = nh.serviceClient<std_srvs::Trigger>("land");
	enableGimbalSrv_ = nh.serviceClient<std_srvs::SetBool>("/jetyak_uav_vision/setGimbalTracking");
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
	// Vertical setpoint
	double vDiff = pos(2) + state.boat_p.z - state.drone_p.z; // Distance from UAV to point (- if below UAV)
	double wDiff = pos(3) + state.heading - state.drone_q.z;	// Angular distance between headings (- if CW of UAV)

	// Ensure angular dist [-pi, pi)
	if (wDiff >= M_PI)
		wDiff -= 2 * M_PI;
	else if (wDiff < -M_PI)
		wDiff += 2 * M_PI;

	// Get setpoint in world frame
	Eigen::Vector2d goal_boat_body(pos(0), pos(1));																											 // Setpoint relative to boat frame
	Eigen::Vector2d goal_boat_world = bsc_common::util::rotation_matrix(state.heading) * goal_boat_body; // Relative to boat in world frame
	Eigen::Vector2d boat_world(state.boat_p.x, state.boat_p.y);																					 // Boat position in world
	Eigen::Vector2d goal_world = boat_world + goal_boat_world;																					 // Setpoint in world frame

	// Transform to drone frame
	Eigen::Vector2d drone_world(state.drone_p.x, state.drone_p.y);																						// Drone position in world
	Eigen::Vector2d goal_drone_world = goal_world - drone_world;																							// World frame vector from drone to setpoint
	Eigen::Vector2d goal_drone_body = bsc_common::util::rotation_matrix(-state.drone_q.z) * goal_drone_world; // Transform into the drone frame

	Eigen::Vector4d dronePos;
	dronePos << goal_drone_body(0), goal_drone_body(1), vDiff, wDiff;

	return dronePos;
}

Eigen::Vector2d Behaviors::gimbal_angle_cmd()
{
	double dx = state.boat_p.x - state.drone_p.x;
	double dy = state.boat_p.y - state.drone_p.y;
	double dz = state.boat_p.z - state.drone_p.z;

	double dxy = sqrt(dx * dx + dy * dy);

	double theta = atan2(dz, dxy);
	double psi = atan2(dy, dx);

	// The gimbal's frame is NED while the local frame is ENU
	psi -= M_PI / 2.0;
	psi = -psi;

	if (psi < 0)
		psi += 2 * M_PI;

	Eigen::Vector2d gimbalAngle;
	gimbalAngle << theta, psi;

	return gimbalAngle;
}
bool Behaviors::inLandThreshold()
{
	double x = state.drone_p.x-state.boat_p.x;
	double y = state.drone_p.y-state.boat_p.y;
	
	Eigen::Vector2d world_off(x,y);	
	Eigen::Vector2d boat_off = bsc_common::util::rotation_matrix(-state.heading) * world_off;

	x=boat_off(0)-land_.goal_pose.x;
	y=boat_off(1)-land_.goal_pose.y;

	double z = state.drone_p.z-state.boat_p.z-land_.goal_pose.z;

	double w = state.drone_q.z-state.heading-land_.goal_pose.w;

	if (w >= M_PI)
		w -= 2 * M_PI;
	else if (w < -M_PI)
		w += 2 * M_PI;

	double xb = land_.xBottomThresh;
	double xt = land_.xTopThresh;

	double yb = land_.yBottomThresh;
	double yt = land_.yTopThresh;

	double zb = land_.bottom; // bottom of trapezoid for finding x,y boundaries
	double zt = land_.top;

	double xl = (z - zb) * (xb - xt) / (zt - zb) - xb;
	double xh = (z - zb) * (xt - xb) / (zt - zb) + xb;

	double yl = (z - zb) * (yb - yt) / (zt - zb) - yb;
	double yh = (z - zb) * (yt - yb) / (zt - zb) + yb;

	bool inX = xl < x and x < xh;
	bool inY = yl < y and y < yh;
	bool inZ = zb < z and z < zt;
	bool inW = fabs(w) < land_.angleThresh;

	double velSqr = pow(state.drone_pdot.x-state.boat_pdot.x, 2) + pow(state.drone_pdot.y-state.boat_pdot.y, 2);
	bool inVel = velSqr < land_.velThreshSqr;
	// ROS_WARN("%1.8f,%1.8f",(pow(state.drone_pdot.x, 2) + pow(state.drone_pdot.y, 2)),land_.velThreshSqr);
	ROS_WARN("%s,%s,%s,%s,%s",inX?" true":"false",inY?" true":"false",inZ?" true":"false",inW?" true":"false",inVel?" true":"false");
	ROS_WARN("X %1.2f<%1.2f<%1.2f",xl,x,xh);
	ROS_WARN("Y %1.2f<%1.2f<%1.2f",yl,y,yh);
	ROS_WARN("Z %1.2f<%1.2f<%1.2f",zb,z,zt);
	ROS_WARN("W %1.2f<%1.2f<%1.2f",-land_.angleThresh,w,land_.angleThresh);
	ROS_WARN("V %1.2f<%1.2f",sqrt(velSqr),sqrt(land_.velThreshSqr));
	return inX and inY and inZ and inW and inVel;
}
