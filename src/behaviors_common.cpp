#include "jetyak_uav_utils/behaviors.h"
/** @file behaviors_common.cpp
 *
 * Implements:
 * 	createPID
 * 	resetPID
 * 	setPID
 * 	importParams
 * 	assignPublishers
 * 	assignServiceClients
 * 	assignServiceServers
 * 	assignSubscribers
 */

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
	getP(ns, "vGain", vGain);

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
	getP(ns, "land_vel_mag", velMag);
	land_.velThreshSqr = velMag * velMag;
	getP(ns, "land_x_low", land_.lowX);
	getP(ns, "land_x_high", land_.highX);
	getP(ns, "land_y_low", land_.lowY);
	getP(ns, "land_y_high", land_.highY);
	getP(ns, "land_z_low", land_.lowZ);
	getP(ns, "land_z_high", land_.highZ);
	getP(ns, "land_angle_thresh", land_.angleThresh);

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

void Behaviors::uploadParams(std::string ns_param)
{
	std::string ns;
	// If it ends in / or is only a ~, it is good to use
	if (ns_param.length() == 0 or ns_param.back() == '/' or ns_param.compare("~") == 0)
		ns = ns_param;
	else	// if it needs a / seperator, add it
		ns = ns_param + "/";

	/**********************
	 * LANDING PARAMETERS *
	 *********************/
	ros::param::set(ns + "land_x_kp", land_.kp.x);
	ros::param::set(ns + "land_y_kp", land_.kp.y);
	ros::param::set(ns + "land_z_kp", land_.kp.z);
	ros::param::set(ns + "land_w_kp", land_.kp.w);

	ros::param::set(ns + "land_x_kd", land_.kd.x);
	ros::param::set(ns + "land_y_kd", land_.kd.y);
	ros::param::set(ns + "land_z_kd", land_.kd.z);
	ros::param::set(ns + "land_w_kd", land_.kd.w);

	ros::param::set(ns + "land_x_ki", land_.ki.x);
	ros::param::set(ns + "land_y_ki", land_.ki.y);
	ros::param::set(ns + "land_z_ki", land_.ki.z);
	ros::param::set(ns + "land_w_ki", land_.ki.w);

	ros::param::set(ns + "land_x", land_.goal_pose.x);
	ros::param::set(ns + "land_y", land_.goal_pose.y);
	ros::param::set(ns + "land_z", land_.goal_pose.z);
	ros::param::set(ns + "land_w", land_.goal_pose.w);

	/**********************
	 * TAKEOFF PARAMETERS *
	 *********************/
	ros::param::set(ns + "takeoff_height", takeoff_.height);
	ros::param::set(ns + "takeoff_threshold", takeoff_.threshold);

	/**********************
	 * FOLLOW PARAMETERS *
	 *********************/
	ros::param::set(ns + "follow_x_kp", follow_.kp.x);
	ros::param::set(ns + "follow_y_kp", follow_.kp.y);
	ros::param::set(ns + "follow_z_kp", follow_.kp.z);
	ros::param::set(ns + "follow_w_kp", follow_.kp.w);

	ros::param::set(ns + "follow_x_kd", follow_.kd.x);
	ros::param::set(ns + "follow_y_kd", follow_.kd.y);
	ros::param::set(ns + "follow_z_kd", follow_.kd.z);
	ros::param::set(ns + "follow_w_kd", follow_.kd.w);

	ros::param::set(ns + "follow_x_ki", follow_.ki.x);
	ros::param::set(ns + "follow_y_ki", follow_.ki.y);
	ros::param::set(ns + "follow_z_ki", follow_.ki.z);
	ros::param::set(ns + "follow_w_ki", follow_.ki.w);

	ros::param::set(ns + "follow_x", follow_.goal_pose.x);
	ros::param::set(ns + "follow_y", follow_.goal_pose.y);
	ros::param::set(ns + "follow_z", follow_.goal_pose.z);
	ros::param::set(ns + "follow_w", follow_.goal_pose.w);

	/**********************
	 * RETURN PARAMETERS *
	 *********************/
	double settleRadius;
	ros::param::set(ns + "return_gotoHeight", return_.gotoHeight);
	ros::param::set(ns + "return_finalHeight", return_.finalHeight);
	ros::param::set(ns + "return_downRadius", return_.downRadius);
	ros::param::set(ns + "return_settleRadius", settleRadius);
	ros::param::set(ns + "return_tagTime", return_.tagTime);
	ros::param::set(ns + "return_tagLossThresh", return_.tagLossThresh);
	return_.settleRadiusSquared = settleRadius * settleRadius;
	ros::param::set(ns + "return_maxVel", return_.maxVel);
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
	setBoatNSService_ = nh.advertiseService("setBoatNS", &Behaviors::setBoatNSCallback, this);
	setFollowPIDService_ = nh.advertiseService("setFollowPID", &Behaviors::setFollowPIDCallback, this);
	setLandPIDService_ = nh.advertiseService("setLandPID", &Behaviors::setLandPIDCallback, this);
	setFollowPosition_ = nh.advertiseService("setFollowPosition", &Behaviors::setFollowPositionCallback, this);
	setLandPosition_ = nh.advertiseService("setLandPosition", &Behaviors::setLandPositionCallback, this);
	setTakeoffParams_ = nh.advertiseService("setTakeoffParams", &Behaviors::setTakeoffParamsCallback, this);
	setReturnParams_ = nh.advertiseService("setReturnParams", &Behaviors::setReturnParamsCallback, this);
	setLandParams_ = nh.advertiseService("setLandParams", &Behaviors::setLandParamsCallback, this);
}

void Behaviors::assignSubscribers()
{
	tagPoseSub_ = nh.subscribe("/jetyak_uav_vision/filtered_tag", 1, &Behaviors::tagPoseCallback, this);
	uavHeightSub_ = nh.subscribe("/dji_sdk/height_above_takeoff", 1, &Behaviors::uavHeightCallback, this);
	uavGPSSub_ = nh.subscribe("/dji_sdk/gps_position", 1, &Behaviors::uavGPSCallback, this);
	boatGPSSub_ = nh.subscribe("/jetyak2/global_position/global", 1, &Behaviors::boatGPSCallback, this);
	uavAttSub_ = nh.subscribe("/dji_sdk/attitude", 1, &Behaviors::uavAttitudeCallback, this);
	boatIMUSub_ = nh.subscribe("/jetyak2/imu/data", 1, &Behaviors::boatIMUCallback, this);
	uavVelSub = nh.subscribe("/dji_sdk/velocity", 1, &Behaviors::uavVelCallback, this);
	extCmdSub_ = nh.subscribe("extCommand", 1, &Behaviors::extCmdCallback, this);
}

double Behaviors::scaleConstant(double C, double e)
{
	/*
	 * double THRESHOLD = .1;
	 * return error < THRESHOLD : 0 ? c;											 // piecewise
	 */
	return abs(bsc_common::util::fastSigmoid(e)) * C;	// scale with fast sigmoid
}
