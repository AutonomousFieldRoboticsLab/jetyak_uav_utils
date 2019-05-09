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
 * This file implements the service callbacks of the behaviors node
 * 
 * Author: Brennan Cain
 */

#include "jetyak_uav_utils/behaviors.h"

bool Behaviors::setModeCallback(jetyak_uav_utils::SetString::Request &req, jetyak_uav_utils::SetString::Response &res)
{
	for (int i = 0; i < (sizeof(JETYAK_UAV_UTILS::nameFromMode) / sizeof(*JETYAK_UAV_UTILS::nameFromMode)); ++i)
	{
		if (bsc_common::util::insensitiveEqual(req.data, JETYAK_UAV_UTILS::nameFromMode[i]))
		{
			currentMode_ = (JETYAK_UAV_UTILS::Mode)i;
			behaviorChanged_ = true;
			res.success = true;
			ROS_WARN("Mode changed to %s", req.data.c_str());
			return true;
		}
	}
	ROS_WARN("Invalid mode: %s", req.data.c_str());
	return false;
}

bool Behaviors::getModeCallback(jetyak_uav_utils::GetString::Request &req, jetyak_uav_utils::GetString::Response &res)
{
	res.data = JETYAK_UAV_UTILS::nameFromMode[(int)currentMode_];
	return true;
}

bool Behaviors::setBoatNSCallback(jetyak_uav_utils::SetString::Request &req, jetyak_uav_utils::SetString::Response &res)
{
	std::string ns = req.data;
	boatGPSSub_ = nh.subscribe(ns + "/global_posiiton/global", 1, &Behaviors::boatGPSCallback, this);
	boatIMUSub_ = nh.subscribe(ns + "/imu/data", 1, &Behaviors::boatIMUCallback, this);

	res.success = true;
	return true;
}

bool Behaviors::setFollowPIDCallback(jetyak_uav_utils::FourAxes::Request &req,
																		 jetyak_uav_utils::FourAxes::Response &res)
{
	// If following, taking off, or returning, change active controller
	if (currentMode_ == JETYAK_UAV_UTILS::FOLLOW or currentMode_ == JETYAK_UAV_UTILS::TAKEOFF or currentMode_ == JETYAK_UAV_UTILS::RETURN)
	{
		xpid_->updateParams(req.x[0], req.x[1], req.x[2]);
		ypid_->updateParams(req.y[0], req.y[1], req.y[2]);
		zpid_->updateParams(req.z[0], req.z[1], req.z[2]);
		wpid_->updateParams(req.w[0], req.w[1], req.w[2]);
		xpid_->reset();
		ypid_->reset();
		zpid_->reset();
		wpid_->reset();
	}

	// either way, change stored values
	follow_.kp.x = req.x[0];
	follow_.ki.x = req.x[1];
	follow_.kd.x = req.x[2];

	follow_.kp.y = req.y[0];
	follow_.ki.y = req.y[1];
	follow_.kd.y = req.y[2];

	follow_.kp.z = req.z[0];
	follow_.ki.z = req.z[1];
	follow_.kd.z = req.z[2];

	follow_.kp.w = req.w[0];
	follow_.ki.w = req.w[1];
	follow_.kd.w = req.w[2];

	res.success = true;
	return true;
}

bool Behaviors::setLandPIDCallback(jetyak_uav_utils::FourAxes::Request &req, jetyak_uav_utils::FourAxes::Response &res)
{
	// If landing, change active controller
	if (currentMode_ == JETYAK_UAV_UTILS::LAND)
	{
		xpid_->updateParams(req.x[0], req.x[1], req.x[2]);
		ypid_->updateParams(req.y[0], req.y[1], req.y[2]);
		zpid_->updateParams(req.z[0], req.z[1], req.z[2]);
		wpid_->updateParams(req.w[0], req.w[1], req.w[2]);
		xpid_->reset();
		ypid_->reset();
		zpid_->reset();
		wpid_->reset();
	}

	// either way, change stored values
	land_.kp.x = req.x[0];
	land_.ki.x = req.x[1];
	land_.kd.x = req.x[2];

	land_.kp.y = req.y[0];
	land_.ki.y = req.y[1];
	land_.kd.y = req.y[2];

	land_.kp.z = req.z[0];
	land_.ki.z = req.z[1];
	land_.kd.z = req.z[2];

	land_.kp.w = req.w[0];
	land_.ki.w = req.w[1];
	land_.kd.w = req.w[2];

	res.success = true;
	return true;
}

bool Behaviors::setFollowPositionCallback(jetyak_uav_utils::FourAxes::Request &req,
																					jetyak_uav_utils::FourAxes::Response &res)
{
	follow_.goal_pose.x = req.x[0];
	follow_.goal_pose.y = req.y[0];
	follow_.goal_pose.z = req.z[0];
	follow_.goal_pose.w = req.w[0];

	res.success = true;
	return true;
}

bool Behaviors::setLandPositionCallback(jetyak_uav_utils::FourAxes::Request &req,
																				jetyak_uav_utils::FourAxes::Response &res)
{
	land_.goal_pose.x = req.x[0];
	land_.goal_pose.y = req.y[0];
	land_.goal_pose.z = req.z[0];
	land_.goal_pose.w = req.w[0];

	res.success = true;
	return true;
}