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
			bool shouldTrack = not ((JETYAK_UAV_UTILS::Mode)i == JETYAK_UAV_UTILS::LEAVE);
			if(shouldTrack xor trackEnabled) {
				std_srvs::SetBool enable;
				enable.request.data = shouldTrack;

				enableGimbalSrv_.call(enable);
				if(not enable.response.success and visionRequired) {
					ROS_WARN("Failed to %s tracking",enable.request.data?"enable":"disable");
					res.success = false;
					return false;
				} else {
					ROS_WARN("Tracking is %s",enable.request.data?"enabled":"disabled");
					trackEnabled = shouldTrack;
				}
			}
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
