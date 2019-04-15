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
 * This file implements the main functions of the behaviors node (Constructor, Destructor, control switch, and main)
 * 
 * Author: Brennan Cain
 */

#include "jetyak_uav_utils/behaviors.h"

Behaviors::Behaviors(ros::NodeHandle &nh_param)
{
	nh = nh_param;

	// initialize mode
	currentMode_ = JETYAK_UAV_UTILS::HOVER;

	assignSubscribers();
	assignPublishers();
	assignServiceClients();
	assignServiceServers();
	downloadParams("~");

	leave_.input.axes.push_back(0);
	leave_.input.axes.push_back(0);
	leave_.input.axes.push_back(0);
	leave_.input.axes.push_back(0);
	leave_.input.axes.push_back(JETYAK_UAV_UTILS::WORLD_FRAME | JETYAK_UAV_UTILS::VELOCITY_CMD);

	// initialize the tag
	tagPose_.pose.orientation.x = 0;
	tagPose_.pose.orientation.y = 0;
	tagPose_.pose.orientation.z = 0;
	tagPose_.pose.orientation.w = 1;

	// Initialize the PID controllers
	createPID(follow_.kp, follow_.ki, follow_.kd);
}

Behaviors::~Behaviors()
{
}

void Behaviors::doBehaviorAction()
{
	// simpleTag_.x = tagPose_.pose.position.x;
	// simpleTag_.y = tagPose_.pose.position.y;

	simpleTag_.z = tagPose_.pose.position.z;

	simpleTag_.w = bsc_common::util::yaw_from_quat(tagPose_.pose.orientation);

	bsc_common::util::rotate_vector(tagPose_.pose.position.x, tagPose_.pose.position.y, -simpleTag_.w, simpleTag_.x,
																	simpleTag_.y);

	simpleTag_.t = tagPose_.header.stamp.toSec();

	switch (currentMode_)
	{
	case JETYAK_UAV_UTILS::TAKEOFF:
	{
		takeoffBehavior();
		break;
	}
	case JETYAK_UAV_UTILS::FOLLOW:
	{
		followBehavior();
		break;
	}
	case JETYAK_UAV_UTILS::LEAVE:
	{
		leaveBehavior();
		break;
	}
	case JETYAK_UAV_UTILS::RETURN:
	{
		returnBehavior();
		break;
	}
	case JETYAK_UAV_UTILS::LAND:
	{
		landBehavior();
		break;
	}
	case JETYAK_UAV_UTILS::RIDE:
	{
		rideBehavior();
		break;
	}
	case JETYAK_UAV_UTILS::HOVER:
	{
		hoverBehavior();
		break;
	}
	default:
	{
		if (propellorsRunning)
		{
			ROS_ERROR("Mode out of bounds: %i. Now hovering.", (char)currentMode_);
			this->currentMode_ = JETYAK_UAV_UTILS::HOVER;
		}
		else
		{
			ROS_ERROR("Mode out of bounds: %i. Now riding.", (char)currentMode_);
			this->currentMode_ = JETYAK_UAV_UTILS::RIDE;
		}
		break;
	}
	}

	// Publish the current behavior mode
	std_msgs::UInt8 behaviorMode;
	behaviorMode.data = currentMode_;
	modePub_.publish(behaviorMode);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "behaviors");
	ros::NodeHandle nh;
	Behaviors behaviors_o(nh);
	ros::Rate rate(25);

	while (ros::ok())
	{
		ros::spinOnce();

		behaviors_o.doBehaviorAction();

		rate.sleep();
	}
	return 0;
}
