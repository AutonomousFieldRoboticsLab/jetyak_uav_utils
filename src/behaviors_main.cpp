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

	// Initialize mode to Ride
	currentMode_ = JETYAK_UAV_UTILS::RIDE;

	assignSubscribers();
	assignPublishers();
	assignServiceClients();
	assignServiceServers();
	downloadParams("~");

	leave_.input.axes.push_back(0);
	leave_.input.axes.push_back(0);
	leave_.input.axes.push_back(0);
	leave_.input.axes.push_back(0);
	leave_.input.axes.push_back(JETYAK_UAV_UTILS::WORLD_RATE);

	lqr_ = new bsc_common::LQR(generalK);
	land_.lqr = new bsc_common::LQR(landK);
}

Behaviors::~Behaviors()
{
}

void Behaviors::doBehaviorAction()
{

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
	Eigen::Vector2d cmds = gimbal_angle_cmd();
	geometry_msgs::Vector3 msg;
	msg.x = 0;
	msg.y = cmds(0);
	msg.z = cmds(1);
	gimbalCmdPub_.publish(msg);
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
