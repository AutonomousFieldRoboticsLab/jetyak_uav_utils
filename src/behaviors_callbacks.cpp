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
 * This file implements the rostopic callbacks of the behaviors node
 * 
 * Author: Brennan Cain
 */

#include "jetyak_uav_utils/behaviors.h"
void Behaviors::stateCallback(const jetyak_uav_utils::ObservedState::ConstPtr &msg)
{
	this->state.header = msg->header;
	this->state.drone_p = msg->drone_p;
	this->state.drone_pdot = msg->drone_pdot;
	this->state.drone_q = msg->drone_q;
	this->state.drone_qdot = msg->drone_qdot;
	this->state.boat_p = msg->boat_p;
	this->state.boat_pdot = msg->boat_pdot;
	this->state.heading = msg->heading;
	this->state.origin = msg->origin;

	Eigen::Matrix<double, 12, 1> state;
}
void Behaviors::tagCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	if (msg->header.stamp.toSec() - lastSpotted > resetFilterTimeThresh)
	{
		ROS_WARN("Tag lost for %1.2fs", msg->header.stamp.toSec() - lastSpotted);
	}
	lastSpotted = msg->header.stamp.toSec();
}

void Behaviors::extCmdCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	leave_.input.header = msg->header;
	leave_.input.axes = msg->axes;
}
