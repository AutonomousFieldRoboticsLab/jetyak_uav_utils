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
	this->state.position = msg->position;
	this->state.velocity = msg->velocity;
	this->state.attitude = msg->attitude;
	this->state.angular_velocity = msg->angular_velocity;

	lqr_->updatePos(msg->position[0], msg->position[1], msg->position[2]);
	lqr_->updateVel(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
	lqr_->updateAngPos(msg->attitude[0], msg->attitude[1], msg->attitude[2]);
	lqr_->updateAngVel(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);
}
void Behaviors::uavGPSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	if (msg->status.status >= 0)
	{
		uavGPS_.header = msg->header;
		uavGPS_.status = msg->status;
		uavGPS_.latitude = msg->latitude;
		uavGPS_.longitude = msg->longitude;
		uavGPS_.altitude = msg->altitude;
		uavGPS_.position_covariance = msg->position_covariance;
		uavGPS_.position_covariance_type = msg->position_covariance_type;
	}
	else
	{
		ROS_WARN("UAV GNSS fix failed. Status: %i", msg->status.status);
	}
}

void Behaviors::uavHeightCallback(const std_msgs::Float32::ConstPtr &msg)
{
	uavHeight_ = msg->data;
}

void Behaviors::boatGPSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	if (msg->status.status >= 0)
	{
		boatGPS_.header = msg->header;
		boatGPS_.status = msg->status;
		boatGPS_.latitude = msg->latitude;
		boatGPS_.longitude = msg->longitude;
		boatGPS_.altitude = msg->altitude;
		boatGPS_.position_covariance = msg->position_covariance;
		boatGPS_.position_covariance_type = msg->position_covariance_type;
	}
	else
	{
		ROS_WARN("Boat GNSS fix failed. Status: %i", msg->status.status);
	}
}

void Behaviors::uavAttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
	uavAttitude_.header = msg->header;
	uavAttitude_.quaternion = msg->quaternion;
}

void Behaviors::uavImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
	uavImu_.header = msg->header;
	uavImu_.orientation = msg->orientation;
	uavImu_.orientation_covariance = msg->orientation_covariance;
	uavImu_.angular_velocity = msg->angular_velocity;
	uavImu_.angular_velocity_covariance = msg->angular_velocity_covariance;
	uavImu_.linear_acceleration = msg->linear_acceleration;
	uavImu_.linear_acceleration_covariance = msg->linear_acceleration_covariance;
}

void Behaviors::boatIMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
	boatImu_.header = msg->header;
	boatImu_.orientation = msg->orientation;
	boatImu_.orientation_covariance = msg->orientation_covariance;
	boatImu_.angular_velocity = msg->angular_velocity;
	boatImu_.angular_velocity_covariance = msg->angular_velocity_covariance;
	boatImu_.linear_acceleration = msg->linear_acceleration;
	boatImu_.linear_acceleration_covariance = msg->linear_acceleration_covariance;
}

void Behaviors::uavVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
	uavWorldVel_.x = msg->vector.x;
	uavWorldVel_.y = msg->vector.y;
	uavWorldVel_.z = msg->vector.z;
	uavWorldVel_.t = msg->header.stamp.toSec();
	if (uavAttitude_.header.stamp.toSec() != 0)
	{
		tf2::Quaternion qRot;
		tf2::convert(uavAttitude_.quaternion, qRot);
		tf2::Vector3 wVel;
		tf2::convert(msg->vector, wVel);
		tf2::Vector3 bVel = tf2::quatRotate(qRot.inverse(), wVel);
		uavBodyVel_.x = bVel.getX();
		uavBodyVel_.y = bVel.getY();
		uavBodyVel_.z = bVel.getZ();
		uavBodyVel_.t = uavWorldVel_.t;
		//ROS_WARN("BV\t x: %.2f, y: %.2f, z: %.2f", uavBodyVel_.x, uavBodyVel_.y, uavBodyVel_.z);
	}
}

void Behaviors::extCmdCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	leave_.input.header = msg->header;
	leave_.input.axes = msg->axes;
}
