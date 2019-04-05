#include "jetyak_uav_utils/behaviors.h"
/** @file behaviors_callbacks.cpp
 *
 * Implements subscriber callbacks:
 * 	tagPoseCallback
 * 	uavGPSCallback
 * 	boatGPSCallback
 * 	uavAttitudeCallback
 * 	uavImuCallback
 * 	boatIMUCallback
 *  extCmdCallback
 */
void Behaviors::tagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	if (msg->header.stamp.toSec() - tagPose_.header.stamp.toSec() > resetFilterTimeThresh)
	{
		ROS_WARN("Tag detected after %1.2f, resetting filter", msg->header.stamp.toSec() - tagPose_.header.stamp.toSec());
		std_srvs::Trigger srv;
		resetKalmanSrv_.call(srv);
		if (srv.response.success)
		{
			tagPose_.pose = msg->pose;
			tagPose_.header = msg->header;
			ROS_WARN("Filter reset and tag updated");
		}
		else
		{
			ROS_WARN("Reset failed with message: %s", srv.response.message.c_str());
		}
	}
	else
	{
		tagPose_.pose = msg->pose;
		tagPose_.header = msg->header;
	}
}

void Behaviors::tagVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
	tagVel_.t = msg->header.stamp.toSec();
	tagVel_.x = msg->vector.x;
	tagVel_.y = msg->vector.y;
	tagVel_.z = msg->vector.z;
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
		ROS_WARN("BV\t x: %.2f, y: %.2f, z: %.2f", uavBodyVel_.x, uavBodyVel_.y, uavBodyVel_.z);
	}
}

void Behaviors::extCmdCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	leave_.input.header = msg->header;
	leave_.input.axes = msg->axes;
}
