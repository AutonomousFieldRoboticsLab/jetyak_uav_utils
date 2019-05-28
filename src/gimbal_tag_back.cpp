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
 * This file implements the gimbal_tag node
 * 
 * Author: Michail Kalaitzakis
 */

#include "jetyak_uav_utils/gimbal_tag.h"
#include <math.h>

#include "tf/transform_datatypes.h"

gimbal_tag::gimbal_tag(ros::NodeHandle &nh)
{
	// Subscribe to topics
	tagPoseSub = nh.subscribe("ar_pose_marker", 1, &gimbal_tag::tagCallback, this);

	gimbalAngleSub = nh.subscribe("/dji_sdk/gimbal_angle", 1, &gimbal_tag::gimbalCallback, this);
	vehicleAttiSub = nh.subscribe("/dji_sdk/attitude", 1, &gimbal_tag::attitudeCallback, this);

	// Set up publisher
	tagPosePub = nh.advertise<geometry_msgs::Vector3>("ar_pose_v3", 1);
	tagBodyPosePub = nh.advertise<geometry_msgs::PoseStamped>("tag_pose", 1);

	tagFound = false;

	if (!ros::param::get("isM100", isM100))
	{
		isM100 = true;
		ROS_WARN("isM100 not available, defaulting to %i", isM100);
	}

	// Initialize the constant offset between Gimbal and Vehicle orientation
	qCamera2Gimbal = tf::Quaternion(0.5, -0.5, 0.5, 0.5);

}

void gimbal_tag::changeTagAxes()
{
	tf::Matrix3x3 rTag(qTag);
	double tR, tP, tY;
	rTag.getRPY(tR, tP, tY);

	qTag = tf::createQuaternionFromRPY(0, 0, tP);

//	if (isM100)
//		
//	else {
//		qTagBody = tf::createQuaternionFromRPY(-tP, -tR, -tY);
//
//	}
	//qTagBody.normalize();
}

void gimbal_tag::publishTagPose()
{
	if (tagFound)
	{
		
		geometry_msgs::PoseStamped tagPoseBody;

		// Calculate offset quaternion
		qOffset = qVehicle.inverse() * qGimbal;
		qOffset.normalize();
		//ROS_WARN("offset X: %1.2f, Y: %1.2f, Z: %1.2f W: %1.2f", qOffset[0],qOffset[1],qOffset[2],qOffset[3]);


		// Apply rotation to go from gimbal frame to body frame
		changeTagAxes();
		qTagBody = qOffset * qTag;
		
		tf::Quaternion positionTagBody = qOffset * posTag * qOffset.inverse();
		// Get time
		ros::Time time = ros::Time::now();

		// Update header
		tagPoseBody.header.stamp = time;
		tagPoseBody.header.frame_id = "body_FLU";

		tagPoseBody.pose.position.x = positionTagBody[0];
		tagPoseBody.pose.position.y = positionTagBody[1];
		tagPoseBody.pose.position.z = positionTagBody[2];

		tf::quaternionTFToMsg(qTagBody, tagPoseBody.pose.orientation);

		tf::Matrix3x3 tmp(qTagBody);
		double tR, tP, tY;
		tmp.getRPY(tR, tP, tY);
		ROS_WARN("pose R %1.2f, P %1.2f, Y %1.2f",tR,tP,tY);

		tagBodyPosePub.publish(tagPoseBody);
	}
}

// Callbacks
void gimbal_tag::tagCallback(const ar_track_alvar_msgs::AlvarMarkers &msg)
{
	if (!msg.markers.empty())
	{
		
		// Update Tag quaternion
		tf::quaternionMsgToTF(msg.markers[0].pose.pose.orientation, qTag);
		//qTag=qTag.inverse().normalized();
		//tf::Quaternion offset(-0.5,0.5,0.5,0.5);
		//qTag = offset *qTag;		
//		tf::Matrix3x3 tmp(qTag);
//		double tR, tP, tY;
//		tmp.getRPY(tR, tP, tY);
//		ROS_WARN("qTag R %1.2f, P %1.2f, Y %1.2f",tR,tP,tY);
		// Update Tag position as quaternion
		posTag[0] = msg.markers[0].pose.pose.position.x;
		posTag[1] = msg.markers[0].pose.pose.position.y;
		posTag[2] = msg.markers[0].pose.pose.position.z;
		posTag[3] = 0;

		// Go from Camera frame to Gimbal frame
		qTag = qCamera2Gimbal * qTag;
		qTag.normalize();
		posTag = qCamera2Gimbal.inverse() * posTag * qCamera2Gimbal;
		//ROS_WARN("tag X: %1.2f, Y: %1.2f, Z: %1.2f",posTag[0],posTag[1],posTag[2]);
		tagFound = true;
		publishTagPose();
	}
	else
		tagFound = false;
}

void gimbal_tag::gimbalCallback(const geometry_msgs::Vector3Stamped &msg)
{
	double newY = -msg.vector.y;
	double newZ = 90-msg.vector.z;
	if(newZ>=180) newZ-=360;
	else if(newZ<-180) newZ+=360;
	qGimbal = tf::createQuaternionFromRPY(DEG2RAD(msg.vector.x), DEG2RAD(newY), DEG2RAD(newZ));

	qGimbal.normalize();
}

void gimbal_tag::attitudeCallback(const geometry_msgs::QuaternionStamped &msg)
{
	// Update Vehicle quaternion
	tf::quaternionMsgToTF(msg.quaternion, qVehicle);
	qVehicle.normalize();
}

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gimbal_test");
	ros::NodeHandle nh;

	gimbal_tag tagTracker(nh);

	ros::spin();
	
	return 0;
}
