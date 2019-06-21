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
 * This file implements the behaviors of the behaviors node
 * 
 * Author: Brennan Cain
 */

#include "jetyak_uav_utils/behaviors.h"
void Behaviors::takeoffBehavior()
{
	if (!propellorsRunning)
	{
		std_srvs::Trigger srv;
		takeoffSrv_.call(srv);
		if (srv.response.success)
		{
			ROS_INFO("Propellors running, switching to follow");
			propellorsRunning = true;
			currentMode_ = JETYAK_UAV_UTILS::FOLLOW;
			behaviorChanged_ = true;
		}
		else
		{
			ROS_WARN("Failure to Start props");
		}
	}
}

void Behaviors::followBehavior()
{
	if (ros::Time::now().toSec() - lastSpotted <= 5 || true) // TODO: Add tag loss threshold for follow_
	{
		// Get the setpoint in the drone FLU
		Eigen::Vector4d goal_b;
		goal_b << follow_.goal_pose.x, follow_.goal_pose.y, follow_.goal_pose.z, follow_.goal_pose.w; // Goal in boat FLU
		Eigen::Vector4d goal_d = boat_to_drone(goal_b);																								// Goal in drone FLU

		// Get boat velocity in drone frame
		Eigen::Vector2d vBoat(state.boat_pdot.x, state.boat_pdot.y);				// Boat velocity in world frame
		vBoat = bsc_common::util::rotation_matrix(state.drone_q.z) * vBoat; // Boat velocity in drone frame

		Eigen::Matrix<double, 12, 1> set;
		set << goal_d(0), goal_d(1), goal_d(2), // Position setpoint (xyz)
				vBoat(0, 0), vBoat(1, 0), 0,				// Velocity setpoint (xyz)
				0, 0, goal_d(3),										// Angle setpoint (rpy)
				0, 0, 0;														// Angular velocity setpoint (rpy)
		Eigen::Vector4d cmdM = lqr_->getCommand(set);

		sensor_msgs::Joy cmd;
		cmd.axes.push_back(cmdM(0));
		cmd.axes.push_back(cmdM(1));
		cmd.axes.push_back(cmdM(2));
		cmd.axes.push_back(cmdM(3));
		cmd.axes.push_back(JETYAK_UAV_UTILS::LQR);
		cmdPub_.publish(cmd);
	}
	else
	{
		ROS_WARN("Follow lost tag for more than 5 seconds, hovering");
		currentMode_ = JETYAK_UAV_UTILS::HOVER;
		behaviorChanged_ = true;
	}
}

void Behaviors::leaveBehavior()
{
	cmdPub_.publish(leave_.input);
}

void Behaviors::returnBehavior()
{
	Eigen::Vector4d goal_boatFLU;
	goal_boatFLU << return_.goal.x, return_.goal.y, return_.goal.z, return_.goal.w;
	Eigen::Vector4d offset = boat_to_drone(goal_boatFLU); // Vector pointing from the UAV to the follow setpoint

	if (ros::Time::now().toSec() - lastSpotted <= return_.tagTime and state.drone_p.z <= return_.finalHeight + return_.heightThresh)
	{
		return_.stage = return_.SETTLE;
		if ((pow(offset(0), 2) + pow(offset(1), 2)) < return_.settleRadiusSquared)
		{
			ROS_WARN("Settled, now following");
			currentMode_ = JETYAK_UAV_UTILS::FOLLOW;
			behaviorChanged_ = true;
		}
		else
		{
			ROS_WARN("Settling: %1.2fm over", -offset(2));

			// Get boat velocity in drone frame
			Eigen::Vector2d vBoat(state.boat_pdot.x, state.boat_pdot.y);				 // Boat velocity in world frame
			vBoat = bsc_common::util::rotation_matrix(-state.drone_q.z) * vBoat; // Boat velocity in drone frame

			Eigen::Matrix<double, 12, 1> set;
			set << offset(0), offset(1), offset(2), // Position setpoint (xyz)
					vBoat(0), vBoat(1), 0,							// Velocity setpoint (xyz)
					0, 0, offset(3),										// Angle setpoint (rpy)
					0, 0, 0;														// Angular velocity setpoint (rpy)

			Eigen::Vector4d cmdM = lqr_->getCommand(set);
			sensor_msgs::Joy cmd;
			cmd.axes.push_back(cmdM(0));
			cmd.axes.push_back(cmdM(1));
			cmd.axes.push_back(cmdM(2));
			cmd.axes.push_back(cmdM(3));
			cmd.axes.push_back(JETYAK_UAV_UTILS::LQR);
			cmdPub_.publish(cmd);
		}
	}
	else if (return_.stage == return_.SETTLE and ros::Time::now().toSec() - state.header.stamp.toSec() > return_.tagLossThresh)
	{
		ROS_WARN("Tag lost for %1.2f seconds, going back up", ros::Time::now().toSec() - state.header.stamp.toSec());
		return_.stage = return_.UP;
	}

	else if (behaviorChanged_)
	{
		ROS_WARN("Behavior is now return");
		behaviorChanged_ = false;
		return_.stage = return_.UP;
		ROS_WARN("Going Up");
	}

	else if (return_.stage == return_.UP)
	{
		if (state.drone_p.z >= return_.gotoHeight - return_.heightThresh)
		{
			ROS_WARN("Changed OVER");
			return_.stage = return_.OVER;
		}
		else
		{
			double u_c = return_.gotoHeight - state.drone_p.z;
			ROS_WARN("Goal: %1.2f, Current %1.2f", return_.gotoHeight, state.drone_p.z);

			// Get boat velocity in drone frame
			Eigen::Vector2d vBoat(state.boat_pdot.x, state.boat_pdot.y);				 // Boat velocity in world frame
			vBoat = bsc_common::util::rotation_matrix(-state.drone_q.z) * vBoat; // Boat velocity in drone frame

			Eigen::Matrix<double, 12, 1> set;
			set << 0, 0, u_c,		 // Position setpoint (xyz)
					0, 0, 0,				 // Velocity setpoint (xyz)
					0, 0, offset(3), // Angle setpoint (rpy)
					0, 0, 0;				 // Angular velocity setpoint (rpy)

			Eigen::Vector4d cmdM = lqr_->getCommand(set);
			sensor_msgs::Joy cmd;
			cmd.axes.push_back(cmdM(0));
			cmd.axes.push_back(cmdM(1));
			cmd.axes.push_back(cmdM(2));
			cmd.axes.push_back(cmdM(3));
			cmd.axes.push_back(JETYAK_UAV_UTILS::LQR);
			cmdPub_.publish(cmd);
		}
	}
	else if (return_.stage == return_.OVER)
	{
		if ((pow(offset(0), 2) + pow(offset(1), 2)) < return_.downRadius)
		{
			ROS_WARN("Changed DOWN");
			return_.stage = return_.DOWN;
		}
		else
		{
			double u_c = return_.gotoHeight - state.drone_p.z;

			// Get boat velocity in drone frame
			Eigen::Vector2d vBoat(state.boat_pdot.x, state.boat_pdot.y);				 // Boat velocity in world frame
			vBoat = bsc_common::util::rotation_matrix(-state.drone_q.z) * vBoat; // Boat velocity in drone frame

			Eigen::Matrix<double, 12, 1> set;
			set << offset(0), offset(1), u_c, // Position setpoint (xyz)
					vBoat(0), vBoat(1), 0,				// Velocity setpoint (xyz)
					0, 0, offset(3),							// Angle setpoint (rpy)
					0, 0, 0;											// Angular velocity setpoint (rpy)

			Eigen::Vector4d cmdM = lqr_->getCommand(set);
			sensor_msgs::Joy cmd;
			cmd.axes.push_back(cmdM(0));
			cmd.axes.push_back(cmdM(1));
			cmd.axes.push_back(cmdM(2));
			cmd.axes.push_back(cmdM(3));
			cmd.axes.push_back(JETYAK_UAV_UTILS::LQR);
			cmdPub_.publish(cmd);
		}
	}
	else if (return_.stage = return_.DOWN)
	{
		double u_c = return_.finalHeight - state.drone_p.z;

		// Get boat velocity in drone frame
		Eigen::Vector2d vBoat(state.boat_pdot.x, state.boat_pdot.y);				 // Boat velocity in world frame
		vBoat = bsc_common::util::rotation_matrix(-state.drone_q.z) * vBoat; // Boat velocity in drone frame

		Eigen::Matrix<double, 12, 1> set;
		set << offset(0), offset(1), u_c, // Position setpoint (xyz)
				vBoat(0), vBoat(1), 0,				// Velocity setpoint (xyz)
				0, 0, offset(3),							// Angle setpoint (rpy)
				0, 0, 0;											// Angular velocity setpoint (rpy)

		Eigen::Vector4d cmdM = lqr_->getCommand(set);
		sensor_msgs::Joy cmd;
		cmd.axes.push_back(cmdM(0));
		cmd.axes.push_back(cmdM(1));
		cmd.axes.push_back(cmdM(2));
		cmd.axes.push_back(cmdM(3));
		cmd.axes.push_back(JETYAK_UAV_UTILS::LQR);
		cmdPub_.publish(cmd);
	}
	else
	{
		ROS_ERROR("BAD CONDITIONALS, DEFAULTING TO HOVER");
		currentMode_ = JETYAK_UAV_UTILS::HOVER;
	}
};

void Behaviors::landBehavior()
{
	if (behaviorChanged_)
	{
		behaviorChanged_ = false;
	}
	else
	{ // DO the loop
		// Get the setpoint in the drone FLU
		Eigen::Vector4d goal_b;
		goal_b << land_.goal_pose.x, land_.goal_pose.y, land_.goal_pose.z, land_.goal_pose.w; // Goal in boat FLU
		Eigen::Vector4d goal_d = boat_to_drone(goal_b);																				// Goal in drone FLU

		Eigen::Vector2d vBoat(state.boat_pdot.x, state.boat_pdot.y);				 // Boat velocity in world frame
		vBoat = bsc_common::util::rotation_matrix(-state.drone_q.z) * vBoat; // Boat velocity in drone frame

		if (ros::Time::now().toSec() - lastSpotted <= 3 or true)
		{

			if (inLandThreshold())
			{
				ROS_WARN("CALLING LAND SERVICE");
				std_srvs::Trigger srv;
				landSrv_.call(srv);
				if (srv.response.success)
				{
					currentMode_ = JETYAK_UAV_UTILS::RIDE;
				}
			}
			else
			{

				Eigen::Matrix<double, 12, 1> set;
				set << goal_d(0), goal_d(1), goal_d(2), // Position setpoint (xyz)
						vBoat(0, 0), vBoat(1, 0), 0,				// Velocity setpoint (xyz)
						0, 0, goal_d(3),										// Angle setpoint (rpy)
						0, 0, 0;														// Angular velocity setpoint (rpy)

				Eigen::Vector4d cmdM = land_.lqr->getCommand(set);
				sensor_msgs::Joy cmd;
				cmd.axes.push_back(cmdM(0));
				cmd.axes.push_back(cmdM(1));
				cmd.axes.push_back(cmdM(2));
				cmd.axes.push_back(cmdM(3));
				cmd.axes.push_back(JETYAK_UAV_UTILS::LQR);
				cmdPub_.publish(cmd);
			}
		}
		else //landing lost the tag for too long, dangerous
		{
			currentMode_ = JETYAK_UAV_UTILS::FOLLOW;
			behaviorChanged_ = true;
			followBehavior();
		}
	}
}

void Behaviors::rideBehavior()
{
	if (propellorsRunning)
	{
		std_srvs::Trigger srv;
		landSrv_.call(srv);
		propellorsRunning = srv.response.success;
		if (srv.response.success)
		{
			ROS_WARN("Arms deactivated");
		}
		else
		{
			ROS_WARN("Failed to deactivate arms");
		}
	}
}

void Behaviors::hoverBehavior()
{
	// Hover is space
	sensor_msgs::Joy cmd;
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(JETYAK_UAV_UTILS::WORLD_RATE);
	cmdPub_.publish(cmd);
};
