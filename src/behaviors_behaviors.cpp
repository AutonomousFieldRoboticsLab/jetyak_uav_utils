#include "jetyak_uav_utils/behaviors.h"
/** @file behaviors_behaviors.cpp
 *
 * Implements behaviors:
 * 	takeoffBehavior
 * 	followBehavior
 * 	returnBehavior
 * 	landBehavior
 * 	rideBehavior
 * 	hoverBehavior
 */

void Behaviors::takeoffBehavior()
{
	/*
	Get altitude/tagpose

	Take off with pilot service

	*/
	if (!propellorsRunning)
	{
		std_srvs::Trigger srv;
		takeoffSrv_.call(srv);
		if (srv.response.success)
		{
			ROS_INFO("Propellors running, switching to follow");
			propellorsRunning = true;
			this->currentMode_ = JETYAK_UAV::FOLLOW;
			this->behaviorChanged_ = true;
		}
		else
		{
			ROS_WARN("Failure to Start props");
		}
	}
}

void Behaviors::followBehavior()
{
	if (behaviorChanged_)
	{	// if just changed
		follow_.lastSpotted = simpleTag_.t;
		resetPID();
		setPID(follow_.kp, follow_.ki, follow_.kd);
		behaviorChanged_ = false;
	}
	else
	{	// DO the loop

		if (follow_.lastSpotted != simpleTag_.t)
		{	// if time changed
			follow_.lastSpotted = simpleTag_.t;

			// line up with pad
			xpid_->update(follow_.goal_pose.x - simpleTag_.x, simpleTag_.t);
			ypid_->update(follow_.goal_pose.y - simpleTag_.y, simpleTag_.t);
			zpid_->update(follow_.goal_pose.z - simpleTag_.z, simpleTag_.t);
			wpid_->update(follow_.goal_pose.w - simpleTag_.w, simpleTag_.t);

			double c_vx = vGain * (uavBodyVel_.x + tagVel_.x);
			double c_vy = vGain * (uavBodyVel_.y + tagVel_.y);

			sensor_msgs::Joy cmd;
			cmd.axes.push_back(-xpid_->get_signal() + c_vx);
			cmd.axes.push_back(-ypid_->get_signal() + c_vy);
			cmd.axes.push_back(-zpid_->get_signal());
			cmd.axes.push_back(-wpid_->get_signal());
			cmd.axes.push_back(JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::BODY_FRAME | JETYAK_UAV::YAW_RATE);
			cmdPub_.publish(cmd);
		}
		else
		{	// if time is same

			if (ros::Time::now().toSec() - follow_.lastSpotted > 1)
			{
				ROS_WARN("Tag Lost for %1.2f seconds", ros::Time::now().toSec() - follow_.lastSpotted);
				sensor_msgs::Joy cmd;
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::YAW_RATE);
				cmdPub_.publish(cmd);
				return;
			}
		}
	}
	resetPID();
}

void Behaviors::leaveBehavior()
{
	if (behaviorChanged_)
	{
		std_srvs::SetBool enable;
		enable.request.data = false;
		enableGimbalSrv_.call(enable);
		behaviorChanged_ = false;
	}
	cmdPub_.publish(leave_.input);
}

void Behaviors::returnBehavior()
{
	double east = bsc_common::util::latlondist(0, boatGPS_.longitude, 0, uavGPS_.longitude);
	double north = bsc_common::util::latlondist(boatGPS_.latitude, 0, uavGPS_.latitude, 0);

	if (boatGPS_.longitude < uavGPS_.longitude)
		east = -east;
	if (boatGPS_.latitude < uavGPS_.latitude)
		north = -north;

	double heading = atan2(north, east);
	ROS_WARN("Heading: %1.3f", heading);

	if (ros::Time::now().toSec() - simpleTag_.t < return_.tagTime and uavHeight_ <= return_.finalHeight)
	{
		if (return_.stage != return_.SETTLE)
		{
			resetPID();
			bsc_common::pose4d_t zero;
			zero.x = zero.y = zero.z = zero.w = 0;
			setPID(follow_.kp, zero, zero);	// set i to zeroes
		}
		return_.stage = return_.SETTLE;
		if ((pow(follow_.goal_pose.x - simpleTag_.x, 2) + pow(follow_.goal_pose.y - simpleTag_.y, 2)) <
				return_.settleRadiusSquared)
		{
			ROS_WARN("Settled, now following");
			currentMode_ = JETYAK_UAV::FOLLOW;
			behaviorChanged_ = true;
		}
		else
		{	// line up with pad
			ROS_WARN("Settling down, z-error: %1.2f", follow_.goal_pose.z - simpleTag_.z);

			xpid_->update(follow_.goal_pose.x - simpleTag_.x, simpleTag_.t);
			ypid_->update(follow_.goal_pose.y - simpleTag_.y, simpleTag_.t);
			zpid_->update(follow_.goal_pose.z - simpleTag_.z, simpleTag_.t);
			wpid_->update(follow_.goal_pose.w - simpleTag_.w, simpleTag_.t);

			double c_vx = vGain * (uavBodyVel_.x + tagVel_.x);
			double c_vy = vGain * (uavBodyVel_.y + tagVel_.y);

			sensor_msgs::Joy cmd;
			cmd.axes.push_back(-xpid_->get_signal() + c_vx);
			cmd.axes.push_back(-ypid_->get_signal() + c_vy);
			cmd.axes.push_back(0);
			// cmd.axes.push_back(-zpid_->get_signal());
			cmd.axes.push_back(-wpid_->get_signal());
			cmd.axes.push_back(JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::BODY_FRAME | JETYAK_UAV::YAW_RATE);
			cmdPub_.publish(cmd);
		}
	}
	else if (return_.stage == return_.SETTLE and ros::Time::now().toSec() - simpleTag_.t > return_.tagLossThresh)
	{
		ROS_WARN("Tag lost for %1.2f seconds, going back up", ros::Time::now().toSec() - simpleTag_.t);
		return_.stage = return_.UP;
	}

	else if (behaviorChanged_)
	{
		/*
		set stage to up
		set behavior changed to false
		init pid
	*/
		ROS_WARN("Behavior is now return");
		resetPID();
		bsc_common::pose4d_t zero;
		zero.x = zero.y = zero.z = zero.w = 0;
		setPID(follow_.kp, zero, zero);
		behaviorChanged_ = false;
		return_.stage = return_.UP;
		ROS_WARN("Going Up");
		std_srvs::Trigger downSrvTmp;
		lookdownSrv_.call(downSrvTmp);
	}

	else if (return_.stage == return_.UP)
	{
		if (uavHeight_ >= return_.gotoHeight)
		{
			ROS_WARN("Changed OVER");
			return_.stage = return_.OVER;

			std_srvs::Trigger downSrvTmp;
			lookdownSrv_.call(downSrvTmp);
		}
		else
		{
			double z_correction = return_.gotoHeight - uavHeight_;
			ROS_WARN("Goal: %1.2f, Current %1.2f", return_.gotoHeight, uavHeight_);
			sensor_msgs::Joy cmd;
			cmd.axes.push_back(0);
			cmd.axes.push_back(0);
			cmd.axes.push_back(z_correction);
			cmd.axes.push_back(0);
			cmd.axes.push_back(JETYAK_UAV::WORLD_FRAME | JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::YAW_RATE);
			cmdPub_.publish(cmd);
		}
		std_srvs::Trigger downSrvTmp;
		lookdownSrv_.call(downSrvTmp);
	}
	else if (return_.stage == return_.OVER)
	{
		/*
		saturate x and y with directions
		z_cmd = gotoHeight-altitude
		if in sphere around goal
			set stage to DOWN
	*/
		if (bsc_common::util::latlondist(uavGPS_.latitude, uavGPS_.longitude, boatGPS_.latitude, boatGPS_.longitude) <
				return_.downRadius)
		{
			ROS_WARN("Changed DOWN");
			return_.stage = return_.DOWN;

			std_srvs::Trigger downSrvTmp;
			lookdownSrv_.call(downSrvTmp);
		}
		else
		{
			double e_c = follow_.kp.x * east;
			double n_c = follow_.kp.y * north;
			double u_c = follow_.kp.z * (return_.gotoHeight - uavHeight_);

			sensor_msgs::Joy cmd;
			cmd.axes.push_back(e_c);
			cmd.axes.push_back(n_c);
			cmd.axes.push_back(u_c);
			cmd.axes.push_back(0);
			cmd.axes.push_back(JETYAK_UAV::WORLD_FRAME | JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::YAW_RATE);
			cmdPub_.publish(cmd);
		}
		std_srvs::Trigger downSrvTmp;
		lookdownSrv_.call(downSrvTmp);
	}
	else if (return_.stage = return_.DOWN)
	{
		/*
		find proper pid constants to maintain location more or less
		z=finalHeight-altitude
		we wont change the mode here as it should search for the tags and
		hopefully find one
	*/

		ROS_WARN("Goal: %1.3f, Current %1.3f", return_.finalHeight, uavHeight_);

		double e_c = follow_.kp.x * east;
		double n_c = follow_.kp.y * north;
		double u_c = follow_.kp.z * (return_.finalHeight - uavHeight_);

		sensor_msgs::Joy cmd;
		cmd.axes.push_back(e_c);
		cmd.axes.push_back(n_c);
		cmd.axes.push_back(u_c);
		cmd.axes.push_back(simpleTag_.w);
		cmd.axes.push_back(JETYAK_UAV::WORLD_FRAME | JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::YAW_RATE);
		cmdPub_.publish(cmd);
	}
	else
	{
		ROS_ERROR("BAD CONDITIONALS, DEFAULTING TO HOVER");
		currentMode_ = JETYAK_UAV::HOVER;
	}
};

void Behaviors::landBehavior()
{
	if (behaviorChanged_)
	{
		resetPID();
		setPID(land_.kp, land_.ki, land_.kd);
		behaviorChanged_ = false;
	}
	else
	{	// DO the loop

		if (land_.lastSpotted != simpleTag_.t)
		{	// if time changed

			// If pose is within a cylinder of radius .1 and height .1
			bool inX = (land_.lowX < simpleTag_.x) and (simpleTag_.x < land_.highX);
			bool inY = (land_.lowY < simpleTag_.y) and (simpleTag_.y < land_.highY);
			bool inZ = (land_.lowZ < simpleTag_.z) and (simpleTag_.z < land_.highZ);

			bool inVelThreshold = pow(tagVel_.x, 2) + pow(tagVel_.y, 2) + pow(tagVel_.z, 2) < land_.velThreshSqr;
			bool inAngleThreshhold = abs(simpleTag_.w) < land_.angleThresh;
			if (inX and inY and inZ and inVelThreshold and inAngleThreshhold)
			{
				ROS_WARN("CALLING LAND SERVICE");
				std_srvs::Trigger srv;
				landSrv_.call(srv);
				if (srv.response.success)
				{
					currentMode_ = JETYAK_UAV::RIDE;
					return;
				}
			}

			land_.lastSpotted = simpleTag_.t;
		}
		else
		{	// if time is same

			if (ros::Time::now().toSec() - land_.lastSpotted > 3)
			{
				ROS_WARN("Tag lost %1.2f seconds, switching to return.", ros::Time::now().toSec() - land_.lastSpotted);
				this->currentMode_ = JETYAK_UAV::RETURN;
				this->behaviorChanged_ = true;
				return;
			}
			else if (ros::Time::now().toSec() - land_.lastSpotted > .5)
			{	// if time has been same for over 3 tick
				ROS_WARN("No tag update: %f", ros::Time::now().toSec() - land_.lastSpotted);
				sensor_msgs::Joy cmd;
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::YAW_RATE);
				cmdPub_.publish(cmd);
				return;
			}
		}

		// line up with pad
		xpid_->update(land_.goal_pose.x - simpleTag_.x, simpleTag_.t);
		ypid_->update(land_.goal_pose.y - simpleTag_.y, simpleTag_.t);
		zpid_->update(land_.goal_pose.z - simpleTag_.z, simpleTag_.t);
		wpid_->update(land_.goal_pose.w - simpleTag_.w, simpleTag_.t);

		double c_vx = vGain * (uavBodyVel_.x + tagVel_.x);
		double c_vy = vGain * (uavBodyVel_.y + tagVel_.y);

		sensor_msgs::Joy cmd;
		cmd.axes.push_back(-xpid_->get_signal() + c_vx);
		cmd.axes.push_back(-ypid_->get_signal() + c_vy);
		cmd.axes.push_back(-zpid_->get_signal());
		cmd.axes.push_back(-wpid_->get_signal());
		cmd.axes.push_back(JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::BODY_FRAME | JETYAK_UAV::YAW_RATE);
		cmdPub_.publish(cmd);
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
	sensor_msgs::Joy cmd;
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::YAW_RATE);
	cmdPub_.publish(cmd);
};
