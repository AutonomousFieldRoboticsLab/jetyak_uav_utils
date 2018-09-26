#include "jetyak_uav_utils/behaviors.h"

behaviors::behaviors(ros::NodeHandle& nh):
  xpid_(NULL),
  ypid_(NULL),
  zpid_(NULL),
  wpid_(NULL)
 {
  //subscribers
  tagPoseSub_ = nh.subscribe("/jetyak_uav_utils/tag_pose",1,&behaviors::tagPoseCallback, this);
  uavGPSSub_ = nh.subscribe("/dji_sdk/gps_position",1,&behaviors::uavGPSCallback, this);
  boatGPSSub_ = nh.subscribe("boat_gps",1,&behaviors::uavGPSCallback, this);
  uavAttSub_ =  nh.subscribe("/dji_sdk/attitude",1, &behaviors::uavAttitudeCallback, this);
  boatIMUSub_ =  nh.subscribe("boat_imu",1, &behaviors::boatIMUCallback, this);

  //Publishers
  cmdPub_ = nh.advertise<sensor_msgs::Joy>("jetyak_uav_utils/behavior_cmd",1);

  //service clients
  taskSrv_ = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");

  // Service servers
  modeService_ = nh.advertiseService("jetyak_uav_utils/mode",&behaviors::modeCallback,this);
}

behaviors::~behaviors() {}


bool behaviors::modeCallback(jetyak_uav_utils::Mode::Request  &req, jetyak_uav_utils::Mode::Response &res) {

  this->currentMode_ = req.mode;
  this->behaviorChanged_=true;
  switch(this->currentMode_)
  {
    case req.TAKEOFF: {
      dji_sdk::DroneTaskControl srv;
      srv.request.task=4;
      taskSrv_.call(srv);
      res.success=srv.response.result;
      if(res.success)
      {
        this->currentMode_=req.FOLLOW;
      }
      else
      {
        this->currentMode_=req.HOVER;
      }
    }
    case req.LAND: {}
    case req.FOLLOW: {}
    case req.LEAVE: {}
    case req.RETURN: {
      res.success=true;
      break;
    }

  }
  behaviorChanged_=true;
  return res.success;
}

void behaviors::tagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  tagPose_.pose=msg->pose;
  tagPose_.header=msg->header;
}

void behaviors::uavGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if(msg->status.status>=0) {
    uavGPS_.header=msg->header;
    uavGPS_.status=msg->status;
    uavGPS_.latitude=msg->latitude;
    uavGPS_.latitude=msg->latitude;
    uavGPS_.latitude=msg->latitude;
    uavGPS_.position_covariance=msg->position_covariance;
    uavGPS_.position_covariance_type=msg->position_covariance_type;
  } else {
    ROS_WARN("UAV GNSS fix failed. Status: %i",msg->status.status);
  }
}

void behaviors::boatGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if(msg->status.status>=0) {
    boatGPS_.header=msg->header;
    boatGPS_.status=msg->status;
    boatGPS_.latitude=msg->latitude;
    boatGPS_.latitude=msg->latitude;
    boatGPS_.latitude=msg->latitude;
    boatGPS_.position_covariance=msg->position_covariance;
    boatGPS_.position_covariance_type=msg->position_covariance_type;
  } else {
    ROS_WARN("Boat GNSS fix failed. Status: %i",msg->status.status);
  }
}

void behaviors::uavAttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
  uavAttitude_.header=msg->header;
  uavAttitude_.quaternion = msg->quaternion;
}

void behaviors::uavImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  uavImu_.header=msg->header;
  uavImu_.orientation=msg->orientation;
  uavImu_.orientation_covariance=msg->orientation_covariance;
  uavImu_.angular_velocity=msg->angular_velocity;
  uavImu_.angular_velocity_covariance=msg->angular_velocity_covariance;
  uavImu_.linear_acceleration=msg->linear_acceleration;
  uavImu_.linear_acceleration_covariance=msg->linear_acceleration_covariance;
}
void behaviors::boatIMUCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  boatImu_.header=msg->header;
  boatImu_.orientation=msg->orientation;
  boatImu_.orientation_covariance=msg->orientation_covariance;
  boatImu_.angular_velocity=msg->angular_velocity;
  boatImu_.angular_velocity_covariance=msg->angular_velocity_covariance;
  boatImu_.linear_acceleration=msg->linear_acceleration;
  boatImu_.linear_acceleration_covariance=msg->linear_acceleration_covariance;
}

void behaviors::takeoffBehavior() {
  dji_sdk::DroneTaskControl srv;
  srv.request.task=4;
  taskSrv_.call(srv);
  if(srv.response.result) {
    currentMode_=jetyak_uav_utils::Mode::Request::FOLLOW;
  }
}

void behaviors::followBehavior() {
  if(behaviorChanged_) {
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();

    xpid_->updateParams(follow_.kp.x ,follow_.ki.x,follow_.kd.x);
    ypid_->updateParams(follow_.kp.y ,follow_.ki.y,follow_.kd.y);
    zpid_->updateParams(follow_.kp.z ,follow_.ki.z,follow_.kd.z);
    wpid_->updateParams(follow_.kp.w ,follow_.ki.w,follow_.kd.w);
  }
}

void behaviors::doBehaviorAction() {
  switch(currentMode_) {
    case jetyak_uav_utils::Mode::Request::TAKEOFF: {
      takeoffBehavior();
      break;
    }
    case jetyak_uav_utils::Mode::Request::FOLLOW: {
      followBehavior();
      break;
    }
  }
}

void behaviors::landReconfigureCallback(jetyak_uav_utils::LandConstantsConfig &config, uint32_t level) {
  ROS_WARN("%s","Reconfigure received for Landing");

  land_.kp.x=config.kp_x;
  land_.kp.y=config.kp_y;
  land_.kp.z=config.kp_z;
  land_.kp.w=config.kp_w;

  land_.kd.x=config.kd_x;
  land_.kd.y=config.kd_y;
  land_.kd.z=config.kd_z;
  land_.kd.w=config.kd_w;

  land_.ki.x=config.ki_x;
  land_.ki.y=config.ki_y;
  land_.ki.z=config.ki_z;
  land_.ki.w=config.ki_w;

  land_.collapseRatio = config.collapse_ratio;

  if (xpid_ != NULL)
  {
    xpid_->updateParams(land_.kp.x,land_.ki.x,land_.kd.x);
    ypid_->updateParams(land_.kp.y,land_.ki.y,land_.kd.y);
    zpid_->updateParams(land_.kp.z,land_.ki.z,land_.kd.z);
    wpid_->updateParams(land_.kp.w,land_.ki.w,land_.kd.w);
  } else {
    xpid_ = new bsc_common::PID(land_.kp.x,land_.ki.x,land_.kd.x);
    ypid_ = new bsc_common::PID(land_.kp.y,land_.ki.y,land_.kd.y);
    zpid_ = new bsc_common::PID(land_.kp.z,land_.ki.z,land_.kd.z);
    wpid_ = new bsc_common::PID(land_.kp.w,land_.ki.w,land_.kd.w);
  }
}

void behaviors::followReconfigureCallback(jetyak_uav_utils::FollowConstantsConfig &config, uint32_t level) {
  ROS_WARN("%s","Reconfigure received for follow");

  follow_.kp.x=config.kp_x;
  follow_.kp.y=config.kp_y;
  follow_.kp.z=config.kp_z;
  follow_.kp.w=config.kp_w;

  follow_.kd.x=config.kd_x;
  follow_.kd.y=config.kd_y;
  follow_.kd.z=config.kd_z;
  follow_.kd.w=config.kd_w;

  follow_.ki.x=config.ki_x;
  follow_.ki.y=config.ki_y;
  follow_.ki.z=config.ki_z;
  follow_.ki.w=config.ki_w;

  follow_.follow_pose.x = config.follow_x;
  follow_.follow_pose.y = config.follow_y;
  follow_.follow_pose.z = config.follow_z;
  follow_.follow_pose.w = config.follow_w;

  if (xpid_ != NULL)
  {
    xpid_->updateParams(follow_.kp.x,follow_.ki.x,follow_.kd.x);
    ypid_->updateParams(follow_.kp.y,follow_.ki.y,follow_.kd.y);
    zpid_->updateParams(follow_.kp.z,follow_.ki.z,follow_.kd.z);
    wpid_->updateParams(follow_.kp.w,follow_.ki.w,follow_.kd.w);
  } else {
    xpid_ = new bsc_common::PID(follow_.kp.x,follow_.ki.x,follow_.kd.x);
    ypid_ = new bsc_common::PID(follow_.kp.y,follow_.ki.y,follow_.kd.y);
    zpid_ = new bsc_common::PID(follow_.kp.z,follow_.ki.z,follow_.kd.z);
    wpid_ = new bsc_common::PID(follow_.kp.w,follow_.ki.w,follow_.kd.w);
  }
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"uav_behaviors");
  ros::NodeHandle nh;
  behaviors uav_behaviors(nh);
  ros::Rate rate(10);

  //Dynamic reconfigure for land configuration
  dynamic_reconfigure::Server<jetyak_uav_utils::LandConstantsConfig> landCfgServer;
  dynamic_reconfigure::Server<jetyak_uav_utils::LandConstantsConfig>::CallbackType landCfgCallback;
  boost::function<void (jetyak_uav_utils::LandConstantsConfig &,int) >
      landCfgCallback2(boost::bind( &behaviors::landReconfigureCallback,&uav_behaviors, _1, _2 ) );
  landCfgCallback=landCfgCallback2;
  landCfgServer.setCallback(landCfgCallback);

  //Dynamic reconfigure for follow configuration
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig> followCfgServer;
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig>::CallbackType followCfgCallback;
  boost::function<void (jetyak_uav_utils::FollowConstantsConfig &,int) >
      followCfgCallback2(boost::bind( &behaviors::followReconfigureCallback,&uav_behaviors, _1, _2 ) );
  followCfgCallback=followCfgCallback2;
  followCfgServer.setCallback(followCfgCallback);

  while(ros::ok())
  {
    ros::spinOnce();

    uav_behaviors.doBehaviorAction();

    rate.sleep();
  }
  return 0;
}
