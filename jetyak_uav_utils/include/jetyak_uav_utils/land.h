/** Controls taking off and following modes
* Angles camera down on liftoff and uses that camera to move up and back
* Angles camera up to find tag2 and follow at some xyz,yaw
* Keeps tag in that position
*/

#ifndef JETYAK_UAV_UTILS_LAND_H_
#define JETYAK_UAV_UTILS_LAND_H_

#include "ros/ros.h"

#include "jetyak_uav_utils/Mode.h"
#include "../lib/bsc_common/include/pid.h"
#include "../lib/bsc_common/include/util.h"

#include "dynamic_reconfigure/server.h"
#include "jetyak_uav_utils/FollowConstantsConfig.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Joy.h"

class land {
  private:

    //declare PID controller constants and controllers
    geometry_msgs::Quaternion kp_,kd_,ki_,follow_pos_;
    bsc_common::PID *xpid_,*ypid_,*zpid_,*wpid_;

    //Subscribers and publishers
    ros::Subscriber arTagSub_, modeSub_;
    ros::Publisher cmdPub_, modePub_, landPub_;

    // x: x dist, y: y dist, z: z dist, w: yaw
    geometry_msgs::Quaternion initialFlyPose_,flyPose_;

    //Keep track of currentmode
    char currentMode_=0;

    bool firstLandLoop_=true;
    double droneLastSeen_=0;

    /** arTagCallback
    * Use tf from jetyak to tags and info on tags to determine position relative to kayak
    *
    * If "close" to dock
    *   landPub_
    *   mode<=Landed
    * else
    *   If first run of loop
    *     init pid
    *     set position goal
    *   increment position goal
    *   set error
    *   publish command
    *
    * @param msg vector of marker info
    */
    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    /** modeCallback
    * receives the mode change
    * if state changed to TAKINGOFF, set wasLastLanded_ to true
    *
    * @param msg Mode that has been activated
    */
    void modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg);

  public:

    void reconfigureCallback(jetyak_uav_utils::FollowConstantsConfig &config, uint32_t level);

    /** Constructor
    * Creates the publishers, subscribers, and service clients
    *
    * @param nh Node handler
    */
    land(ros::NodeHandle& nh);
};
#endif
