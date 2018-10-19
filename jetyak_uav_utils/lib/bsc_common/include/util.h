#ifndef BSC_COMMON_UTIL_
#define BSC_COMMON_UTIL_


#include <math.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Empty.h"

namespace bsc_common {


class util {
public:



  static constexpr long double C_PI = 3.14159265358979323846; // My long PI value

  /* rpy_from_quat
   * Gives the roll pitch and yaw in radians given a quaternion
   *
   * @param orientation quaternion
   * @param state saves the roll,pitch,yaw encoded as a Vector3
   */
  static void rpy_from_quat(const geometry_msgs::Quaternion& pose, geometry_msgs::Vector3* state);

  /* yaw_from_quat
   * Gives the roll pitch and yaw in radians given a quaternion
   *
   * @param orientation quaternion
   * @return yaw in range [-pi,pi)
   */
  static double yaw_from_quat(const geometry_msgs::Quaternion& orientation);

  /* clip
   * clips a value to be between a min and maxSpeed
   * equivalent to max(min(high,x),low)
   *
   * @param x value to be clipped
   * @param low lower bound
   * @param high upper bound
   *
   * @return clipped value of x
  */
  template <typename T>
  static T clip(T x, T low, T high);

  /* rotate_vector
   *
   * @param x x coordinate
   * @param y y coordinate
   * @param theta angle to rotate
   * @param xp x after rotation
   * @param yp y after rotation
  */
  static void rotate_vector(double x, double y, double theta,double &xp,double &yp);

  /* inverse_pose
   * invert the pose. Make it from child to parent frame
   *
   * @param in pose from parent to child
   * @param out pose from child to parent
  */
  static void inverse_pose(const geometry_msgs::Pose& in, geometry_msgs::Pose &out);

  /* ang_dist
   * finds the shortest angular distance between two angles. The sign follows
   * CCW as positive. ex. start=170, stop=-170 => +20, start=+170, stop=170 => -20
   *
   * @param start beginning angle
   * @param end final angle
   * @param rad=true using radians if true (default)
  */
   static double ang_dist(double start,double stop, bool rad=true);
};

} // namespace bsc_common
#endif
