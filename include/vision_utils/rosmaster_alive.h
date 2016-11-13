#include <ros/ros.h>

namespace vision_utils {

//! Check whether the master is up
bool rosmaster_alive() {
  bool ok = ros::master::check();
  if (!ok)
    ROS_WARN("rosmaster_alive() is false!");
  return ok;
}

} // end namespace vision_utils
