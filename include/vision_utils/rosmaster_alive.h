#include <ros/ros.h>

namespace vision_utils {

//! Check whether the master is up
bool rosmaster_alive() {
  return ros::master::check();
}

} // end namespace vision_utils
