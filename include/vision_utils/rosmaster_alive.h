#include <ros/ros.h>

namespace vision_utils {

bool rosmaster_alive() {
  return ros::ok();
}

} // end namespace vision_utils
