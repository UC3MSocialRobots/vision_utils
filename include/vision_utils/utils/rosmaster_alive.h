#include <ros/ros.h>

bool rosmaster_alive() {
  return ros::ok();
}
