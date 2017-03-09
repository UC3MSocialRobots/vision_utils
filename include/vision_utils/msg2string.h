#ifndef MSG2STRING_H
#define MSG2STRING_H
// ROS
#include <std_msgs/Empty.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
// vision_utils
#include <vision_utils/cast_to_string.h>

namespace vision_utils {

template<class _Msg>
bool msg2string(const _Msg & in, std::string & out) {
  bool ok;
  out = vision_utils::cast_to_string(in.data, ok);
  return ok;
}

////////////////////////////////////////////////////////////////////////////////
// template specifications
//! std_msgs::Empty
template<>
bool msg2string(const std_msgs::Empty & /*in*/, std::string & out) {
  out = "";
  return true;
}

//! std_msgs::ColorRGBA
template<>
bool msg2string(const std_msgs::ColorRGBA & in, std::string & out) {
  std::ostringstream out_str;
  out_str << in.r << ";" << in.g << ";" << in.b << ";" << in.a;
  out = out_str.str();
  return true;
}

template<class _Msg>
bool xyz2string(const _Msg & in, std::string & out) {
  std::ostringstream out_str;
  out_str << in.x << ";" << in.y << ";" << in.z;
  out = out_str.str();
  return true;
}

//! geometry_msgs::Point
template<>
bool msg2string(const geometry_msgs::Point & in, std::string & out) {
  return xyz2string(in, out);
}

//! geometry_msgs::Point32
template<>
bool msg2string(const geometry_msgs::Point32 & in, std::string & out) {
  return xyz2string(in, out);
}

//! geometry_msgs::Vector3
template<>
bool msg2string(const geometry_msgs::Vector3 & in, std::string & out) {
  return xyz2string(in, out);
}

//! geometry_msgs::Pose2D
template<>
bool msg2string(const geometry_msgs::Pose2D & in, std::string & out) {
  std::ostringstream out_str;
  out_str << in.x << ";" << in.y << "; " << in.theta;
  out = out_str.str();
  return true;
}

//! geometry_msgs::Quaternion
template<>
bool msg2string(const geometry_msgs::Quaternion & in, std::string & out) {
  std::ostringstream out_str;
  out_str << in.x << ";" << in.y << ";" << in.z << ";" << in.w;
  out = out_str.str();
  return true;
}

template<class _Msg>
bool xyzxyz2string(const _Msg & in, std::string & out) {
  std::ostringstream out_str;
  out_str << in.linear.x << ";" << in.linear.y << ";" << in.linear.z << "; "
          << in.angular.x << ";" << in.angular.y << ";" << in.angular.z;
  out = out_str.str();
  return true;
}

//! geometry_msgs::Accel
template<>
bool msg2string(const geometry_msgs::Accel & in, std::string & out) {
  return xyzxyz2string(in, out);
}

//! geometry_msgs::Twist
template<>
bool msg2string(const geometry_msgs::Twist & in, std::string & out) {
  return xyzxyz2string(in, out);
}

//! geometry_msgs::Pose
template<>
bool msg2string(const geometry_msgs::Pose & in, std::string & out) {
  std::ostringstream out_str;
  out_str << in.position.x << ";" << in.position.y << ";" << in.position.z << "; "
          << in.orientation.x << ";" << in.orientation.y << ";"
          << in.orientation.z << "; " << in.orientation.w;
  out = out_str.str();
  return true;
}

} // end namespace vision_utils

#endif // MSG2STRING_H
