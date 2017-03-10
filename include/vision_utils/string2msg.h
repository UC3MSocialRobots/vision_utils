#ifndef STRING2MSG_H
#define STRING2MSG_H
// ROS
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
// vision_utils
#include <vision_utils/string_split.h>

namespace vision_utils {

//! a function to cast a vector of numbers into a ROS message
template<class _Msg>
bool dvec2msg(const std::vector<double> & in, _Msg & out) {
  if (in.size() != 1)
    return false;
  //! alias for the C type: bool, int, float, etc.
  typedef typename _Msg::_data_type  _Type;
  out.data = (_Type) in.front(); // cast
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// template specifications
template<class _T>
inline bool dvec2msg3(const std::vector<double> & in,
                      _T & x1,  _T & x2,  _T & x3,
                      bool size_check = true) {
  if (size_check && in.size() != 3)
    return false;
  x1 = in[0];
  x2 = in[1];
  x3 = in[2];
  return true;
}

template<class _T>
inline bool dvec2msg4(const std::vector<double> & in,
                      _T & x1,  _T & x2,  _T & x3, _T & x4,
                      bool size_check = true) {
  if (size_check && in.size() != 4)
    return false;
  dvec2msg3(in, x1, x2, x3, false);
  x4 = in[3];
  return true;
}

template<class _T>
inline bool dvec2msg6(const std::vector<double> & in,
                      _T & x1, _T & x2, _T & x3, _T & x4, _T & x5, _T & x6,
                      bool size_check = true) {
  if (size_check && in.size() != 6)
    return false;
  dvec2msg4(in, x1, x2, x3, x4, false);
  x5 = in[4];
  x6 = in[5];
  return true;
}

template<class _T>
inline bool dvec2msg7(const std::vector<double> & in,
                      _T & x1, _T & x2, _T & x3, _T & x4, _T & x5, _T & x6, _T & x7,
                      bool size_check = true) {
  if (size_check && in.size() != 7)
    return false;
  dvec2msg6(in, x1, x2, x3, x4, x5, x6, false);
  x7 = in[6];
  return true;
}

//! std_msgs::Empty
template<>
bool dvec2msg<std_msgs::Empty>(const std::vector<double> & in,
                               std_msgs::Empty & /*out*/) {
  return in.empty(); // nothing to do
}
//! std_msgs::ColorRGBA
template<>
bool dvec2msg<std_msgs::ColorRGBA>(const std::vector<double> & in,
                                   std_msgs::ColorRGBA & out) {
  return dvec2msg4(in, out.r, out.g, out.b, out.a);
}

//! geometry_msgs::Point
template<>
bool dvec2msg<geometry_msgs::Point>(const std::vector<double> & in,
                                    geometry_msgs::Point & out) {
  return dvec2msg3(in, out.x, out.y, out.z);
}

//! geometry_msgs::Point32
template<>
bool dvec2msg<geometry_msgs::Point32>(const std::vector<double> & in,
                                      geometry_msgs::Point32 & out) {
  return dvec2msg3(in, out.x, out.y, out.z);
}

//! geometry_msgs::Vector3
template<>
bool dvec2msg<geometry_msgs::Vector3>(const std::vector<double> & in,
                                      geometry_msgs::Vector3 & out) {
  return dvec2msg3(in, out.x, out.y, out.z);
}

//! geometry_msgs::Pose2D
template<>
bool dvec2msg<geometry_msgs::Pose2D>(const std::vector<double> & in,
                                     geometry_msgs::Pose2D & out) {
  return dvec2msg3(in, out.x, out.y, out.theta);
}

//! geometry_msgs::Quaternion
template<>
bool dvec2msg<geometry_msgs::Quaternion>(const std::vector<double> & in,
                                         geometry_msgs::Quaternion & out) {
  return dvec2msg4(in, out.x, out.y, out.z, out.w);
}

//! geometry_msgs::Accel
template<>
bool dvec2msg<geometry_msgs::Accel>(const std::vector<double> & in,
                                    geometry_msgs::Accel & out) {
  return dvec2msg6(in, out.linear.x, out.linear.y, out.linear.z,
                   out.angular.x, out.angular.y, out.angular.z);
}

//! geometry_msgs::Twist
template<>
bool dvec2msg<geometry_msgs::Twist>(const std::vector<double> & in,
                                    geometry_msgs::Twist & out) {
  return dvec2msg6(in, out.linear.x, out.linear.y, out.linear.z,
                   out.angular.x, out.angular.y, out.angular.z);
}

//! geometry_msgs::Pose
template<>
bool dvec2msg<geometry_msgs::Pose>(const std::vector<double> & in,
                                   geometry_msgs::Pose & out) {
  return dvec2msg7(in, out.position.x, out.position.y, out.position.z,
                   out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//! rely on dvec2msg() for all cases based on numbers
template<class _Msg>
bool string2msg(const std::string & in, _Msg & out) {
  std::vector<double> values;
  if (!vision_utils::StringSplit_<double>(in, ";", &values)) {
    ROS_WARN("string2msg:StringSplit_() returned an error!");
    return false;
  }
  return dvec2msg(values, out);
}

// template specifications
//! std_msgs::String
template<>
bool string2msg(const std::string & in, std_msgs::String & out) {
  out.data = in;
  return true;
}
} // end namespace vision_utils

#endif // STRING2MSG_H
