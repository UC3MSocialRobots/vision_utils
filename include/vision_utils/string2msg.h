#ifndef STRING2MSG_H
#define STRING2MSG_H
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
#include <vision_utils/string_split.h>
#include <vision_utils/cast_from_string.h>
#include <vision_utils/cast_to_string.h>

namespace vision_utils {
#if 1
//! rely on cast_to_string() for most simple cases
template<class _Msg>
bool string2msg(const std::string & in, _Msg & out) {
  //! alias for the C type: bool, int, float, etc.
  typedef typename _Msg::_data_type  _Type;
  bool ok;
  out.data = vision_utils::cast_from_string<_Type>(in, ok);
  return ok;
}

////////////////////////////////////////////////////////////////////////////////
// template specifications
//! std_msgs::Empty
template<>
bool string2msg<std_msgs::Empty>(const std::string & /*in*/,
                                 std_msgs::Empty & /*out*/) {
  return true; // nothing to do
}
//! std_msgs::ColorRGBA
template<class _T>
inline bool string2xyzw(const std::string & value,
                       _T & outx,  _T & outy,  _T & outz, _T & outw) {
  std::vector<double> values;
  vision_utils::StringSplit_<double>(value, ";", &values);
  if (values.size() != 4)
    return false;
  outx = values[0];
  outy = values[1];
  outz = values[2];
  outw = values[3];
  return true;
}

template<>
bool string2msg<std_msgs::ColorRGBA>(const std::string & value,
                                     std_msgs::ColorRGBA & out) {
  return string2xyzw(value, out.r, out.g, out.b, out.a);
}

template<class _T>
inline bool string2xyz(const std::string & value,
                       _T & outx,  _T & outy,  _T & outz) {
  std::vector<double> values;
  vision_utils::StringSplit_<double>(value, ";", &values);
  if (values.size() != 3)
    return false;
  outx = values[0];
  outy = values[1];
  outz = values[2];
  return true;
}
template<class _Msg>
inline bool string2xyz(const std::string & value, _Msg & out) {
  return string2xyz(value, out.x, out.y, out.z);
}
//! geometry_msgs::Point
template<>
bool string2msg<geometry_msgs::Point>(const std::string & value,
                                      geometry_msgs::Point & out) {
  return string2xyz(value, out);
}

//! geometry_msgs::Point32
template<>
bool string2msg<geometry_msgs::Point32>(const std::string & value,
                                        geometry_msgs::Point32 & out) {
  return string2xyz(value, out);
}

//! geometry_msgs::Vector3
template<>
bool string2msg<geometry_msgs::Vector3>(const std::string & value,
                                        geometry_msgs::Vector3 & out) {
  return string2xyz(value, out);
}

//! geometry_msgs::Pose2D
template<>
bool string2msg<geometry_msgs::Pose2D>(const std::string & value,
                                       geometry_msgs::Pose2D & out) {
  return string2xyz(value, out.x, out.y, out.theta);
}

//! geometry_msgs::Quaternion
template<>
bool string2msg<geometry_msgs::Quaternion>(const std::string & value,
                                       geometry_msgs::Quaternion & out) {
  return string2xyzw(value, out.x, out.y, out.z, out.w);
}

template<class _Msg>
inline bool string2xyzxyz(const std::string & value, _Msg & out) {
  std::vector<double> values;
  vision_utils::StringSplit_<double>(value, ";", &values);
  if (values.size() != 6)
    return false;
  out.linear.x = values[0];
  out.linear.y = values[1];
  out.linear.z = values[2];
  out.angular.x = values[3];
  out.angular.y = values[4];
  out.angular.z = values[5];
  return true;
}

//! geometry_msgs::Accel
template<>
bool string2msg<geometry_msgs::Accel>(const std::string & value,
                                      geometry_msgs::Accel & out) {
  return string2xyzxyz(value, out);
}

//! geometry_msgs::Twist
template<>
bool string2msg<geometry_msgs::Twist>(const std::string & value,
                                      geometry_msgs::Twist & out) {
  return string2xyzxyz(value, out);
}

//! geometry_msgs::Pose
template<>
bool string2msg<geometry_msgs::Pose>(const std::string & value,
                                      geometry_msgs::Pose & out) {
  std::vector<double> values;
  vision_utils::StringSplit_<double>(value, ";", &values);
  if (values.size() != 7)
    return false;
  out.position.x = values[0];
  out.position.y = values[1];
  out.position.z = values[2];
  out.orientation.x = values[3];
  out.orientation.y = values[4];
  out.orientation.z = values[5];
  out.orientation.w = values[6];
  return true;
}
#else
//! a function to cast a vector of numbers into a ROS message
template<class _Msg>
bool dvec2msg(const std::vector<double> & in, _Msg & out) {
  if (in.size() != 1)
    return false;
  out.data = in.front();
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// template specifications
template<class _T>
inline bool dvec23(const std::vector<double> & values,
                   _T & x1,  _T & x2,  _T & x3,
                   bool size_check = true) {
  if (size_check && values.size() != 3)
    return false;
  x1 = values[0];
  x2 = values[1];
  x3 = values[2];
  return true;
}

template<class _T>
inline bool dvec24(const std::vector<double> & values,
                   _T & x1,  _T & x2,  _T & x3, _T & x4,
                   bool size_check = true) {
  if (size_check && values.size() != 4)
    return false;
  dvec23(values, x1, x2, x3, false);
  x4 = values[3];
  return true;
}

template<class _T>
inline bool dvec26(const std::vector<double> & values,
                   _T & x1, _T & x2, _T & x3, _T & x4, _T & x5, _T & x6,
                   bool size_check = true) {
  if (size_check && values.size() != 6)
    return false;
  dvec24(values, x1, x2, x3, x4, false);
  x5 = values[4];
  x6 = values[5];
  return true;
}

template<class _T>
inline bool dvec27(const std::vector<double> & values,
                   _T & x1, _T & x2, _T & x3, _T & x4, _T & x5, _T & x6, _T & x7,
                   bool size_check = true) {
  if (size_check && values.size() != 7)
    return false;
  dvec26(values, x1, x2, x3, x4, x5, x6, false);
  x7 = values[6];
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
bool dvec2msg<std_msgs::ColorRGBA>(const std::vector<double> & values,
                                   std_msgs::ColorRGBA & out) {
  return dvec24(values, out.r, out.g, out.b, out.a);
}

//! geometry_msgs::Point
template<>
bool dvec2msg<geometry_msgs::Point>(const std::vector<double> & values,
                                    geometry_msgs::Point & out) {
  return dvec23(values, out.x, out.y, out.z);
}

//! geometry_msgs::Point32
template<>
bool dvec2msg<geometry_msgs::Point32>(const std::vector<double> & values,
                                      geometry_msgs::Point32 & out) {
  return dvec23(values, out.x, out.y, out.z);
}

//! geometry_msgs::Vector3
template<>
bool dvec2msg<geometry_msgs::Vector3>(const std::vector<double> & values,
                                      geometry_msgs::Vector3 & out) {
  return dvec23(values, out.x, out.y, out.z);
}

//! geometry_msgs::Pose2D
template<>
bool dvec2msg<geometry_msgs::Pose2D>(const std::vector<double> & values,
                                     geometry_msgs::Pose2D & out) {
  return dvec23(values, out.x, out.y, out.theta);
}

//! geometry_msgs::Quaternion
template<>
bool dvec2msg<geometry_msgs::Quaternion>(const std::vector<double> & values,
                                         geometry_msgs::Quaternion & out) {
  return dvec24(values, out.x, out.y, out.z, out.w);
}

//! geometry_msgs::Accel
template<>
bool dvec2msg<geometry_msgs::Accel>(const std::vector<double> & values,
                                    geometry_msgs::Accel & out) {
  return dvec26(values, out.linear.x, out.linear.y, out.linear.z,
                out.angular.x, out.angular.y, out.angular.z);
}

//! geometry_msgs::Twist
template<>
bool dvec2msg<geometry_msgs::Twist>(const std::vector<double> & values,
                                    geometry_msgs::Twist & out) {
  return dvec26(values, out.linear.x, out.linear.y, out.linear.z,
                out.angular.x, out.angular.y, out.angular.z);
}

//! geometry_msgs::Pose
template<>
bool dvec2msg<geometry_msgs::Pose>(const std::vector<double> & values,
                                   geometry_msgs::Pose & out) {
  return dvec27(values, out.position.x, out.position.y, out.position.z,
                out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//! rely on dvec2msg() for all cases based on numbers
template<class _Msg>
bool string2msg(const std::string & in, _Msg & out) {
  std::vector<double> values;
  vision_utils::StringSplit_<double>(in, ";", &values);
  return dvec2msg(values, out);
}

template<>
bool string2msg(const std::string & in, std_msgs::String & out) {
  out.data = in;
  return true;
}
#endif
} // end namespace vision_utils

#endif // STRING2MSG_H
