#ifndef MSG2STRING_H
#define MSG2STRING_H
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

namespace vision_utils {

//! a function to cast a vector of numbers into a ROS message
template<class _Msg>
bool msg2dvec(const _Msg & in, std::vector<double> & out) {
  out.resize(1);
  out[0] = (double) in.data; // cast
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// template specifications
template<class _T>
inline bool msg2dvec3(const _T & x1,  const _T & x2,  const _T & x3,
                      std::vector<double> & out,
                      bool resize = true) {
  if (resize)
    out.resize(3);
  out[0] = x1;
  out[1] = x2;
  out[2] = x3;
  return true;
}

template<class _T>
inline bool msg2dvec4(const _T & x1,  const _T & x2,  const _T & x3, const _T & x4,
                      std::vector<double> & out,
                      bool resize = true) {
  if (resize)
    out.resize(4);
  msg2dvec3(x1, x2, x3, out, false);
  out[3] = x4;
  return true;
}

template<class _T>
inline bool msg2dvec6(const _T & x1, const _T & x2, const _T & x3, const _T & x4, const _T & x5, const _T & x6,
                      std::vector<double> & out,
                      bool resize = true) {
  if (resize)
    out.resize(6);
  msg2dvec4(x1, x2, x3, x4, out, false);
  out[4] = x5;
  out[5] = x6;
  return true;
}

template<class _T>
inline bool msg2dvec7(const _T & x1, const _T & x2, const _T & x3, const _T & x4, const _T & x5, const _T & x6, const _T & x7,
                      std::vector<double> & out,
                      bool resize = true) {
  if (resize)
    out.resize(7);
  msg2dvec6(x1, x2, x3, x4, x5, x6, out, false);
  out[6] = x7;
  return true;
}

//! std_msgs::Empty
template<>
bool msg2dvec<std_msgs::Empty>(const std_msgs::Empty & /*in*/,
                               std::vector<double> & out) {
  out.clear();
  return true;
}
//! std_msgs::ColorRGBA
template<>
bool msg2dvec<std_msgs::ColorRGBA>(const std_msgs::ColorRGBA & in,
                                   std::vector<double> & out) {
  return msg2dvec4(in.r, in.g, in.b, in.a, out);
}

//! geometry_msgs::Point
template<>
bool msg2dvec<geometry_msgs::Point>(const geometry_msgs::Point & in,
                                    std::vector<double> & out) {
  return msg2dvec3(in.x, in.y, in.z, out);
}

//! geometry_msgs::Point32
template<>
bool msg2dvec<geometry_msgs::Point32>(const geometry_msgs::Point32 & in,
                                      std::vector<double> & out) {
  return msg2dvec3(in.x, in.y, in.z, out);
}

//! geometry_msgs::Vector3
template<>
bool msg2dvec<geometry_msgs::Vector3>(const geometry_msgs::Vector3 & in,
                                      std::vector<double> & out) {
  return msg2dvec3(in.x, in.y, in.z, out);
}

//! geometry_msgs::Pose2D
template<>
bool msg2dvec<geometry_msgs::Pose2D>(const geometry_msgs::Pose2D & in,
                                     std::vector<double> & out) {
  return msg2dvec3(in.x, in.y, in.theta, out);
}

//! geometry_msgs::Quaternion
template<>
bool msg2dvec<geometry_msgs::Quaternion>(const geometry_msgs::Quaternion & in,
                                         std::vector<double> & out) {
  return msg2dvec4(in.x, in.y, in.z, in.w, out);
}

//! geometry_msgs::Accel
template<>
bool msg2dvec<geometry_msgs::Accel>(const geometry_msgs::Accel & in,
                                    std::vector<double> & out) {
  return msg2dvec6(in.linear.x, in.linear.y, in.linear.z,
                   in.angular.x, in.angular.y, in.angular.z, out);
}

//! geometry_msgs::Twist
template<>
bool msg2dvec<geometry_msgs::Twist>(const geometry_msgs::Twist & in,
                                    std::vector<double> & out) {
  return msg2dvec6(in.linear.x, in.linear.y, in.linear.z,
                   in.angular.x, in.angular.y, in.angular.z, out);
}

//! geometry_msgs::Pose
template<>
bool msg2dvec<geometry_msgs::Pose>(const geometry_msgs::Pose & in,
                                   std::vector<double> & out) {
  return msg2dvec7(in.position.x, in.position.y, in.position.z,
                   in.orientation.x, in.orientation.y, in.orientation.z,
                   in.orientation.w, out);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//! rely on msg2dvec() for all cases based on numbers
template<class _Msg>
bool msg2string(const _Msg & in, std::string & out) {
  std::vector<double> values;
  if (!msg2dvec(in, values))
    return false;
  std::ostringstream out_str;
  //out_str << std::fixed;
  unsigned int nvalues = values.size();
  for (unsigned int i = 0; i < nvalues; ++i) {
    out_str << values[i];
    if (i < nvalues - 1)
      out_str << "; ";
  }
  out = out_str.str();
  return true;
}

// template specifications
//! std_msgs::String
template<>
bool msg2string(const std_msgs::String & in, std::string & out) {
  out = in.data; // simple copy
  return true;
}

} // end namespace vision_utils

#endif // MSG2STRING_H
