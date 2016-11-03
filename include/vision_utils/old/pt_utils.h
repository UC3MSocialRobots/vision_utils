#ifndef PT_UTILS_H
#define PT_UTILS_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

// version specific code - http://answers.ros.org/question/9562/how-do-i-test-the-ros-version-in-c-code/
#include <ros/common.h>

namespace vision_utils {

//////////////////////////////////////////////////////////////////////////////
//cut:copy2
//! copy the 2 fields of a (x, y) structure to another one
template<class _Tsrc, class _Tdst>
inline void copy2(const _Tsrc & src, _Tdst & dst) {
  dst.x = src.x; dst.y = src.y;
}
//cut:copy3
//! copy the 3 fields of a (x, y, z) structure to another one
template<class _Tsrc, class _Tdst>
inline void copy3(const _Tsrc & src, _Tdst & dst) {
  dst.x = src.x; dst.y = src.y; dst.z = src.z;
}

////////////////////////////////////////////////////////////////////////////////
//cut:print_point
template<class _T>
inline std::string print_point(const _T & pt) {
  std::ostringstream out;
  out << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")";
  return out.str();
}

////////////////////////////////////////////////////////////////////////////////
//cut:rpy_from_quaternion
//! extract roll, pitch, yaw from a quaternion
inline void rpy_from_quaternion(const geometry_msgs::Quaternion & quat,
                                double & roll, double & pitch, double & yaw) {
  // init all values to 0 for avoiding readings of uninitalized values
  roll = 0; pitch = 0; yaw = 0;

// electric:  v1.6.5
#if ROS_VERSION_MINIMUM(1, 7, 0) // fuerte code
  tf::Quaternion bt_q;
  tf::quaternionMsgToTF(quat, bt_q);
  tf::Matrix3x3 mat(bt_q);
  mat.getRPY(roll, pitch, yaw);
#else // electric code
  btQuaternion q(quat.x, quat.y, quat.z, quat.w);
  btMatrix3x3(q).getEulerYPR(roll, pitch, yaw);
#endif // USE
}

////////////////////////////////////////////////////////////////////////////////
//cut:print_pose
/*!
  Trabsform a pose to a string
 \param pose
 \return std::string
*/
inline std::string print_pose(const geometry_msgs::Pose & pose) {
  std::ostringstream out;
  double r, p, y;
  rpy_from_quaternion(pose.orientation, r, p, y);
  out << "position: " <<print_point(pose.position)
      << ", rpy:(" << r << ", " << p << ", " << y << ")";
  return out.str();
}

////////////////////////////////////////////////////////////////////////////////

/*!
  Trabsform a pose to a string
 \param pose
 \return std::string
*/
inline std::string print_pose(const geometry_msgs::PoseStamped & pose) {
  std::ostringstream out;
  out << print_pose(pose.pose) << " in '" << pose.header.frame_id << "'";
  return out.str();
}


} // end namespace vision_utils

#endif // PT_UTILS_H
