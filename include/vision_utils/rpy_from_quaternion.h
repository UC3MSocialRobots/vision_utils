/*!
  \file        rpy_from_quaternion.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/2
________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________
 */

#ifndef RPY_FROM_QUATERNION_H
#define RPY_FROM_QUATERNION_H

#include <ros/common.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

namespace vision_utils {

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

} // end namespace vision_utils

#endif // RPY_FROM_QUATERNION_H
