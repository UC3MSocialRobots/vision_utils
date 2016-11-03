/*!
  \file        print_pose.h
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

#ifndef PRINT_POSE_H
#define PRINT_POSE_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

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

} // end namespace vision_utils

#endif // PRINT_POSE_H
