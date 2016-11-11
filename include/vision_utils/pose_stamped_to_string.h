/*!
  \file        pose_stamped_to_string.h
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

#ifndef POSE_STAMPED_TO_STRING_H
#define POSE_STAMPED_TO_STRING_H
// std includes
#include <sstream> // for ostringstream
#include <string>
#include <vision_utils/printP.h>
#include <vision_utils/printP4.h>

namespace vision_utils {

template<class _Pose3Stamped>
inline std::string pose_stamped_to_string(const _Pose3Stamped & pose) {
  std::ostringstream ans;
  ans << "pose: {"
      << "t:" << std::fixed << pose.header.stamp.toSec() << "s, "
      << "frame: '" << pose.header.frame_id.c_str() << "' : "
      << "pos:" << printP(pose.pose.position)
      << ", orien:" << printP4(pose.pose.orientation)
      << " }";
  return ans.str();
}

} // end namespace vision_utils

#endif // POSE_STAMPED_TO_STRING_H
