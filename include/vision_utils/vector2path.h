/*!
  \file        vector2path.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/11
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
#ifndef VECTOR2PATH_H
#define VECTOR2PATH_H
#include <nav_msgs/Path.h>
#include <vector>

namespace vision_utils {

template<class _Pt2>
inline void vector2path(const std::vector<_Pt2> & traj_xy, nav_msgs::Path & _path_msg) {
  unsigned int npts = traj_xy.size();
  _path_msg.poses.resize(npts);
  for (unsigned int i = 0; i < npts; ++i) {
    _path_msg.poses[i].pose.position.x = traj_xy[i].x;
    _path_msg.poses[i].pose.position.y = traj_xy[i].y;
    _path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0);;
  }
}

} // end namespace vision_utils

#endif // VECTOR2PATH_H

