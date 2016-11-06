/*!
  \file        is_trajectory_collision_free.h
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

#ifndef IS_TRAJECTORY_COLLISION_FREE_H
#define IS_TRAJECTORY_COLLISION_FREE_H
// std includes
#include <vector>
#include "vision_utils/is_point_in_costmap.h"

namespace vision_utils {

template<class _Pt2>
inline bool is_trajectory_collision_free(const _Pt2 & start_pos, const float & start_yaw,
                                         const nav_msgs::GridCells & costmap,
                                         const float & time_end, const float & dt,
                                         const float & vel_lin, const float & vel_ang,
                                         std::vector<_Pt2> & traj_buffer)
{
  make_trajectory(vel_lin, vel_ang, traj_buffer, time_end, dt,
                              start_pos.x, start_pos.y, start_yaw);
  for (unsigned int traj_pt_idx = 0; traj_pt_idx < traj_buffer.size(); ++traj_pt_idx) {
    if (is_point_in_costmap<_Pt2>(traj_buffer[traj_pt_idx], costmap))
      return false;
  } // end loop traj_pt_idx
  return true;
} // end is_trajectory_collision_free()

} // end namespace vision_utils

#endif // IS_TRAJECTORY_COLLISION_FREE_H
