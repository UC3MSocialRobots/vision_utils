/*!
  \file        trajectory_mark.h
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

#ifndef TRAJECTORY_MARK_H
#define TRAJECTORY_MARK_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 \param start_pos
 \param start_yaw
 \param goal
 \param costmap
 \param time_end
 \param dt
 \param vel_lin
 \param vel_ang
 \param traj_buffer
 \return float
    infinity if collision,
    otherwise the L2 distance in meters between the trajectory and the goal
*/
template<class _Pt2>
inline float trajectory_mark(const _Pt2 & start_pos, const float & start_yaw,
                             const _Pt2 & goal,
                             const nav_msgs::GridCells & costmap,
                             const float & time_end, const float & dt,
                             const float & vel_lin, const float & vel_ang,
                             std::vector<_Pt2> & traj_buffer) {
  // return infinity if there is a collision
  if (!is_trajectory_collision_free(start_pos, start_yaw, costmap,
                                    time_end, dt, vel_lin, vel_ang, traj_buffer))
    return std::numeric_limits<float>::infinity();

  // evaluate distance to the goal - best point from the set
  //  return dist_pt_set
  //      (goal, traj_buffer, dist_L2);

  // evaluate distance to the goal - last point
  return dist_L2(goal, traj_buffer.back());
} // end trajectory_mark()

} // end namespace vision_utils

#endif // TRAJECTORY_MARK_H
