/*!
  \file        toggle_point_in_costmap.h
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

#ifndef TOGGLE_POINT_IN_COSTMAP_H
#define TOGGLE_POINT_IN_COSTMAP_H

namespace vision_utils {

/*!
 * Add a point if it is not in the costmap, remove it otherwise
 * \param pt
 *        a query point
 * \param costmap
 *         the costmap that needs to be modified
 * \return true if \arg pt is in the map after modification
 *         (ie it was not in the map before),
 *         false otherwise
 */
template<class _Pt2>
inline bool toggle_point_in_costmap(const _Pt2 & pt,
                                    nav_msgs::GridCells & costmap) {

  if (is_point_in_costmap(pt, costmap)) {
    remove_point_from_costmap(pt, costmap);
    return false;
  }
  add_point_to_costmap(pt, costmap);
  return true;
} // end if add_point_to_costmap()

} // end namespace vision_utils

#endif // TOGGLE_POINT_IN_COSTMAP_H
