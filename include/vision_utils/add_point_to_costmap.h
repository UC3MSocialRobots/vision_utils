/*!
  \file        add_point_to_costmap.h
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

#ifndef ADD_POINT_TO_COSTMAP_H
#define ADD_POINT_TO_COSTMAP_H

namespace vision_utils {

/*!
 * Add a point in a costmap, only if it is not included in it.
 * Do not modify the costmap otherwise.
 * \param pt
 *        a query point
 * \param costmap
 *         the costmap that needs to be modified
 * \return true if a new cell was effectively added,
 *         false if pt was actually already included in the costmap
 */
template<class _Pt2>
inline bool add_point_to_costmap(const _Pt2 & pt,
                                 nav_msgs::GridCells & costmap) {
  if (is_point_in_costmap(pt, costmap))
    return false;
  costmap.cells.push_back(closest_center_with_costmap_dims
                          (point_factory(pt.x, pt.y), costmap));
  return true;
} // end if add_point_to_costmap()

} // end namespace vision_utils

#endif // ADD_POINT_TO_COSTMAP_H
