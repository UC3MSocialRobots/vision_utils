/*!
  \file        closest_center_with_costmap_dims.h
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

#ifndef CLOSEST_CENTER_WITH_COSTMAP_DIMS_H
#define CLOSEST_CENTER_WITH_COSTMAP_DIMS_H

namespace vision_utils {

/*!
 * Gives the center of a cell that would be the closest from a query point
 * (even if this cell is actually not part of the map).
 * \param pt
 * \param map
 */
template<class _Pt2>
inline _Pt2 closest_center_with_costmap_dims(const _Pt2 & pt,
                                             const nav_msgs::GridCells & map) {
  _Pt2 ans;
  ans.x = map.cell_width * floor(pt.x / map.cell_width) + map.cell_width / 2;
  ans.y = map.cell_height * floor(pt.y / map.cell_height) + map.cell_height / 2;
  return ans;
} // end closest_center_with_costmap_dims()

} // end namespace vision_utils

#endif // CLOSEST_CENTER_WITH_COSTMAP_DIMS_H
