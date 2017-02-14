/*!
  \file        remove_point_from_costmap.h
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

#ifndef REMOVE_POINT_FROM_COSTMAP_H
#define REMOVE_POINT_FROM_COSTMAP_H

#include "vision_utils/point_factory.h"
#include "vision_utils/closest_center_with_costmap_dims.h"

namespace vision_utils {

/*!
 * remove a point in a costmap, only if it is not included in it.
 * Do not modify the costmap otherwise.
 * \param pt
 *        a query point
 * \param costmap
 *         the costmap that needs to be modified
 * \return true if a new cell was effectively removed,
 *         false if pt was not in the costmapz
 */
template<class _Pt2>
inline bool remove_point_from_costmap(const _Pt2 & pt,
                                      nav_msgs::GridCells & costmap) {
  geometry_msgs::Point center = closest_center_with_costmap_dims
                                (point_factory(pt.x, pt.y), costmap);
  bool was_pt_erased = false;
  for (unsigned int pt_idx = 0; pt_idx < costmap.cells.size(); ++pt_idx) {

    if (center.x == costmap.cells[pt_idx].x &&
        center.y == costmap.cells[pt_idx].y) {
      costmap.cells.erase(costmap.cells.begin() + pt_idx);
      --pt_idx;
      was_pt_erased = true;
    }
  }
  return was_pt_erased;
} // end if remove_point_from_costmap()

} // end namespace vision_utils

#endif // REMOVE_POINT_FROM_COSTMAP_H
