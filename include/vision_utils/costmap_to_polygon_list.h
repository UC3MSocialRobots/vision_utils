/*!
  \file        costmap_to_polygon_list.h
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

#ifndef COSTMAP_TO_POLYGON_LIST_H
#define COSTMAP_TO_POLYGON_LIST_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 * \brief costmap_to_polygon_list
 * \param costmap
 * \param out
 *    points in this order:
 *  y ^
 *    |  3   2
 *    |    x
 *    |  0   1
 *   0+---------> x
 */
template<class _Pt3>
void costmap_to_polygon_list(const nav_msgs::GridCells & costmap,
                             std::vector<_Pt3> & out) {
  double halfwidth = costmap.cell_width / 2;
  double halfheight = costmap.cell_height / 2;
  out.clear();
  out.reserve(costmap.cells.size() * 4);
  _Pt3 corner;
  for (unsigned int cell_idx = 0; cell_idx < costmap.cells.size(); ++cell_idx) {
    // A B
    // C D
    // add C
    corner.x = costmap.cells[cell_idx].x - halfwidth;
    corner.y = costmap.cells[cell_idx].y - halfheight;
    corner.z = costmap.cells[cell_idx].z;
    out.push_back(corner);
    // add D
    corner.x = costmap.cells[cell_idx].x + halfwidth;
    out.push_back(corner);
    // add B
    corner.y = costmap.cells[cell_idx].y + halfheight;
    out.push_back(corner);
    // add A
    corner.x = costmap.cells[cell_idx].x - halfwidth;
    out.push_back(corner);
  } // end loop cell_idx
} // end costmap_to_string()

} // end namespace vision_utils

#endif // COSTMAP_TO_POLYGON_LIST_H
