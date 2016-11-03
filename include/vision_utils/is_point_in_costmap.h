/*!
  \file        is_point_in_costmap.h
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

#ifndef IS_POINT_IN_COSTMAP_H
#define IS_POINT_IN_COSTMAP_H

namespace vision_utils {

/*!
 * \brief is_point_in_costmap
 * \return true fif \arg pt is in one of the cells of \arg costmap
 */
template<class _Pt2>
inline bool is_point_in_costmap(const _Pt2 & pt,
                                const nav_msgs::GridCells & costmap) {
  double halfwidth = costmap.cell_width / 2;
  double halfheight = costmap.cell_height / 2;
  for (unsigned int cell_idx = 0; cell_idx < costmap.cells.size(); ++cell_idx) {
    //const geometry_msgs::Point & curr_cell = costmap.cells[cell_idx];
    if (fabs(pt.x - costmap.cells[cell_idx].x ) <= halfwidth
        && fabs(pt.y - costmap.cells[cell_idx].y ) <= halfheight) {
      //      //printf("Pt (%g, %g) is within the cell of index %i:(%g, %g)+/-(%g, %g)",
      //               pt.x, pt.y, cell_idx,
      //               costmap.cells[cell_idx].x, costmap.cells[cell_idx].y,
      //               costmap.cell_width / 2, costmap.cell_height/ 2);
      return true;
    }
  } // end loop cell_idx
  return false;
} // end if is_point_in_costmap()

} // end namespace vision_utils

#endif // IS_POINT_IN_COSTMAP_H
