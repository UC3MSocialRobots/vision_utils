/*!
  \file        costmap_to_string.h
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

#ifndef COSTMAP_TO_STRING_H
#define COSTMAP_TO_STRING_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

std::string costmap_to_string(const nav_msgs::GridCells & costmap) {
  std::ostringstream ans;
  ans << "w:" << costmap.cell_width << ", h:" << costmap.cell_height << ", ";
  ans << costmap.cells.size() << " cells: ";
  for (unsigned int cell_idx = 0; cell_idx < costmap.cells.size(); ++cell_idx)
    ans << "(" << costmap.cells[cell_idx].x
        << ", " << costmap.cells[cell_idx].y << "); ";
  return ans.str();
} // end costmap_to_string()

} // end namespace vision_utils

#endif // COSTMAP_TO_STRING_H
