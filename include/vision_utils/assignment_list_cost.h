/*!
  \file        assignment_list_cost.h
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

#ifndef ASSIGNMENT_LIST_COST_H
#define ASSIGNMENT_LIST_COST_H

namespace vision_utils {

Cost assignment_list_cost(const MatchList & list) {
  Cost total_cost = 0;
  unsigned int npairs = list.size();
  for (unsigned int pair_idx = 0; pair_idx < npairs; ++pair_idx)
    total_cost += list[pair_idx].cost;
  return total_cost;
}

} // end namespace vision_utils

#endif // ASSIGNMENT_LIST_COST_H
