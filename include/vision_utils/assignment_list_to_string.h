/*!
  \file        assignment_list_to_string.h
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

#ifndef ASSIGNMENT_LIST_TO_STRING_H
#define ASSIGNMENT_LIST_TO_STRING_H
// std includes
#include <sstream> // for ostringstream
#include <string>
#include "vision_utils/match.h"

namespace vision_utils {

//! convert an AssignmentList to a string, for displaying and debugging
inline std::string assignment_list_to_string(const MatchList & list) {
  std::ostringstream out;
  unsigned int npairs = list.size();
  out << "AssignmentList:" << npairs
      << " pairs, total cost:" << assignment_list_cost(list) << ": ";
  for (unsigned int pair_idx = 0; pair_idx < npairs; ++pair_idx)
    out << list[pair_idx].first << "->" << list[pair_idx].second
        << " (" << std::setprecision(3) << list[pair_idx].cost << "); ";
  return out.str();
} // end assignment_list_to_string()

} // end namespace vision_utils

#endif // ASSIGNMENT_LIST_TO_STRING_H
