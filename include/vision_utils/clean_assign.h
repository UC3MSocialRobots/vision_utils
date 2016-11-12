/*!
  \file        clean_assign.h
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

#ifndef CLEAN_ASSIGN_H
#define CLEAN_ASSIGN_H
#include <vision_utils/match.h>

namespace vision_utils {

//! remove the unassigned matches in a MatchList
inline void clean_assign(MatchList & assign) {
  for (unsigned int i = 0; i < assign.size(); ++i) {
    if (assign[i].first != UNASSIGNED
        && assign[i].second != UNASSIGNED)
      continue;
    assign.erase(assign.begin() + i);
    --i;
  } // end for i
} // end clean_assign();

} // end namespace vision_utils

#endif // CLEAN_ASSIGN_H
