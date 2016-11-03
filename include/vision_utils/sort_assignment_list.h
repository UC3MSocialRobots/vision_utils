/*!
  \file        sort_assignment_list.h
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

#ifndef SORT_ASSIGNMENT_LIST_H
#define SORT_ASSIGNMENT_LIST_H

namespace vision_utils {

void sort_assignment_list(MatchList & list) {
  std::sort(list.begin(), list.end());
} // end reverse_assignment_list()

} // end namespace vision_utils

#endif // SORT_ASSIGNMENT_LIST_H
