/*!
  \file        ppl2names.h
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

#ifndef PPL2NAMES_H
#define PPL2NAMES_H
// std includes
#include <string>
#include <vector>

namespace vision_utils {

inline std::vector<std::string> ppl2names(const people_msgs::People & ppl) {
  unsigned int nusers = ppl.people.size();
  std::vector<std::string> ans(nusers);
  for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx)
    ans[user_idx] = ppl.people[user_idx].name;
  return ans;
}

} // end namespace vision_utils

#endif // PPL2NAMES_H
