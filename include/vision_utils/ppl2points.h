/*!
  \file        ppl2points.h
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

#ifndef PPL2POINTS_H
#define PPL2POINTS_H
// std includes
#include <vector>
#include <people_msgs/People.h>

namespace vision_utils {

template<class Pt3>
inline std::vector<Pt3> ppl2points(const people_msgs::People & ppl) {
  unsigned int nusers = ppl.people.size();
  std::vector<Pt3> ans(nusers);
  for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
    const geometry_msgs::Point *src = &(ppl.people[user_idx].position);
    Pt3 *dst = &(ans[user_idx]);
    dst->x = src->x;
    dst->y = src->y;
    dst->z = src->z;
  }
  return ans;
}

} // end namespace vision_utils

#endif // PPL2POINTS_H
