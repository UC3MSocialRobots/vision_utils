/*!
  \file        people_pose_list_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/26

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

Some useful functions for processing People messages.

 */

#ifndef vision_utils_H
#define vision_utils_H

// msg
#include <people_msgs/People.h>
#include <sstream>

namespace vision_utils {
//cut:ppl2points
template<class Pt3>
inline std::vector<Pt3> ppvec2points(const people_msgs::People & ppvec) {
  unsigned int nusers = ppvec.size();
  std::vector<Pt3> ans(nusers);
  for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
    const geometry_msgs::Point *src = &(ppvec[user_idx].position);
    Pt3 *dst = &(ans[user_idx]);
    dst->x = src->x;
    dst->y = src->y;
    dst->z = src->z;
  }
  return ans;
}

////////////////////////////////////////////////////////////////////////////////
template<class Pt3>
inline std::vector<Pt3> ppl2points(const people_msgs::People & ppl) {
  return ppvec2points<Pt3>(ppl.people);
}

////////////////////////////////////////////////////////////////////////////////
//cut:ppl2names
inline std::vector<std::string> ppl2names(const people_msgs::People & ppl) {
  unsigned int nusers = ppl.people.size();
  std::vector<std::string> ans(nusers);
  for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx)
    ans[user_idx] = ppl.people[user_idx].name;
  return ans;
}

////////////////////////////////////////////////////////////////////////////////
//cut:ppl2string
std::string pp2string(const people_msgs::Person & pp,
                      unsigned int precision = 3,
                      bool print_last_update = true) {
  std::ostringstream out;
  out << "Track '" << pp.name << "':("
      << std::setprecision(precision) << pp.position.x << ","
      << std::setprecision(precision) << pp.position.y << ","
      << std::setprecision(precision) << pp.position.z
      << ")@" << pp.header.frame_id;
  if (print_last_update)
    out << ",confidence:" << std::setprecision(precision) << pp.reliability
        << ",updated "
        << (int) (1000*(ros::Time::now() - pp.header.stamp).toSec()) << " ms ago";
  unsigned int nattr = pp.tagnames.size();
  if (nattr == 0 || nattr != pp.tags.size())
    return out.str();
  out << ", attributes:";
  for (unsigned int attr = 0; attr < nattr; ++attr) {
    out << "" << pp.tagnames[attr] << "='"
        << pp.tags[attr] << "';";
  } // end for attr
  return out.str();
}

////////////////////////////////////////////////////////////////////////////////

std::string ppl2string(const people_msgs::People & ppl,
                       unsigned int precision = 3,
                       bool print_last_update = true) {
  if (ppl.people.empty())
    return "empty PPL";
  std::ostringstream out;
  if (ppl.people.size() == 1) {
    out << "PPL:{1 PP=" << pp2string(ppl.people.front(), precision, print_last_update) << "}";
    return out.str();
  }
  out << "PPL (" << ppl.people.size() << " PPs):{";
  for (unsigned int user = 0; user < ppl.people.size(); ++user)
    out << std::endl << "  " << pp2string(ppl.people[user], precision, print_last_update);
  out << std::endl << "}";
  return out.str();
}

} // end namespace vision_utils

#endif // vision_utils_H
