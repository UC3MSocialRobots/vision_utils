/*!
  \file        ppl2string.h
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

#ifndef PPL2STRING_H
#define PPL2STRING_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

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

} // end namespace vision_utils

#endif // PPL2STRING_H
