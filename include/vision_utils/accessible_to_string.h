/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/11/3
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

  odo Description of the file
 */

#ifndef ACCESSIBLE_TO_STRING_H
#define ACCESSIBLE_TO_STRING_H
#include <sstream>

namespace vision_utils {

//cut:accessible_to_string
/*! Convert a obj to a string, concatenating all values
 \param obj
    anything that can be accesed using []
 \return std::string the representation of this obj
 */
template<class _Accessible>
inline std::string accessible_to_string(const _Accessible & obj) {
  if (obj.size() == 0)
    return "{}";
  std::ostringstream ans_stream;
  ans_stream << "[";
  for (unsigned int idx = 0; idx < obj.size(); ++idx)
    ans_stream << obj[idx] <<  (idx < obj.size() - 1 ? "; " : "");
  ans_stream << "]";
  return ans_stream.str();
}

} // end namespace vision_utils

#endif // ACCESSIBLE_TO_STRING_H

