/*!
  \file        accessible_to_string_nospaces.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/3
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

#ifndef ACCESSIBLE_TO_STRING_NOSPACES_H
#define ACCESSIBLE_TO_STRING_NOSPACES_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

/*! Convert a obj to a string, concatenating all values with no spaces
 \param obj
    anything that can be accesed using []
 \return std::string the representation of this obj
 */
template<class _Accessible>
inline std::string accessible_to_string_nospaces(const _Accessible & obj) {
  std::ostringstream ans_stream;
  for (unsigned int idx = 0; idx < obj.size(); ++idx)
    ans_stream << obj[idx];
  return ans_stream.str();
}

} // end namespace vision_utils

#endif // ACCESSIBLE_TO_STRING_NOSPACES_H
