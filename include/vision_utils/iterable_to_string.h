/*!
  \file        iterable_to_string.h
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

#ifndef ITERABLE_TO_STRING_H
#define ITERABLE_TO_STRING_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

/*! Convert a vector to a fancy string
 \param vector
    anything that can be accesed with iterators
 \return std::string the representation of this vector, separated by ;
 */
template<class _Iterable>
inline std::string iterable_to_string(const _Iterable & obj) {
  if (obj.size() == 0)
    return "{}";
  std::ostringstream ans_stream;
  ans_stream << "[";
  typename _Iterable::const_iterator it = obj.begin();
  while (it != obj.end()) {
    ans_stream << *it++;
    if (it!= obj.end())
      ans_stream << "; ";
  }
  ans_stream << "]";
  return ans_stream.str();
}

} // end namespace vision_utils

#endif // ITERABLE_TO_STRING_H
