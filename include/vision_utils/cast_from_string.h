/*!
  \file        cast_from_string.h
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

#ifndef CAST_FROM_STRING_H
#define CAST_FROM_STRING_H
// std includes
#include <string>

namespace vision_utils {

/*! cast a string to another type
 * \param in the string to cast
 * \param success true if we managed, false otherwise
 * \return the cast
 */
template<class _T>
_T cast_from_string(const std::string & in, bool & success) {
  std::istringstream myStream(in);
  _T ans;
  success = (myStream >> ans);
  return ans;
}
/*! cast a string to another type
 * \param in the string to cast
 * \return the cast
 */
template<class _T>
_T cast_from_string(const std::string & in) {
  bool success;
  return cast_from_string<_T> (in, success);
}

////////////////////// template specializations ////////////////////////////////

/*! a specialization for cast_from_string() */
template<> inline
const char* cast_from_string(const std::string & in, bool & success) {
  success = true;
  return in.c_str();
}

template<> inline
std::string cast_from_string(const std::string & in, bool & success) {
  success = true;
  return in;
}

} // end namespace vision_utils

#endif // CAST_FROM_STRING_H
