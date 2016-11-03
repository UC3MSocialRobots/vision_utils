/*!
  \file        cast_to_string.h
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

#ifndef CAST_TO_STRING_H
#define CAST_TO_STRING_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

/*! cast any type to string
 * \param in the value to cast
 * \param success true if we managed, false otherwise
 * \return the cast
 */
template<class _T>
std::string cast_to_string(const _T in, bool & success) {
  std::ostringstream ans;
  success = (ans << in);
  return ans.str();
}

/*! cast any type to string
 * \param in the value to cast
 * \return the cast
 */
template<class _T>
std::string cast_to_string(const _T in) {
  bool success;
  return cast_to_string<_T> (in, success);
}

} // end namespace vision_utils

#endif // CAST_TO_STRING_H
