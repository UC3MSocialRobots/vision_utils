/*!
  \file        cast_type_to_string.h
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

#ifndef CAST_TYPE_TO_STRING_H
#define CAST_TYPE_TO_STRING_H
// std includes
#include <string>

namespace vision_utils {

/*! cast a type to an explicit string
  \example string -> 'string' */
template<class _T>
inline std::string cast_type_to_string();

/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<bool>() {
  return "bool";
}
/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<char>() {
  return "char";
}
/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<short>() {
  return "short";
}
/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<int>() {
  return "int";
}
/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<long int>() {
  return "long int";
}
/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<float>() {
  return "float";
}
/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<double>() {
  return "double";
}
/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<std::string>() {
  return "std::string";
}
/*! a specialization for cast_type_to_string() */
template<> inline std::string cast_type_to_string<const char*>() {
  return "const char*";
}

} // end namespace vision_utils

#endif // CAST_TYPE_TO_STRING_H
