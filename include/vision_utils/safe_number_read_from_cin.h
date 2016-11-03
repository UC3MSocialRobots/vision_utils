/*!
  \file        safe_number_read_from_cin.h
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

#ifndef SAFE_NUMBER_READ_FROM_CIN_H
#define SAFE_NUMBER_READ_FROM_CIN_H
// std includes
#include <string>

namespace vision_utils {

/*!
  Read number values from cin.
  It is templated so it can read ints, doubles, etc.
  \return true if success
  */
template<class _T>
inline bool safe_number_read_from_cin(_T & dst) {
  // read line
  std::string line;
  std::getline(std::cin, line);
  // cast it to _T
  std::istringstream myStream(line);
  bool success = (myStream >> dst);
  return success;
} // end safe_number_read_from_cin()

} // end namespace vision_utils

#endif // SAFE_NUMBER_READ_FROM_CIN_H
