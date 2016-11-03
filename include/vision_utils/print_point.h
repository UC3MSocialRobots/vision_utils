/*!
  \file        print_point.h
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

#ifndef PRINT_POINT_H
#define PRINT_POINT_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

template<class _T>
inline std::string print_point(const _T & pt) {
  std::ostringstream out;
  out << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")";
  return out.str();
}

} // end namespace vision_utils

#endif // PRINT_POINT_H
