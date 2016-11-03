/*!
  \file        printP4.h
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

#ifndef PRINTP4_H
#define PRINTP4_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

/*!
 * print a 4D point in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Point3>
static inline std::string printP4(const Point3 & p) {
  std::ostringstream ans;
  ans << "(" << p.x << ", " << p.y << ", " << p.z << ", " << p.w << ")";
  return ans.str();
}

} // end namespace vision_utils

#endif // PRINTP4_H
