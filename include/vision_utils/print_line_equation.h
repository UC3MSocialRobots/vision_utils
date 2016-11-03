/*!
  \file        print_line_equation.h
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

#ifndef PRINT_LINE_EQUATION_H
#define PRINT_LINE_EQUATION_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

/**
 * @brief  return a string representing a line equation,
 * \example 2 x + 3 y + 5 = 0
 */
template<class Point3>
static inline std::string print_line_equation(Point3 line_eq) {
  std::ostringstream out;
  if (fabs(line_eq.x) > 1E-4) {
    if (line_eq.x != 1)
      out << line_eq.x << " ";
    out << "x";
  } // end x != 0

  if (fabs(line_eq.y) > 1E-4) {
    out << (out.str().size() > 0 ? " + " : "");
    if (line_eq.y != 1)
      out << line_eq.y << " ";
    out << "y";
  } // end y != 0

  if (fabs(line_eq.z) > 1E-4)
    out << " + " << line_eq.z;

  out << " = 0";
  return out.str();

}

} // end namespace vision_utils

#endif // PRINT_LINE_EQUATION_H
