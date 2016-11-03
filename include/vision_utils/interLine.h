/*!
  \file        interLine.h
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

#ifndef INTERLINE_H
#define INTERLINE_H

namespace vision_utils {

/*!
 * \brief   computes the intersection of two lines
 * of equations a_i* x + b_i * y + c_i = 0
 */
template<class Point2>
static inline Point2 interLine(const double a1, const double b1, const double c1,
                               const double a2, const double b2, const double c2) {
  double x = 1.f * (+c1 * b2 - c2 * b1) / (b1 * a2 - a1 * b2);
  double y = 1.f * (-c1 * a2 + c2 * a1) / (b1 * a2 - a1 * b2);
  return Point2(x, y);
}

} // end namespace vision_utils

#endif // INTERLINE_H
