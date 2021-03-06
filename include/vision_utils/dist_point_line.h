/*!
  \file        dist_point_line.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/5
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

#ifndef DIST_POINT_LINE_H
#define DIST_POINT_LINE_H

namespace vision_utils {

/*!
  \return the projection of P on the line ax + by + c = 0
  */
template<class Point2>
static inline double
dist_point_line(const Point2 & P,
                const double & a, const double & b, const double & c) {
  return distance_points(P, proj_point_on_line(P, a, b, c));
}

} // end namespace vision_utils

#endif // DIST_POINT_LINE_H
