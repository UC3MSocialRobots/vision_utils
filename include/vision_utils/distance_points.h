/*!
  \file        distance_points.h
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

#ifndef DISTANCE_POINTS_H
#define DISTANCE_POINTS_H

namespace vision_utils {

/*!
  \return the distance AB
  */
template<class Point2_A, class Point2_B>
static inline double
distance_points(const Point2_A & A, const Point2_B & B) {
  return hypot(A.x - B.x, A.y - B.y);
}

} // end namespace vision_utils

#endif // DISTANCE_POINTS_H
