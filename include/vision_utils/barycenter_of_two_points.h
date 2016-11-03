/*!
  \file        barycenter_of_two_points.h
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

#ifndef BARYCENTER_OF_TWO_POINTS_H
#define BARYCENTER_OF_TWO_POINTS_H

namespace vision_utils {

/*!
 * \brief   returns alpha * A + (1 - alpha) * B
 * \param alpha : if 0, returns B, if 1, returns A
 */
template<class Point2>
static inline Point2 barycenter_of_two_points(const double alpha,
                                              const Point2 & A,
                                              const Point2 & B) {
  return Point2 (alpha * A.x + (1.f - alpha) * B.x,
                 alpha * A.y + (1.f - alpha) * B.y);

}

} // end namespace vision_utils

#endif // BARYCENTER_OF_TWO_POINTS_H
