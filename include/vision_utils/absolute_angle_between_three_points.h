/*!
  \file        absolute_angle_between_three_points.h
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

#ifndef ABSOLUTE_ANGLE_BETWEEN_THREE_POINTS_H
#define ABSOLUTE_ANGLE_BETWEEN_THREE_POINTS_H

#include "vision_utils/absolute_angle_between_two_vectors.h"

namespace vision_utils {

/*!
 * \brief   returns the angle between three points A, B, C,
 * that is the angle between the vectors BA and BC - between 0 and PI
 */
template<class Point2>
static inline double absolute_angle_between_three_points(const Point2 & A,
                                                         const Point2 & B,
                                                         const Point2 & C) {
  return absolute_angle_between_two_vectors(
        Point2 (A.x - B.x, A.y - B.y),  // BA
        Point2 (C.x - B.x, C.y - B.y) //   BC
        );
}

} // end namespace vision_utils

#endif // ABSOLUTE_ANGLE_BETWEEN_THREE_POINTS_H
