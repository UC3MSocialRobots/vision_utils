/*!
  \file        counter_clockwise.h
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

#ifndef COUNTER_CLOCKWISE_H
#define COUNTER_CLOCKWISE_H

namespace vision_utils {

/*!
 determining if three points are listed in a counterclockwise order.
 So say you have three points A, B and C.
 If the slope of the line AC is greater than the slope of the line AB
 then the three points are listed in a counterclockwise order.

 From http://www.bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
 \param A
 \param B
 \param C
 \return bool
*/
template<class Point2>
static inline bool counter_clockwise(const Point2 & A,
                                     const Point2 & B,
                                     const Point2 & C) {
  return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x);
}

} // end namespace vision_utils

#endif // COUNTER_CLOCKWISE_H
