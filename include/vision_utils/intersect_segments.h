/*!
  \file        intersect_segments.h
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

#ifndef INTERSECT_SEGMENTS_H
#define INTERSECT_SEGMENTS_H

namespace vision_utils {

/*!
 determines if two line segments AB and CD intersect.
 These intersect if and only if
 points A and B are separated by segment CD and
 points C and D are separated by segment AB.

 If points A and B are separated by segment CD
 then ACD and BCD should have opposite orientation
 meaning either ACD or BCD is counterclockwise but not both

 From http://www.bryceboe.com/2006/10/23/line-segment-intersection-algorithm/

 \return true if they intersect
*/
template<class Point2>
static inline bool intersect_segments(const Point2 & A, const Point2 & B,
                                      const Point2 & C, const Point2 & D) {
  // one common end
  if (A == C || A == D || B == C || B == D)
    return true;

  // check parallel segments
  if (colinear(A, B, C) && colinear(A, B, D)) {
    if
        (// C in AB
         (distance_points_squared(A, C) <= distance_points_squared(A, B)
          && distance_points_squared(B, C) <= distance_points_squared(A, B))
         || // D in AB
         (distance_points_squared(A, D) <= distance_points_squared(A, B)
          && distance_points_squared(B, D) <= distance_points_squared(A, B))
         || // A in CD
         (distance_points_squared(C, A) <= distance_points_squared(C, D)
          && distance_points_squared(D, A) <= distance_points_squared(C, D))
         || // B in CD
         (distance_points_squared(C, B) <= distance_points_squared(C, D)
          && distance_points_squared(D, B) <= distance_points_squared(C, D))
         )
      return true;
    else
      return false;
  }

  return (counter_clockwise(A,C,D) != counter_clockwise(B,C,D))
      && (counter_clockwise(A,B,C) != counter_clockwise(A,B,D));
}

} // end namespace vision_utils

#endif // INTERSECT_SEGMENTS_H
