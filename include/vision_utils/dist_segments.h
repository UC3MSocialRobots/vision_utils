/*!
  \file        dist_segments.h
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

#ifndef DIST_SEGMENTS_H
#define DIST_SEGMENTS_H

namespace vision_utils {

template<class Point2>
static inline double dist_segments(const Point2 & A, const Point2 & B,
                                   const Point2 & C, const Point2 & D) {
  if (intersect_segments(A, B, C, D))
    return 0;
  //    //printf("%g, %g, %g, %g",
  //                 distance_point_segment(A, C, D), distance_point_segment(B, C, D),
  //                 distance_point_segment(C, A, B), distance_point_segment(D, A, B));
  return fmin(
        fmin(distance_point_segment(A, C, D), distance_point_segment(B, C, D)),
        fmin(distance_point_segment(C, A, B), distance_point_segment(D, A, B))
        );
}

} // end namespace vision_utils

#endif // DIST_SEGMENTS_H
