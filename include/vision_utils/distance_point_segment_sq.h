/*!
  \file        distance_point_segment_sq.h
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

#ifndef DISTANCE_POINT_SEGMENT_SQ_H
#define DISTANCE_POINT_SEGMENT_SQ_H
#include <vision_utils/distance_points_squared.h>

namespace vision_utils {

/*!
 return the squared euclididan distance to a segment
 \param pt
    the point whose distance to the segment we want to compute
 \param seg1
   the first end of the segment
 \param seg2
   the second end of the segment
 \param proj
  the corresponding projection on the segment
 \return double
    ths squared euclidean distance between pt and [seg1, seg2]
*/
template<class Point2>
static inline double distance_point_segment_sq(const Point2 & pt,
                                               const Point2 & seg1,
                                               const Point2 & seg2,
                                               Point2 & proj) {
  // compute the projection of P3 on [P1, P2]
  Point2 diff;
  diff.x = seg2.x - seg1.x;
  diff.y = seg2.y - seg1.y;
  double u = 1.f * ((pt.x - seg1.x) * diff.x + (pt.y - seg1.y) * diff.y)
             / (diff.x * diff.x + diff.y * diff.y);

  if (u < 0) { // the projection is before seg1
    proj.x = seg1.x;
    proj.y = seg1.y;
  }
  else if (u > 1) { // the projection is after seg2
    proj.x = seg2.x;
    proj.y = seg2.y;
  }
  else { // proj on the segment
    proj.x = u * seg2.x + (1 - u) * seg1.x;
    proj.y = u * seg2.y + (1 - u) * seg1.y;
  }
  return distance_points_squared(pt, proj);
}

} // end namespace vision_utils

#endif // DISTANCE_POINT_SEGMENT_SQ_H
