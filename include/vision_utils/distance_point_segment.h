/*!
  \file        distance_point_segment.h
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

#ifndef DISTANCE_POINT_SEGMENT_H
#define DISTANCE_POINT_SEGMENT_H

#include "vision_utils/distance_points.h"

namespace vision_utils {

/*!
 return the distance to a segment
 \param pt
 \param seg1
 \param seg2
 \return double
*/
template<class Point2>
static inline double distance_point_segment(const Point2 & pt,
                                            const Point2 & seg1,
                                            const Point2 & seg2) {
  // compute the projection of P3 on [P1, P2]
  Point2 diff (seg2.x - seg1.x, seg2.y - seg1.y);
  double u = 1.f * ((pt.x - seg1.x) * diff.x + (pt.y - seg1.y) * diff.y)
             / (diff.x * diff.x + diff.y * diff.y);
  //printf("p_curr:%s, p_next:%s, u:%g", printP2(seg1).c_str(), printP2(seg2).c_str(), u);

  if (u < 0) // the projection is before seg1
    return distance_points(pt, seg1);
  else if (u > 1) // the projection is after seg2
    return distance_points(pt, seg2);
  else { // proj on the segment
    return distance_points(pt, Point2
                           ( // the proj
                             u * seg2.x + (1 - u) * seg1.x,
                             u * seg2.y + (1 - u) * seg1.y
                             ));
    //cout << " proj:" << proj.x << "," << proj.y << endl;
  }
}

} // end namespace vision_utils

#endif // DISTANCE_POINT_SEGMENT_H
