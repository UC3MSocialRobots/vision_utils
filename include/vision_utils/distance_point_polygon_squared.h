/*!
  \file        distance_point_polygon_squared.h
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

#ifndef DISTANCE_POINT_POLYGON_SQUARED_H
#define DISTANCE_POINT_POLYGON_SQUARED_H
#include <stdio.h> // printf
#include <vector>
#include <vision_utils/distance_point_segment_sq.h>

namespace vision_utils {

/*!
 \brief   returns the squared min distance to the sides of the polygon
 (euclidean projection on the sides).
 \param pt
    the test point
 \param poly
    the list of points in the polygon
 \param closest pt
    the closest point in the polygon to pt
 \param closest_dist_sq
    the squared euclidean distance between pt and poly
 \param is_closed
    true if poly is closed
 */
template<class Point2>
static inline bool distance_point_polygon_squared
(const Point2 & pt,
 const std::vector<Point2> & poly,
 Point2 & closest_pt,
 double & closest_dist_sq,
 bool is_poly_closed = true) {
  // do nothing if polygon empty
  if (poly.empty()) {
    printf("distance_point_polygon_squared() should not be called with an empty polygon");
    closest_dist_sq = 0;
    closest_pt = pt;
    return false;
  }

  closest_dist_sq = std::numeric_limits<double>::max();
  closest_pt = poly.front();
  Point2 curr_closest_pt;
  unsigned int poly_size = poly.size(), max_pt_idx = poly_size - 1;
  for (unsigned int pt_idx = 0; pt_idx < max_pt_idx; ++pt_idx) {
    double curr_dist_sq = distance_point_segment_sq
                          (pt, poly[pt_idx], poly[pt_idx + 1], curr_closest_pt);
    if (closest_dist_sq > curr_dist_sq) {
      closest_dist_sq = curr_dist_sq;
      closest_pt = curr_closest_pt;
    }
  } // end loop pt_idx

  // check the last segment if the polygon is closed
  if (!is_poly_closed)
    return true;
  double curr_dist_sq = distance_point_segment_sq
                        (pt, poly.back(), poly.front(), curr_closest_pt);
  if (closest_dist_sq > curr_dist_sq) {
    closest_dist_sq = curr_dist_sq;
    closest_pt = curr_closest_pt;
  }
  return true;
} // end distance_point_polygon

} // end namespace vision_utils

#endif // DISTANCE_POINT_POLYGON_SQUARED_H
