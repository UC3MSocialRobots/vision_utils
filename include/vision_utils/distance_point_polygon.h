/*!
  \file        distance_point_polygon.h
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

#ifndef DISTANCE_POINT_POLYGON_H
#define DISTANCE_POINT_POLYGON_H
#include <vector>

namespace vision_utils {

/*!
 * \brief   returns the min distance to the sides of the polygon
 * (euclidean projection on the sides).
 * Cf http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/
     *
 * \param   pt the test point
 * \param   poly the list of points in the polygon
 */
template<class Point2>
static inline double distance_point_polygon(const Point2 & pt,
                                            const std::vector<Point2> & poly,
                                            bool is_poly_closed = true) {
  double minDist = std::numeric_limits<double>::max();//, currDist, u;

  typename std::vector<Point2>::const_iterator p_curr = poly.begin(),
      p_next = poly.begin() + 1;

  for (unsigned int i = 1;
       i <= (is_poly_closed? poly.size() : poly.size() - 1);
       ++i) {

    minDist = fmin(minDist, distance_point_segment(pt, *p_curr, *p_next));

    ++p_curr;
    ++p_next;
    if (is_poly_closed && i == poly.size() - 1)
      p_next = poly.begin();
  } // end loop i

  return minDist;
}

} // end namespace vision_utils

#endif // DISTANCE_POINT_POLYGON_H
