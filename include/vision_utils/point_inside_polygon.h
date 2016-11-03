/*!
  \file        point_inside_polygon.h
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

#ifndef POINT_INSIDE_POLYGON_H
#define POINT_INSIDE_POLYGON_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 * \brief   detect if a point is inside a polygon - return true or false
 *  http://en.wikipedia.org/wiki/Point_in_polygon#Winding_number_algorithm
     *
 * \param   p the point
 * \param   poly the polygon
 * \return  true if the point is in the polygon
 */
template<class Point2>
static inline bool point_inside_polygon(const Point2 & p,
                                        const std::vector<Point2> & poly) {
#if 0 // old implementation
  double sum_angles = 0;
  double angle;
  const Point2 *p_curr = &poly.at(0], *p_next = &poly.at(0];
                                                p_next++;

                         for (unsigned int i = 1; i <= poly.size(); ++i) {
    Point2 v1 (p.x - p_curr->x, p.y - p_curr->y);
    Point2 v2 (p.x - p_next->x, p.y - p_next->y);
    if ((v1.x == 0 && v1.y == 0) || (v2.x == 0 && v2.y == 0))
      return true;
    angle = acos(
              // dot product
              1.f * v1.x * v2.x + 1.f * v1.y * v2.y
              /
              (hypot(v1.x, v1.y) * hypot(v2.x, v2.y)));
    if (1.f * v1.x * v2.y - 1.f * v1.y * v2.x < 0)
      angle = -angle; // use of the vectorial product
    sum_angles += angle;

    ++p_curr;
    ++p_next;
    if (i == poly.size() - 1)
      p_next = &poly.at(0];
  }

  //cout << " p:" << p.x << "," << p.y << " - sum:" << sum_angles << endl;
  if (sum_angles < PI && sum_angles > -PI)
    return 0;
  return 1;
#else
  /*
     * algo from http://www.visibone.com/inpoly/
     */
  Point2 p_old, p_new, p1, p2;
  bool inside = false;
  int npoints = poly.size();

  if (npoints < 3) {
    return(0);
  }
  p_old = poly[npoints-1];

  for (int i=0 ; i < npoints ; i++) {
    p_new = poly[i];
    if (p_new.x > p_old.x) {
      p1 = p_old;
      p2 = p_new;
    }
    else {
      p1 = p_new;
      p2 = p_old;
    }
    if ((p_new.x < p.x) == (p.x <= p_old.x)          /* edge "open" at one end */
        && 1.f * (p.y-p1.y) * (p2.x-p1.x) < 1.f * (p2.y-p1.y) * (p.x-p1.x)) {
      inside = !inside;
    }
    p_old.x = p_new.x;
    p_old.y = p_new.y;
  } // end loop i
  return(inside);
#endif
}

} // end namespace vision_utils

#endif // POINT_INSIDE_POLYGON_H
