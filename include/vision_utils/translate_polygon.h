/*!
  \file        translate_polygon.h
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

#ifndef TRANSLATE_POLYGON_H
#define TRANSLATE_POLYGON_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 * \brief   translates a polygon
 * \param   poly_in the list of points in the polygon
 * \param   translation the vector among which we translate
 */
template<class Point2>
static inline void translate_polygon(std::vector<Point2> & poly,
                                     const Point2 & translation) {
  for(typename std::vector<Point2>::iterator pt = poly.begin();
      pt != poly.end(); ++pt) {
    pt->x += translation.x;
    pt->y += translation.y;
  } // end loop pt
}

} // end namespace vision_utils

#endif // TRANSLATE_POLYGON_H
