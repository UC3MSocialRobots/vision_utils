/*!
  \file        rotate_dilate_polygon.h
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

#ifndef ROTATE_DILATE_POLYGON_H
#define ROTATE_DILATE_POLYGON_H
// std includes
#include <vector>
#include "vision_utils/rotate_translate_polygon.h"

namespace vision_utils {

/*!
 * \brief   rotate a polygon from the angle theta around its center ,
 * with optionnaly a dilatation
     *
 * \param   theta the angle of rotation in radians
 * \param   dilat the factor of dilatation of the rectangle (1=no change)
 * \param   poly the list of points in the polygon
 */
template<class Point2>
static inline void rotate_dilate_polygon(std::vector<Point2> & poly,
                                         const double & theta,
                                         const double & dilat) {
  // determine center O of the polygon
  double ox = 0, oy = 0;
  for(typename std::vector<Point2>::const_iterator curr = poly.begin();
      curr != poly.end(); ++curr) {
    ox += curr->x;
    oy += curr->y;
  }
  ox = ox / poly.size();
  oy = oy / poly.size();
  //cout << "oX:" << ox << "\t oY:" << oy << endl;

  // apply the rotation and dilation
  for(typename std::vector<Point2>::iterator curr = poly.begin();
      curr != poly.end(); ++curr) {
    curr->x = dilat * ROTATE_ANGLE_X(curr->x - ox, curr->y - oy, theta) + ox;
    curr->y = dilat * ROTATE_ANGLE_Y(curr->x - ox, curr->y - oy, theta) + oy;
  }
}

} // end namespace vision_utils

#endif // ROTATE_DILATE_POLYGON_H
