/*!
  \file        rotate_translate_polygon.h
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

#ifndef ROTATE_TRANSLATE_POLYGON_H
#define ROTATE_TRANSLATE_POLYGON_H
// std includes
#include <vector>

namespace vision_utils {

#define ROTATE_COSSIN_X(x, y, cos_angle, sin_angle) \
  (cos_angle * (x) - sin_angle * (y))
#define ROTATE_COSSIN_Y(x, y, cos_angle, sin_angle) \
  (sin_angle * (x) + cos_angle * (y))
#define ROTATE_ANGLE_X(x, y, angle) \
  (ROTATE_COSSIN_X(x, y, cos(angle), sin(angle)) )
#define ROTATE_ANGLE_Y(x, y, angle) \
  (ROTATE_COSSIN_Y(x, y, cos(angle), sin(angle)) )


////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   rotate a polygon from the an agle
 * and then apply a translation
 * \param   poly_in the list of points in the polygon
 * \param   poly_out must be different from poly_in
 * \param   rotation the angle of rotation in radians
 * \param   translation the given translation
 */
template<class Point2>
static inline void rotate_translate_polygon(const std::vector<Point2> & poly_in,
                                            std::vector<Point2> & poly_out,
                                            const double & rotation,
                                            const Point2 & translation) {
  // handle the empty vector case
  if (poly_in.size() == 0) {
    poly_out.clear();
    return;
  }

  // check that both vectors are not identical
  assert(poly_in != poly_out);

  int npts = poly_in.size();
  double cos_angle = cos(rotation), sin_angle = sin(rotation);
  poly_out.resize(npts);

  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    poly_out[pt_idx].x = ROTATE_COSSIN_X(poly_in[pt_idx].x, poly_in[pt_idx].y,
                                         cos_angle, sin_angle)
                         + translation.x;
    poly_out[pt_idx].y = ROTATE_COSSIN_Y(poly_in[pt_idx].x, poly_in[pt_idx].y,
                                         cos_angle, sin_angle)
                         + translation.y;
  } // end loop pt_idx
} // end rotate_translate_polygon()

} // end namespace vision_utils

#endif // ROTATE_TRANSLATE_POLYGON_H
