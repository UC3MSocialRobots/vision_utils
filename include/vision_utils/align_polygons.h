/*!
  \file        align_polygons.h
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

#ifndef ALIGN_POLYGONS_H
#define ALIGN_POLYGONS_H
// std includes
#include <vector>
#include "vision_utils/rotate_translate_polygon.h"

namespace vision_utils {

/*!
 Aligns two polygons(rotation and translation, but no scaling).
 \param A
 \param B
 \param B_out
    will contain rotated and translated points of B,
    such as B_out[0] = A[0],
    and the orientation of B_out is the same as A
*/
template<class Point2>
static inline void align_polygons(const std::vector<Point2> & A,
                                  const std::vector<Point2> & B,
                                  std::vector<Point2> & B_out) {
  // find angles in A and B
  double angle_A = oriented_angle_of_a_vector
                   (A.back() - A.front());
  double angle_B = oriented_angle_of_a_vector
                   (B.back() - B.front());
  //printf("angle_A:%g, angle_B:%g\n", angle_A, angle_B);

  B_out.clear();
  B_out.reserve(B.size());
  double cos_angle = cos(angle_A - angle_B), sin_angle = sin(angle_A - angle_B);

  for (unsigned int pt_idx = 0; pt_idx < B.size(); ++pt_idx) {
    B_out.push_back
        (Point2(ROTATE_COSSIN_X(B[pt_idx].x - B.front().x, B[pt_idx].y - B.front().y,
                                cos_angle, sin_angle)
                + A.front().x,
                ROTATE_COSSIN_Y(B[pt_idx].x - B.front().x, B[pt_idx].y - B.front().y,
                                cos_angle, sin_angle)
                + A.front().y
                ));
  }
} // end align_polygons()

} // end namespace vision_utils

#endif // ALIGN_POLYGONS_H
