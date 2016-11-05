/*!
  \file        distance_points3_squared.h
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

#ifndef DISTANCE_POINTS3_SQUARED_H
#define DISTANCE_POINTS3_SQUARED_H

namespace vision_utils {

/*!
 \param A  a 3D pt (x, y, z)
 \param B  a 3D pt (x, y, z) - it can be of another type than A
 \return the quantity AB * AB. It is faster to compute than their actual distance
 and enough for, for instance, comparing two distances
*/
template<class Point3_A, class Point3_B>
static inline double
distance_points3_squared(const Point3_A & A, const Point3_B & B) {
  return (A.x - B.x) * (A.x - B.x)
      +  (A.y - B.y) * (A.y - B.y)
      +  (A.z - B.z) * (A.z - B.z);
}

} // end namespace vision_utils

#endif // DISTANCE_POINTS3_SQUARED_H
