/*!
  \file        absolute_angle_between_two_vectors.h
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

#ifndef ABSOLUTE_ANGLE_BETWEEN_TWO_VECTORS_H
#define ABSOLUTE_ANGLE_BETWEEN_TWO_VECTORS_H

namespace vision_utils {

/*!
 * \brief   returns the angle formed between two vectors
 * \param   v1 the first vector
 * \param   v2 the second vector
 * \return  the angle between 0 and PI
 */
template<class Vec2>
static inline double absolute_angle_between_two_vectors(const Vec2 & v1,
                                                        const Vec2 & v2) {
  //double cos_angle =
  //printf("cos_angle:%f", cos_angle);
  return acos( fmin(1, fmax(-1,
                            // cos angle is :
                            (v1.x * v2.x + v1.y * v2.y) // scalar product
                            /
                            (norm2(v1) * norm2(v2)) // product of norms
                            )));
}

} // end namespace vision_utils

#endif // ABSOLUTE_ANGLE_BETWEEN_TWO_VECTORS_H
