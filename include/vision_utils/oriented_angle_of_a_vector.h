/*!
  \file        oriented_angle_of_a_vector.h
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

#ifndef ORIENTED_ANGLE_OF_A_VECTOR_H
#define ORIENTED_ANGLE_OF_A_VECTOR_H

namespace vision_utils {

/*!
 * \brief   returns the angle formed between a vector and the horizontal line
 * \param   v the vector
 * \return  the principal angle of v, in the interval [-pi,+pi] radians.
 */
template<class Vec2>
static inline double oriented_angle_of_a_vector(const Vec2 & v) {
  return atan2(v.y, v.x);
}

} // end namespace vision_utils

#endif // ORIENTED_ANGLE_OF_A_VECTOR_H
