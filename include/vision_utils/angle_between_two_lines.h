/*!
  \file        angle_between_two_lines.h
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

#ifndef ANGLE_BETWEEN_TWO_LINES_H
#define ANGLE_BETWEEN_TWO_LINES_H

namespace vision_utils {

/*!
 * \brief   returns the angle formed at the intersection between the two lines
     *
 * \param   l1 equation of the first line
 * \param   l2 equation of the second line
 * \return  the angle between 0 and PI
 */
template<class Point3>
static inline double angle_between_two_lines(const Point3 & l1,
                                             const Point3 & l2) {
  FooPoint2d vdir_l1 (-l1.y, l1.x);
  FooPoint2d vdir_l2 (-l2.y, l2.x);
  return absolute_angle_between_two_vectors(vdir_l1, vdir_l2);
}

} // end namespace vision_utils

#endif // ANGLE_BETWEEN_TWO_LINES_H
