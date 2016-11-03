/*!
  \file        norm.h
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

#ifndef NORM_H
#define NORM_H

namespace vision_utils {

/*!
 * @param p
 * @return the norm of p
 */
template<class Vector3>
static inline float norm(const Vector3 & p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

} // end namespace vision_utils

#endif // NORM_H
