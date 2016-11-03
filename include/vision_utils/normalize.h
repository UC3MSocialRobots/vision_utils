/*!
  \file        normalize.h
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

#ifndef NORMALIZE_H
#define NORMALIZE_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 * normalize a std::vector
 * @param p the std::vector to normalize
 */
template<class Vector3>
static inline void normalize(Vector3 & p) {
  float normP = norm(p);
  p.x /= normP;
  p.y /= normP;
  p.z /= normP;
}

} // end namespace vision_utils

#endif // NORMALIZE_H
