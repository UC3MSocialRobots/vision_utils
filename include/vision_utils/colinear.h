/*!
  \file        colinear.h
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

#ifndef COLINEAR_H
#define COLINEAR_H

namespace vision_utils {

template<class Point2>
static inline bool colinear(const Point2 & A,
                            const Point2 & B,
                            const Point2 & C) {
  return ((C.y-A.y)*(B.x-A.x) == (B.y-A.y)*(C.x-A.x));
}

} // end namespace vision_utils

#endif // COLINEAR_H
