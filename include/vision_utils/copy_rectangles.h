/*!
  \file        copy_rectangles.h
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

#ifndef COPY_RECTANGLES_H
#define COPY_RECTANGLES_H

namespace vision_utils {

//! copy a Rectangle like class into another
template<class RectA, class RectB>
inline void copy_rectangles(const RectA & A, RectB & B) {
  B.x = A.x;
  B.y = A.y;
  B.width = A.width;
  B.height = A.height;
} // end biggest_rect()

} // end namespace vision_utils

#endif // COPY_RECTANGLES_H
