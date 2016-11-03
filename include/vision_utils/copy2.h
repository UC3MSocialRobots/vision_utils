/*!
  \file        copy2.h
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

#ifndef COPY2_H
#define COPY2_H

namespace vision_utils {

//! copy the 2 fields of a (x, y) structure to another one
template<class _Tsrc, class _Tdst>
inline void copy2(const _Tsrc & src, _Tdst & dst) {
  dst.x = src.x; dst.y = src.y;
}

} // end namespace vision_utils

#endif // COPY2_H
