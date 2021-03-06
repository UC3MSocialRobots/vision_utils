/*!
  \\file        is_nan_depth.h
  \\author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \\date        2016/10/29

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

#ifndef IS_NAN_DEPTH_H
#define IS_NAN_DEPTH_H

namespace vision_utils {

//! \return \true if distance is a NAN (or a simulated one)
inline bool is_nan_depth(const float & distance) {
  return (distance == 0 || isnan(distance));
}

} // end namespace vision_utils

#endif // IS_NAN_DEPTH_H
