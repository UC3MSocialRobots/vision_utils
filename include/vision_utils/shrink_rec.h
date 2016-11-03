/*!
  \file        shrink_rec.h
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

#ifndef SHRINK_REC_H
#define SHRINK_REC_H

namespace vision_utils {

/*! shrink a rectangle by a given ratio, keeping the same center
  * \arg ratio
      for instance, 0.5 will divide the sides of the rect by 2.
      0.9 will shrink just a bit the size of the rectangle.
  */
template<class Rect>
inline Rect shrink_rec(const Rect & in,
                       const double & ratio_width,
                       const double & ratio_height) {
  Rect ans;
  ans.x = in.x + in.width * (1. - ratio_width) / 2;
  ans.y = in.y + in.height * (1. - ratio_height) / 2;
  ans.width = in.width * ratio_width;
  ans.height = in.height * ratio_height;
  return ans;
}

template<class Rect>
inline Rect shrink_rec(const Rect & in, const double & ratio) {
  return shrink_rec<Rect>(in, ratio, ratio);
}

} // end namespace vision_utils

#endif // SHRINK_REC_H
