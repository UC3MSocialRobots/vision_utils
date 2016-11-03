/*!
  \file        img_bbox.h
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

#ifndef IMG_BBOX_H
#define IMG_BBOX_H

namespace vision_utils {

template<class Img, class Rect>
inline Rect img_bbox(const Img & i) {
  return Rect(0, 0, i.cols, i.rows);
} // end biggest_rect()

} // end namespace vision_utils

#endif // IMG_BBOX_H
