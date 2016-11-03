/*!
  \file        rectangle_intersection.h
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

#ifndef RECTANGLE_INTERSECTION_H
#define RECTANGLE_INTERSECTION_H

#include "vision_utils/img_bbox.h"

namespace vision_utils {

/*! \return the rectangle intersection between r1 and r2
 *  \see http://docs.opencv.org/modules/core/doc/basic_structures.html#Rect_
 */
template<class Rect>
inline Rect rectangle_intersection(const Rect & r1,
                                   const Rect & r2) {
  return (r1 & r2);
} // end biggest_rect()

////////////////////////////////////////////////////////////////////////////////

template<class Img, class Rect>
inline Rect rectangle_intersection_img(const Img & i,
                                       const Rect & r) {
  return rectangle_intersection(img_bbox<Img, Rect>(i), r);
} // end biggest_rect()

} // end namespace vision_utils

#endif // RECTANGLE_INTERSECTION_H
