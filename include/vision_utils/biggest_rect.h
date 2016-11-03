/*!
  \file        biggest_rect.h
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

#ifndef BIGGEST_RECT_H
#define BIGGEST_RECT_H
// std includes
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

//! \return the index of the biggest rectangle in a set
template<class Rect>
inline int biggest_rect_index(const std::vector<Rect> & rects) {
  if (rects.size() < 1) {
    printf("Cannot call biggest_rect() on an empty vector!");
    return -1;
  }
  int best_rec_idx = 0;
  double best_rec_area = rects[0].width * rects[0].height;
  for (unsigned int curr_rec_idx = 1; curr_rec_idx < rects.size(); ++curr_rec_idx) {
    double curr_rec_area = rects[curr_rec_idx].width * rects[curr_rec_idx].width;
    if (curr_rec_area > best_rec_area) {
      best_rec_area = curr_rec_area;
      best_rec_idx = curr_rec_idx;
    }
  } // end loop rec_idx
  return best_rec_idx;
} // end biggest_rect_index()

////////////////////////////////////////////////////////////////////////////////

//! \return the biggest rectangle of a set
template<class Rect>
inline Rect biggest_rect(const std::vector<Rect> & rects) {
  if (rects.size() < 1) {
    printf("Cannot call biggest_rect() on an empty vector!");
    return Rect();
  }
  return rects[biggest_rect_index(rects)];
} // end biggest_rect()

} // end namespace vision_utils

#endif // BIGGEST_RECT_H
