/*!
  \file        minnonzero_from_botton_row.h
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

#ifndef MINNONZERO_FROM_BOTTON_ROW_H
#define MINNONZERO_FROM_BOTTON_ROW_H
// std includes
#include <opencv2/core/core.hpp>

namespace vision_utils {

/*!
 * Find the min non zero value in an image, starting at the bottom row
 * and going upwards.
 * \param img
 *    the image
 * \param minnonzero
 *    the minimum non null value found on the lowest non null row
 * \param minnonzero_pos
 *    the exact position where \a minnonzero was found
 * \return
 *    true if there was a non null value found,
 *    false if img == 0
 */
template<class _T>
inline bool minnonzero_from_botton_row(const cv::Mat_<_T> & img,
                                       _T & minnonzero,
                                       cv::Point & minnonzero_pos) {
  minnonzero = std::numeric_limits<_T>::max();
  // start at one-before-last row
  unsigned int row = img.rows - 2, ncols = img.cols;
  bool was_found = false;
  // const _T* data = img.data;
  while (row > 0) {
    const _T* data = (const _T*) (img.ptr(row));
    for (unsigned int col = 0; col < ncols; ++col) {
      if (data[col]) {
        if (minnonzero > data[col]) {
          minnonzero = data[col];
          minnonzero_pos.x = col;
          minnonzero_pos.y = row;
        }
        was_found = true;
      } // end if (data[col])
    } // end loop col
    if (was_found) // stop if found something
      break;
    --row; // otherwise go to previous row
  } // end
  return (was_found);
}

} // end namespace vision_utils

#endif // MINNONZERO_FROM_BOTTON_ROW_H
