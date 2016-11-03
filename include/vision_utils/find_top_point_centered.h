/*!
  \file        find_top_point_centered.h
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

#ifndef FIND_TOP_POINT_CENTERED_H
#define FIND_TOP_POINT_CENTERED_H
// std includes
#include <algorithm> // for std::min(), std::max()...
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include "vision_utils/boundingBox.h"

namespace vision_utils {

/*! Find the non null point with the lowest row index
 * in a mask, among non zero pixels.
 * We only search in columns with an index close to the vertical center
 * of the bounding box of the content.
 * \arg width_ratio_to_bbox in [0, 1]
 *   around 0, very narrow search window
 *   around 1, very wide search window (almost equal to image bbox)
 * \return (-1, -1) if not found
 */
template<class _T>
cv::Point find_top_point_centered(const cv::Mat_<_T> & image,
                                  bool compute_bbox,
                                  const double width_ratio_to_bbox = .3,
                                  const _T zero_value = _T(0),
                                  cv::Rect* search_window_out = NULL) {
  assert(width_ratio_to_bbox > 0 && width_ratio_to_bbox <= 1);
  // find bounding box and allowed columns
  cv::Rect bbox = (compute_bbox ? boundingBox(image) : bbox_full(image));
  int
      min_col = bbox.x + (1.f - width_ratio_to_bbox) * bbox.width / 2,
      max_col = bbox.x + (1.f + width_ratio_to_bbox) * bbox.width / 2,
      min_row = bbox.y,
      max_row = bbox.y + bbox.height;
  //min_col = std::max(min_col, 0);
  //max_col = std::min(max_col, image.cols - 1);
  // return search window if user curious about it
  if (search_window_out != NULL)
    *search_window_out = cv::Rect(min_col, min_row, max_col - min_col + 1, max_row - min_row + 1);

  // run among the allowed window
  for (int row = min_row; row < max_row; ++row) {
    // get the address of row
    const _T* data = image.ptr(row);
    for (int col_begin = min_col; col_begin <= max_col; ++col_begin) {
      if (data[col_begin] != zero_value) {
        // now see where the non zeros stop
        int col_end = col_begin;
        while (col_end < image.cols && data[col_end] != zero_value)
          ++col_end;
        // return the middle of these non zero values
        return cv::Point((col_begin + col_end) / 2, row);
      }
    } // end loop col_begin
  } // end loop row
  printf("find_top_point_centered(img:%ix%i, search_window:(%ix%i)->(%ix%i)): "
         "could not find a non zero point!\n",
         image.cols, image.rows, min_col, min_row, max_col, max_row);
  return cv::Point(-1, -1);
}

} // end namespace vision_utils

#endif // FIND_TOP_POINT_CENTERED_H
