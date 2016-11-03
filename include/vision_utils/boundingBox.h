/*!
  \file        boundingBox.h
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

#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc

namespace vision_utils {

/*!
 *\brief   get the bounding box of the non null points of an image
 *\param   img a monochrome image
 *\return the bounding box of the non null points,
 *        cv::Rect(-1, -1, -1, -1) if the image is empty
 */
template<class _T>
static inline cv::Rect boundingBox(const cv::Mat_<_T> & img) {
  if (img.empty())
    return cv::Rect(-1, -1, -1, -1);
  assert(img.isContinuous());
  int xMin = 0, yMin = 0, xMax = 0, yMax = 0;
  bool was_init = false;
  const _T* img_it = (_T*) img.ptr(0);
  int nrows = img.rows, ncols = img.cols;
  for (int y = 0; y < nrows; ++y) {
    for (int x = 0; x < ncols; ++x) {
      if (*img_it++) {
        if (!was_init) {
          xMin = xMax = x;
          yMin = yMax = y;
          was_init = true;
          continue;
        }
        if (x < xMin)
          xMin = x;
        else if (x > xMax)
          xMax = x;

        if (y < yMin)
          yMin = y;
        else if (y > yMax)
          yMax = y;
      }
    } // end loop x
  } // end loop y

  if (!was_init) { // no white point found
    printf("boundingBox: no non null point found!\n");
    return cv::Rect(-1, -1, -1, -1);
  }
  // from http://docs.opencv.org/java/org/opencv/core/Rect.html
  // OpenCV typically assumes that the top and left boundary of the rectangle
  // are inclusive, while the right and bottom boundaries are not.
  // For example, the method Rect_.contains returns true if
  // x <= pt.x < x+width,   y <= pt.y < y+height
  return cv::Rect(xMin, yMin, 1 + xMax - xMin,  1 + yMax - yMin);
}

} // end namespace vision_utils

#endif // BOUNDINGBOX_H
