/*!
  \file        centroid.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/1/4

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

Find the centroid of a monochrome image.
 */
#ifndef CENTROID_H
#define CENTROID_H
#include <opencv2/core/core.hpp>
namespace vision_utils {
inline cv::Point centroidOfMonochromeImage(const cv::Mat1b & img) {
  assert(img.isContinuous());
  unsigned int ncols = img.cols, nrows = img.rows;
  const unsigned char* img_ptr = img.ptr();
  double colsum = 0, rowsum = 0, npts = 0;
  for(unsigned int row = 0 ; row < nrows ; ++row) {
    for(unsigned int col = 0 ; col < ncols ; ++col) {
      if (*img_ptr) {
        colsum += col;
        rowsum += row;
        ++npts;
      }
      ++img_ptr;
    } // end loop row
  } // end loop col
  if (npts == 0)
    return cv::Point();
  return cv::Point(colsum / npts, rowsum / npts);
} // end centroidOfMonochromeImage()
} // end namespace vision_utils

#endif // CENTROID_H
