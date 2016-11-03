/*!
  \file        nonNulPoints.h
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

#ifndef NONNULPOINTS_H
#define NONNULPOINTS_H
// std includes
#include <opencv2/core/core.hpp>

namespace vision_utils {

/*!
 *\brief   get a vector from an image with all the non nul points
 */
template<class Pt2Iterable>
inline void nonNulPoints(const cv::Mat1b & img, Pt2Iterable & rep) {
  //printf("nonNulPoints()");
#if 0
  rep.clear();
  // rep.reserve( cv::countNonZero(img) );
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const uchar* data = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; ++col) {
      if (*data++)
        rep.push_back(cv::Point(col, row));
    } // end loop col
  } // end loop row
#else
  rep.resize( cv::countNonZero(img) );
  int rep_idx = 0;
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const uchar* data = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; ++col) {
      if (*data++) {
        rep[rep_idx].x = col;
        rep[rep_idx].y = row;
        ++rep_idx;
      }
    } // end loop col
  } // end loop row
#endif
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   get a vector from an image with all the non nul points
 */
template<class CoordIterable>
inline void nonNulPoints(const cv::Mat1b & img, CoordIterable & x, CoordIterable & y) {
  //printf("nonNulPoints()");
  x.resize( cv::countNonZero(img) );
  y.resize(x.size());
  int rep_idx = 0;
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const uchar* data = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; ++col) {
      if (*data++) {
        x[rep_idx] = col;
        y[rep_idx] = row;
        ++rep_idx;
      }
    } // end loop col
  } // end loop row
}

} // end namespace vision_utils

#endif // NONNULPOINTS_H
