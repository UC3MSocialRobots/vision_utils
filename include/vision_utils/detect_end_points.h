/*!
  \file        detect_end_points.h
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

#ifndef DETECT_END_POINTS_H
#define DETECT_END_POINTS_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

template<class _T>
inline bool detect_end_points(const cv::Mat_<_T> & img,
                              std::vector<cv::Point> & end_pts) {
  //printf("detect()\n");
  end_pts.clear();
  if (img.empty()) {
    printf("EndFinder: empty input image.");
    return true;
  }
  int rows = img.rows, rowsm = img.rows - 1,
      cols = img.cols, colsm = img.cols - 1;
  //  const _T *up_row = NULL,
  //      *curr_row = (rows > 0 ? img.ptr(0) : NULL),
  //      *down_row = (rows > 1 ? img.ptr(1) : NULL);
  for (int row = 0; row < rows; ++row) {
    bool up_ok = row > 0, down_ok = row < rowsm;
    const _T *up_row = (up_ok ? img.ptr(row-1) : NULL),
        *curr_row = img.ptr(row),
        *down_row = (down_ok ? img.ptr(row+1) : NULL);
    for (int col = 0; col < cols; ++col) {
      //printf("img(%i, %i) = %i\n", col, row, (int) curr_row[col]);
      if (!curr_row[col])
        continue;
      bool left_ok = col > 0, right_ok = col < colsm;
      // count nb of neighbours
      int neigh_nb = 0;
      if (left_ok && curr_row[col-1] && (++neigh_nb) >= 2)
        continue;
      if (right_ok && curr_row[col+1] && (++neigh_nb) >= 2)
        continue;
      if (up_ok && up_row[col] && (++neigh_nb) >= 2)
        continue;
      if (down_ok && down_row[col] && (++neigh_nb) >= 2)
        continue;
      // C8
      if (left_ok && up_ok && up_row[col-1] && (++neigh_nb) >= 2)
        continue;
      if (right_ok && up_ok && up_row[col+1] && (++neigh_nb) >= 2)
        continue;
      if (left_ok && down_ok && down_row[col-1] && (++neigh_nb) >= 2)
        continue;
      if (right_ok && down_ok && down_row[col+1] && (++neigh_nb) >= 2)
        continue;
      // found an end!
      //printf("adding (%i, %i)\n", col, row);
      end_pts.push_back(cv::Point(col, row));
    } // end loop col
    // shifting data pointers
    //      printf("Shifting\n");
    //      up_row = curr_row;
    //      curr_row = down_row;
    //      if (down_ok)
    //        down_row = img.ptr(row+1);
  } // end loop row
  return true;
} // end detect_end_points()

} // end namespace vision_utils

#endif // DETECT_END_POINTS_H
