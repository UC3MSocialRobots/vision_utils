/*!
  \file        print_random_pts_int.h
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

#ifndef PRINT_RANDOM_PTS_INT_H
#define PRINT_RANDOM_PTS_INT_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc

namespace vision_utils {

template<class _T>
static inline void print_random_pts_int(const cv::Mat & img, const int & nb_pts) {
  //printf("print_random_pts(nb_pts:%i)", nb_pts);
  for (int pt_idx = 0; pt_idx < nb_pts; ++pt_idx) {
    int col = rand() % img.cols;
    int row = rand() % img.rows;
    printf("row:%i, col:%i, val:%i\n", row, col, (int) img.at<_T>(col, row));
  } // end loop pt_idx
} // end print_random_pts

} // end namespace vision_utils

#endif // PRINT_RANDOM_PTS_INT_H
