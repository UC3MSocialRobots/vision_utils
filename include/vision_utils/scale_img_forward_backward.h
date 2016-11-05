/*!
  \file        scale_img_forward_backward.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/5
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

#ifndef SCALE_IMG_FORWARD_BACKWARD_H
#define SCALE_IMG_FORWARD_BACKWARD_H
#include <opencv2/imgproc/imgproc.hpp>
#include <vision_utils/rectangle_intersection.h>

namespace vision_utils {

template<class _T>
void scale_img_forward_backward(const cv::Mat_<_T> & in, cv::Mat_<_T> & out,
                                const float & xscale = 1, const int & xoffset = 0,
                                const float & yscale = 1, const int & yoffset = 0,
                                bool forward = false) {
  out.create(in.size());
  out.setTo(0);
  if (xscale == 0 || yscale == 0)
    return;
  if (xscale == 1 && yscale == 1) {
    cv::Rect r1(xoffset, yoffset, in.cols, in.rows);
    cv::Rect r1_inter_out = rectangle_intersection_img(out, r1);
    cv::Rect r1_inter_in (r1_inter_out.x - xoffset, r1_inter_out.y - yoffset,
                          r1_inter_out.width, r1_inter_out.height);
    //    printf("r1:%s, r1_inter_in:%s, r1_inter_out:%s\n",
    //           print_rect(r1).c_str(),
    //           print_rect(r1_inter_in).c_str(),
    //           print_rect(r1_inter_out).c_str());
    in(r1_inter_in).copyTo(out(r1_inter_out));
    return;
  }

  int rows = in.rows, cols = in.cols;
  if (forward) {
    for (int row = 0; row < rows; ++row) {
      _T* data_in = (_T*) in.ptr(row);
      for (int col = 0; col < cols; ++col) {
        cv::Scalar color = CV_RGB(data_in[col][2], data_in[col][1], data_in[col][0]);
        cv::Point pt_out(col * xscale + xoffset, row * yscale + yoffset);
        if (pt_out.x >= 0 && pt_out.x < cols && pt_out.y >= 0 && pt_out.y < rows)
          cv::circle(out, pt_out, 2, color, -1);
      } // end loop col
    } // end loop row
  } // end if (forward)
  else { // backward
    int mincol = std::max(0,        (int) (0 * xscale + xoffset)),
        maxcol = std::min(cols - 1, (int) (cols * xscale + xoffset)),
        minrow = std::max(0,        (int) (0 * yscale + yoffset)),
        maxrow = std::min(rows - 1, (int) (rows * yscale + yoffset));
    float xscaleinv = 1.f / xscale, yscaleinv = 1.f / yscale,
        xoffsetinv = -xscaleinv * xoffset, yoffsetinv = -yscaleinv * yoffset;
    _T* data_in = (_T*) in.ptr(0);
    _T* data_out = (_T*) out.ptr(0);
    for (int row = minrow; row <= maxrow; ++row) {
      int in_row = yscaleinv * row + yoffsetinv;
      for (int col = mincol; col <= maxcol; ++col) {
        data_out[row * cols + col] = data_in[in_row * cols + (int) (xscaleinv * col + xoffsetinv)];
      } // end loop col
    } // end loop row
  } // end if (backward)
}

} // end namespace vision_utils

#endif // SCALE_IMG_FORWARD_BACKWARD_H
