/*!
  \file        point_vec_to_string.h
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

#ifndef POINT_VEC_TO_STRING_H
#define POINT_VEC_TO_STRING_H
// std includes
#include <opencv2/core/core.hpp>
#include <string>

namespace vision_utils {

template<class Pt2Iterable>
static std::string point_vec_to_string(const Pt2Iterable & in, const cv::Size size) {
  //cv::Rect r = boundingBox_vec<cv::Rect, cv::Point>(in);
  //cv::Mat1b out(r.width, r.height);
  cv::Mat1b out(size, (uchar) 0);
  int cols = size.width;
  uchar* outdata = out.ptr(0);
  for (unsigned int pt_idx = 0; pt_idx < in.size(); ++pt_idx) {
    if (in[pt_idx].x >= 0 && in[pt_idx].x < size.width &&
        in[pt_idx].y >= 0 && in[pt_idx].y < size.height)
      outdata[in[pt_idx].y * cols + in[pt_idx].x] = 255;
  } // end loop pt_idx
  return img2string(out);
}

} // end namespace vision_utils

#endif // POINT_VEC_TO_STRING_H
