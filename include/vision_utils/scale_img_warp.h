/*!
  \file        scale_img_warp.h
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

#ifndef SCALE_IMG_WARP_H
#define SCALE_IMG_WARP_H
#include <opencv2/core/core.hpp>

namespace vision_utils {

template<class _T>
void scale_img_warp(const cv::Mat_<_T> & in, cv::Mat_<_T> & out,
                    const float & xscale = 1, const int & xoffset = 0,
                    const float & yscale = 1, const int & yoffset = 0) {
  if (xscale == 0 || yscale == 0)
    return;
  cv::Mat1d M(2, 3, (float) 0); // rows, cols
  M(0, 0) = xscale;
  M(1, 1) = yscale;
  M(0, 2) = xoffset;
  M(1, 2) = yoffset;
  cv::warpAffine(in, out, M, in.size(), cv::INTER_NEAREST);
}

} // end namespace vision_utils

#endif // SCALE_IMG_WARP_H
