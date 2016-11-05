/*!
  \file        resize_constrain_proportions.h
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

#ifndef RESIZE_CONSTRAIN_PROPORTIONS_H
#define RESIZE_CONSTRAIN_PROPORTIONS_H
#include <opencv2/core/core.hpp>

namespace vision_utils {

/*!
 Just a ROI of the image will be kept,
 such as it fits in (dst_width, dst_height)
 with the same aspect.
 \param big_img
 \param small_img
 \param width
 \param height
*/
template<class Img>
inline void resize_constrain_proportions(const Img & src,
                                         Img & dst,
                                         const int & dst_width ,
                                         const int & dst_height,
                                         int interpolation = cv::INTER_NEAREST) {
  float ratio_src = 1.f * src.cols / src.rows;
  float ratio_dst = 1.f * dst_width / dst_height;
  /*        width    case 1 +-----+   case2 +-----+-----+-----+
     ratio= ------          | src |         | src | dst | src |
            height          +-----+         +-----+-----+-----+
                            | dst |
                            +-----+
                            | src |
                            +-----+      */
  cv::Rect ROI;
  if (ratio_src < ratio_dst) { // case 1
    //printf("case 1\n");
    ROI.width = src.cols;
    ROI.height = ROI.width / ratio_dst;
    ROI.x = 0;
    ROI.y = (src.rows - ROI.height) / 2;
  }
  else {
    //printf("case 2\n");
    ROI.height = src.rows;
    ROI.width = ROI.height * ratio_dst;
    ROI.y = 0;
    ROI.x = (src.cols - ROI.width) / 2;
  }
  //  printf("src:%i x %i, dst:%i x %i, dst_width:%i, dst_height:%i\n",
  //         src.cols, src.rows, dst.cols, dst.rows, dst_width, dst_height);
  cv::resize(src(ROI), dst, cv::Size(dst_width, dst_height), 0, 0, interpolation);
} // end resize_if_bigger()

} // end namespace vision_utils

#endif // RESIZE_CONSTRAIN_PROPORTIONS_H
