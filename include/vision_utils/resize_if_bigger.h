/*!
  \file        resize_if_bigger.h
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

#ifndef RESIZE_IF_BIGGER_H
#define RESIZE_IF_BIGGER_H
#include <opencv2/core/core.hpp>
#include <limits>

namespace vision_utils {

/*!
  Resize an image if it is bigger than the given limit dimensions
 \param big_img
    The image to resize
 \param small_img
 \param max_width , max_height
    The maximum dimensions for \a small_img
 \param force_resize
    If true, big_img will be always be resized into small_img
    even though it is small enough (in that case, we perform a simple deep copy).
 \return scale
    The scaling factor
    in ]0, 1] if the image was resized
    < 0 else
*/
template<class Img>
inline float resize_if_bigger(const Img & big_img,
                              Img & small_img,
                              const int & max_width ,
                              const int & max_height,
                              int interpolation = cv::INTER_NEAREST,
                              bool resize_if_smaller = false,
                              bool force_resize = false) {
  //printf("resize_if_bigger()");

  // foolproof for empty images
  if (big_img.empty() || big_img.cols <= 0 || big_img.rows <= 0) {
    printf("resize_if_bigger():empty input image!\n");
    small_img = Img();
    return 0;
  }

  // determine the scale
  bool need_resize = false;
  float scale = std::numeric_limits<float>::max();
  if (big_img.cols > max_width || resize_if_smaller) {
    need_resize = true;
    scale = 1.f * max_width / big_img.cols;
    //printf("scale:%g\n", scale);
  }
  if (big_img.rows > max_height || resize_if_smaller) {
    need_resize = true;
    scale = std::min(scale, 1.f * max_height / big_img.rows);
    //printf("scale:%g\n", scale);
  }

  // resize if needed or wanted
  if (need_resize) {
    cv::resize(big_img, small_img, cv::Size(0, 0), scale, scale, interpolation);
    return scale;
  }
  if (!need_resize && force_resize) {
    big_img.copyTo(small_img);
    return 1;
  }

  // no resize done -> return a negative value
  return -1;
} // end resize_if_bigger()

} // end namespace vision_utils

#endif // RESIZE_IF_BIGGER_H
