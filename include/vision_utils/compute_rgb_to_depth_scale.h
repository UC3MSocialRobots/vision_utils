/*!
  \file        compute_rgb_to_depth_scale.h
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

#ifndef COMPUTE_RGB_TO_DEPTH_SCALE_H
#define COMPUTE_RGB_TO_DEPTH_SCALE_H
// std includes
#include <opencv2/core/core.hpp>

namespace vision_utils {

inline void compute_rgb_to_depth_scale(const cv::Size & rgb_size,
                                       const cv::Size & depth_size,
                                       double & rgb_to_depth_scale_x,
                                       double & rgb_to_depth_scale_y) {
  rgb_to_depth_scale_x = 1.f * depth_size.width / rgb_size.width;
  // 1.f * bridge_depth_img_ptr->image.cols / bridge_rgb_img_ptr->image.cols;
  rgb_to_depth_scale_y = 1.f * depth_size.height / rgb_size.height;
  // 1.f * bridge_depth_img_ptr->image.rows / bridge_rgb_img_ptr->image.rows;
}

} // end namespace vision_utils

#endif // COMPUTE_RGB_TO_DEPTH_SCALE_H
