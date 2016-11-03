/*!
  \file        rgb_saturate_saturation_value.h
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

#ifndef RGB_SATURATE_SATURATION_VALUE_H
#define RGB_SATURATE_SATURATION_VALUE_H
// std includes
#include <opencv2/core/core.hpp>
#include <vector>
#include "vision_utils/hue_to_rgb_mat.h"
#include "vision_utils/rgb2hue.h"
#include "vision_utils/saturate_saturation_value.h"

namespace vision_utils {

/*!
 *\brief   create the rgb image which illustrate the hue
 *\param   src a bgr image
 *\param   dest a bgr image
 */
inline void rgb_saturate_saturation_value_slow(const cv::Mat3b & src_bgr, cv::Mat3b & dest_bgr)  {
  std::vector<cv::Mat> layers;
  saturate_saturation_value(src_bgr, layers, dest_bgr);
}

inline void rgb_saturate_saturation_value(const cv::Mat3b & src_bgr, cv::Mat3b & dest_bgr)  {
  cv::Mat1b hue;
  rgb2hue(src_bgr, dest_bgr, hue);
  hue2rgb(hue, dest_bgr);
}

} // end namespace vision_utils

#endif // RGB_SATURATE_SATURATION_VALUE_H
