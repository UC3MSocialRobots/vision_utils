/*!
  \file        rgb2hsv_layer.h
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

#ifndef RGB2HSV_LAYER_H
#define RGB2HSV_LAYER_H
// std includes
#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/extractLayer.h"

namespace vision_utils {

/*!
 *\brief   save in "dest" one HSV component of "src"
 */
inline void rgb2hsv_layer(const cv::Mat3b & src_bgr, cv::Mat3b & temp_hsv,
                          cv::Mat1b & dest_layer, const int layer_idx) {
  cv::cvtColor(src_bgr, temp_hsv, CV_BGR2HSV); // conversion in HSV
  extractLayer(temp_hsv, dest_layer, layer_idx);
}

} // end namespace vision_utils

#endif // RGB2HSV_LAYER_H
