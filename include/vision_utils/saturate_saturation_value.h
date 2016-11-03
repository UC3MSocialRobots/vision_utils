/*!
  \file        saturate_saturation_value.h
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

#ifndef SATURATE_SATURATION_VALUE_H
#define SATURATE_SATURATION_VALUE_H
// std includes
#include <opencv2/core/core.hpp>
#include <vector>

namespace vision_utils {

void saturate_saturation_value(const cv::Mat3b & src_bgr,
                               std::vector<cv::Mat> & layers,
                               cv::Mat3b & out_bgr) {
  cv::cvtColor(src_bgr, out_bgr, CV_BGR2HSV); // convert BGR -> HSV
  cv::split(out_bgr, layers);
  layers[1].setTo(255); // set saturation to 255
  layers[2].setTo(255); // set value to 255
  cv::merge(layers, out_bgr); // recombine HSV
  cv::cvtColor(out_bgr, out_bgr, CV_HSV2BGR); // now convert HSV -> BGR
}

} // end namespace vision_utils

#endif // SATURATE_SATURATION_VALUE_H
