/*!
  \file        rgb_file2hue.h
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

#ifndef RGB_FILE2HUE_H
#define RGB_FILE2HUE_H
// std includes
#include <opencv2/core/core.hpp>
#include <string>

namespace vision_utils {

//! read an image from file and return its
inline cv::Mat1b rgb_file2hue(const std::string & filename) {
  cv::Mat3b img_rgb = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  return rgb2hue(img_rgb);
}

} // end namespace vision_utils

#endif // RGB_FILE2HUE_H
