/*!
  \file        rgb2hue_and_showRGB.h
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

#ifndef RGB2HUE_AND_SHOWRGB_H
#define RGB2HUE_AND_SHOWRGB_H
// std includes
#include <opencv2/core/core.hpp>
#include <string>

namespace vision_utils {

/*!
 *\brief   create the rgb image which illustrate the hue component
 *of the image "filename"
 */
inline void rgb2hue_and_showRGB(const std::string & filename) {
  cv::Mat3b original_h = cv::imread(filename, CV_LOAD_IMAGE_COLOR), res;
  rgb_saturate_saturation_value(original_h, res);
  const char* window_hueImage = "Hue Previewer";
  cv::namedWindow(window_hueImage, CV_WINDOW_AUTOSIZE);
  cv::imshow(window_hueImage, res);
  cv::waitKey(0);
} // end rgb2hue_and_showRGB();

} // end namespace vision_utils

#endif // RGB2HUE_AND_SHOWRGB_H
