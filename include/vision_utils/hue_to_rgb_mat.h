/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/11/3
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

  odo Description of the file
 */

#ifndef HUE_TO_RGB_MAT_H
#define HUE_TO_RGB_MAT_H
#include "vision_utils/hue2rgb_make_lookup_table.h"

namespace vision_utils {

inline void hue2rgb(const cv::Mat1b & hue, cv::Mat3b & rgb) {
  // make lookup table - hue goes in 0..180
  static std::vector<cv::Vec3b> hue_lut;
  hue2rgb_make_lookup_table(hue_lut, 180);
  // use it
  rgb.create(hue.size());
  for (int row = 0; row < hue.rows; ++row) {
    // get the address of row
    const uchar* hue_data = hue.ptr<uchar>(row);
    cv::Vec3b* rgb_data = rgb.ptr<cv::Vec3b>(row);
    for (int col = 0; col < hue.cols; ++col) {
      rgb_data[col] = hue_lut[ hue_data[col] ];
    } // end loop col
  } // end loop row
}

////////////////////////////////////////////////////////////////////////////

inline cv::Mat3b hue2rgb(const cv::Mat1b & hue) {
  cv::Mat3b rgb;
  hue2rgb(hue, rgb);
  return rgb;
}



} // end namespace vision_utils

#endif // HUE_TO_RGB_MAT_H

