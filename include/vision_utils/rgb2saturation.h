/*!
  \file        rgb2saturation.h
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

#ifndef RGB2SATURATION_H
#define RGB2SATURATION_H
// std includes
#include <vision_utils/rgb2hsv_layer.h>

namespace vision_utils {

/*!
 *\brief   save in "dest" the saturation component of "src"
 */
inline void rgb2saturation(const cv::Mat3b & src_bgr, cv::Mat3b & temp_hsv,
                           cv::Mat1b & dest_saturation) {
  rgb2hsv_layer(src_bgr, temp_hsv, dest_saturation, 1);
}

} // end namespace vision_utils

#endif // RGB2SATURATION_H
