/*!
  \file        hue2rgb.h
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

#ifndef HUE2RGB_H
#define HUE2RGB_H
// std includes
#include <opencv2/core/core.hpp>
#include <vision_utils/HSVtoRGB.h>

namespace vision_utils {

/*!
 * \brief   color conversion function
 * \param hue
 *  in [0..180] (the way it comes with OpenCV hue conversion)
 *  traditionally, hue is in [0..360], so please divide it by 2
   */
inline cv::Vec3b hue2rgb(const unsigned char hue) {
  float r, g, b, h_filter = (hue >= 179 ? 179 : hue);
  HSVtoRGB(h_filter * 2.f, 1.f, 1.f, r, g, b);
  return cv::Vec3b(b * 255, g * 255, r * 255);
}
/*!
 * \brief   color conversion function
 * \param hue
 *  in [0..180] (the way it comes with OpenCV hue conversion)
 *  traditionally, hue is in [0..360], so please divide it by 2
   */
template<class Color3_255>
inline Color3_255 hue2rgb(const unsigned char hue) {
  float r, g, b, h_filter = (hue >= 179 ? 179 : hue);
  HSVtoRGB(h_filter * 2.f, 1.f, 1.f, r, g, b);
  return Color3_255(b * 255, g * 255, r * 255);
}

} // end namespace vision_utils

#endif // HUE2RGB_H
