/*!
  \file        hue_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/10

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

Some utils to play with hue

 */

#ifndef HUE_UTILS_H
#define HUE_UTILS_H

#include <math.h>
#include <vector>
#include <string>
#include <vision_utils/HSVtoRGB.h>

namespace vision_utils {
//cut:hue2rgb
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

////////////////////////////////////////////////////////////////////////////////
//cut:hue2rgb_make_lookup_table
template<class Color3_255>
inline void hue2rgb_make_lookup_table(std::vector<Color3_255> & lut,
                                      const unsigned int lut_size = 255) {
  lut.reserve(lut_size);
  lut.clear();
  for (unsigned int hue_idx = 0; hue_idx < lut_size; ++hue_idx) {
    unsigned char hue =  hue_idx * 180.f / lut_size;
    lut.push_back(hue2rgb<Color3_255>(hue));
  }
} // end hue2rgb_make_lookup_table()

////////////////////////////////////////////////////////////////////////////////
//cut:hue_to_string
/*!
 * Converts a hue value into a string representation
 * \param hue
 *  in [0..180] (the way it comes with OpenCV hue conversion)
 *  traditionally, hue is in [0..360], so please divide it by 2
 * \example "red", "blue", etc.
 */
inline std::string hue_to_string(const unsigned char hue) {
  if (hue <  10) // 20 degrees
    return "red";
  else if (hue <  25) // 50 degrees
    return "orange";
  else if (hue <  35) // 70 degrees
    return "yellow";
  else if (hue <  45) // 90 degrees
    return "lemon";
  else if (hue <  75) // 150 degrees
    return "green";
  else if (hue <  95) // 190 degrees
    return "electric blue";
  else if (hue <  135) // 270 degrees
    return "blue";
  else if (hue <  165) // 330 degrees
    return "purple";
  else
    return "red";
}

} // end namespace vision_utils

#endif // HUE_UTILS_H
