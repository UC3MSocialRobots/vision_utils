/*!
  \file        hue2rgb_make_lookup_table.h
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

#ifndef HUE2RGB_MAKE_LOOKUP_TABLE_H
#define HUE2RGB_MAKE_LOOKUP_TABLE_H
// std includes
#include <vector>
#include "vision_utils/hue2rgb.h"

namespace vision_utils {

inline void hue2rgb_make_lookup_table(std::vector<cv::Vec3b> & lut,
                                      const unsigned int lut_size = 255) {
  lut.reserve(lut_size);
  lut.clear();
  for (unsigned int hue_idx = 0; hue_idx < lut_size; ++hue_idx) {
    unsigned char hue =  hue_idx * 180.f / lut_size;
    lut.push_back(hue2rgb(hue));
  }
} // end hue2rgb_make_lookup_table()

} // end namespace vision_utils

#endif // HUE2RGB_MAKE_LOOKUP_TABLE_H
