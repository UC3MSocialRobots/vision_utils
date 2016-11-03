/*!
  \file        hue_to_string.h
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

#ifndef HUE_TO_STRING_H
#define HUE_TO_STRING_H
// std includes
#include <string>

namespace vision_utils {

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

#endif // HUE_TO_STRING_H
