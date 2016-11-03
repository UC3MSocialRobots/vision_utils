/*!
  \file        uchar_bgr_color_to_ros_rgba_color.h
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

#ifndef UCHAR_BGR_COLOR_TO_ROS_RGBA_COLOR_H
#define UCHAR_BGR_COLOR_TO_ROS_RGBA_COLOR_H

#include <std_msgs/ColorRGBA.h>

namespace vision_utils {

/*!
 * Converts a 3 fields structure into a ROS std_msgs::ColorRGBA color.
 * \param b, g, r
 *   The blue, green, red fields, between 0 and 255
 * \return
 *   The corresponding color, with fields in 0..1.
 */
template<class Value255>
std_msgs::ColorRGBA uchar_bgr_color_to_ros_rgba_color
(const Value255& b, const Value255& g, const Value255& r) {
  std_msgs::ColorRGBA color;
  color.r = r / 255.f;
  color.g = g / 255.f;
  color.b = b / 255.f;
  color.a = 1;
  return color;
}

} // end namespace vision_utils

#endif // UCHAR_BGR_COLOR_TO_ROS_RGBA_COLOR_H
