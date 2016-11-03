/*!
  \file        rectangle_to_string.h
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

#ifndef RECTANGLE_TO_STRING_H
#define RECTANGLE_TO_STRING_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

/*!
 *build a string to represent a rectangle
 *\param rect
 *\return something like (x, y)+(w, h)
 */
inline std::string rectangle_to_string(const cv::Rect & rect) {
  std::ostringstream concl;
  concl << "(" << rect.x << ", " << rect.y << ")";
  concl << "+(" << rect.width << ", " << rect.height << ")";
  return concl.str();
}

} // end namespace vision_utils

#endif // RECTANGLE_TO_STRING_H
