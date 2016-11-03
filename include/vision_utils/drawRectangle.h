/*!
  \file        drawRectangle.h
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

#ifndef DRAWRECTANGLE_H
#define DRAWRECTANGLE_H
// std includes
#include <opencv2/core/core.hpp>

namespace vision_utils {

inline void drawRectangle(cv::Mat & img, const cv::Rect & r, const cv::Scalar & color,
                          const int thickness =1,
                          const int line_type =8,
                          const int shift =0) {
  cv::rectangle(img, cv::Point(r.x, r.y),
                cv::Point(r.x + r.width, r.y + r.height),
                color, thickness, line_type, shift);
}

} // end namespace vision_utils

#endif // DRAWRECTANGLE_H
