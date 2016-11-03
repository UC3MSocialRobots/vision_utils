/*!
  \file        drawPolygon.h
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

#ifndef DRAWPOLYGON_H
#define DRAWPOLYGON_H
// std includes
#include <opencv2/core/core.hpp>
#include <vector>

namespace vision_utils {

inline void drawPolygon(cv::Mat & img,
                        const std::vector<cv::Point> & poly,
                        bool is_closed,
                        const cv::Scalar & color,
                        const int thickness =1,
                        const int line_type = 8,
                        const int shift = 0) {
  if (poly.size() == 0)
    return;
  if (poly.size() == 1) {
    cv::circle(img, poly[0], thickness, color, -1, line_type, shift);
    return;
  }
  for (unsigned int pt_idx = 0; pt_idx < poly.size() - 1; ++pt_idx)
    cv::line(img, poly[pt_idx], poly[pt_idx + 1], color, thickness,line_type, shift);
  if (is_closed)
    cv::line(img, poly[poly.size() - 1], poly[0], color, thickness,line_type, shift);
}

} // end namespace vision_utils

#endif // DRAWPOLYGON_H
