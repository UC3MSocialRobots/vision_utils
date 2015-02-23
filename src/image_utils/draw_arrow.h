/*!
  \file        draw_arrow.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/12/7

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
A small function for drawing an arrow.
 */
#ifndef DRAW_ARROW_H
#define DRAW_ARROW_H

#include <opencv2/core/core.hpp>

namespace image_utils {
inline void draw_arrow
(cv::Mat3b& img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color,
 int thickness=1, int linetype=8, int shift=0) {
  // draw the body of the arrow
  cv::line(img, pt1, pt2, color, thickness, linetype, shift);
  // compute parameters of the arrow
  double side_strokes_length = hypot(pt2.y - pt1.y, pt2.x - pt1.x) / 3;
  double arrow_orien = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
  cv::Point pt_end;
  // draw first side stroke
  pt_end.x  = pt2.x + side_strokes_length * cos(arrow_orien + M_PI + M_PI / 6);
  pt_end.y  = pt2.y + side_strokes_length * sin(arrow_orien + M_PI + M_PI / 6);
  cv::line(img, pt2, pt_end, color, thickness, linetype, shift);
  // draw second side stroke
  pt_end.x  = pt2.x + side_strokes_length * cos(arrow_orien + M_PI - M_PI / 6);
  pt_end.y  = pt2.y + side_strokes_length * sin(arrow_orien + M_PI - M_PI / 6);
  cv::line(img, pt2, pt_end, color, thickness, linetype, shift);
} // end draw_arrow();
} // end namespace image_utils

#endif // DRAW_ARROW_H
