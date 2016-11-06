/*!
  \file        draw_ellipses.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/3/10

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

Some plugins to draw more stuff with a MiniStage.

 */

#ifndef DRAW_ELLIPSES_H
#define DRAW_ELLIPSES_H

#include <opencv2/core/core.hpp>

namespace vision_utils {

typedef cv::RotatedRect Ellipse;

static const double RAD2DEG_ = 57.2957795130823208768;
static const double DEG2RAD_ = 0.01745329251994329577;

template<class Pt2>
Ellipse three_pts2ellipse(const Pt2 & center, const Pt2 & end1, const Pt2 & end2) {
  Pt2 V1 = end1 - center, V2 = end2 - center;
  return Ellipse(center,
                 cv::Size2f(2 * hypot(V1.x, V1.y), 2 * hypot(V2.x, V2.y)),
                 atan2(V1.y, V1.x) * RAD2DEG_);
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
void ellipse_axes(const Ellipse & e,
                  Pt2 & long_axis_end1, Pt2 & long_axis_end2,
                  Pt2 & short_axis_end1, Pt2 & short_axis_end2) {
  double short_radius = e.size.width / 2, long_radius = e.size.height / 2,
      cosa = cos(e.angle * DEG2RAD_), sina = sin(e.angle * DEG2RAD_);
  short_axis_end1.x = e.center.x + short_radius * cosa;
  short_axis_end1.y = e.center.y + short_radius * sina;
  short_axis_end2.x = 2 * e.center.x - short_axis_end1.x;
  short_axis_end2.y = 2 * e.center.y - short_axis_end1.y;
  // long: angle = angle + PI/2
  // -> cos(angle + PI/2) = -sin(angle), sin(angle + PI/2) = cos(angle)
  long_axis_end1.x = e.center.x - long_radius * sina;
  long_axis_end1.y = e.center.y + long_radius * cosa;
  long_axis_end2.x = 2 * e.center.x - long_axis_end1.x;
  long_axis_end2.y = 2 * e.center.y - long_axis_end1.y;

  if (short_radius > long_radius) {
    std::swap(long_axis_end1, short_axis_end1);
    std::swap(long_axis_end2, short_axis_end2);
  }
}

} // end namespace vision_utils

#endif // DRAW_ELLIPSES_H
