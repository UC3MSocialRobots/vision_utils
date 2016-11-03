/*!
  \file        drawLine.h
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

#ifndef DRAWLINE_H
#define DRAWLINE_H
// std includes
#include <opencv2/core/core.hpp>

namespace vision_utils {

inline void drawLine(cv::Mat & img,
                     const double & a,
                     const double & b,
                     const double & c,
                     const cv::Scalar & color,
                     const int thickness = 2) {
  //cout << "line : 0," << (int) (-eqn.val[2] / eqn.val[1]) << " - 100, " << (int) ((-100 * eqn.val[0] - eqn.val[2]) / eqn.val[1]) << endl;
  cv::line(img,
           cv::Point(0, (int) (-c / b)),
           cv::Point(1000, (int) ((-1000 * a - c) / b)),
           color, thickness);
}

} // end namespace vision_utils

#endif // DRAWLINE_H
