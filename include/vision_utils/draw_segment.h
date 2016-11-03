/*!
  \file        draw_segment.h
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

#ifndef DRAW_SEGMENT_H
#define DRAW_SEGMENT_H
// std includes
#include <opencv2/core/core.hpp>

namespace vision_utils {

inline void draw_segment(cv::Mat & img,
                         const cv::Vec4i & segment_ends,
                         const cv::Scalar & color,
                         const int thickness = 2) {
  cv::line(img,
           cv::Point2i(segment_ends[0], segment_ends[1]),
      cv::Point2i(segment_ends[2], segment_ends[3]),
      color, thickness);
}

} // end namespace vision_utils

#endif // DRAW_SEGMENT_H
