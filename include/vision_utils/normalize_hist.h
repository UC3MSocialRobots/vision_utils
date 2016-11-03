/*!
  \file        normalize_hist.h
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

#ifndef NORMALIZE_HIST_H
#define NORMALIZE_HIST_H
// std includes
#include <opencv2/core/core.hpp>

namespace vision_utils {

typedef cv::MatND Histogram;

//! normalizes the histogram bins by scaling them so that the sum of the bins becomes equal to factor.
void normalize_hist(Histogram & hist, const double factor = 1.0) {
  cv::normalize(hist, hist, factor, 0, cv::NORM_L1);
} // end normalize_hist();

} // end namespace vision_utils

#endif // NORMALIZE_HIST_H
