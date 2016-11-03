/*!
  \file        get_hue_histogram.h
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

#ifndef GET_HUE_HISTOGRAM_H
#define GET_HUE_HISTOGRAM_H
// std includes
#include <opencv2/core/core.hpp>
#include <vision_utils/rgb2hue.h>

namespace vision_utils {

//! Computes the 1D histogram of the first channel of the image
Histogram get_hue_histogram(const cv::Mat3b &rgb,
                            cv::Mat3b& hsv_buffer,
                            cv::Mat1b& hue_buffer,
                            const int & nbins, const double max_value,
                            const cv::Mat & mask = cv::Mat(),
                            bool want_normalize_hist = true) {
  rgb2hue(rgb, hsv_buffer, hue_buffer);
  return get_histogram(hue_buffer, nbins, max_value, mask, want_normalize_hist);
}

} // end namespace vision_utils

#endif // GET_HUE_HISTOGRAM_H
