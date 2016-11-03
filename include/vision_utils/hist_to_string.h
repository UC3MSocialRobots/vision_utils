/*!
  \file        hist_to_string.h
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

#ifndef HIST_TO_STRING_H
#define HIST_TO_STRING_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

inline std::string hist_to_string(const Histogram &hist) {
  std::ostringstream bin_content;
  int nbins = hist.rows;
  // float sum = 0;
  float sum = cv::norm(hist, cv::NORM_L1); // fast computing instead of manual summing
  for( int bin_idx = 0; bin_idx < nbins; bin_idx++ ) {
    float binVal = hist.at<float>(bin_idx);
    bin_content << "[" << bin_idx << ": " << binVal << "] ";
    // sum += binVal;
  }
  std::ostringstream out;
  out << "hist: " << nbins << " bins"
         // << ", sum:" << sum
      << ", norm: L1 (sum): " << sum
      << ", L2: " << cv::norm(hist, cv::NORM_L2)
      << ", INF: " << cv::norm(hist, cv::NORM_INF)
      << ", average value: " << sum / nbins
      << ", bin_content:" << bin_content.str();
  return out.str();
} // end hist_to_string()

} // end namespace vision_utils

#endif // HIST_TO_STRING_H
