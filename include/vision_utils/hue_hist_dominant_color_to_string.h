/*!
  \file        hue_hist_dominant_color_to_string.h
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

#ifndef HUE_HIST_DOMINANT_COLOR_TO_STRING_H
#define HUE_HIST_DOMINANT_COLOR_TO_STRING_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <string>
#include <vision_utils/is_histogram_empty.h>
#include <vision_utils/hist_max.h>
#include <vision_utils/hue_to_string.h>


namespace vision_utils {

inline std::string hue_hist_dominant_color_to_string(const Histogram & hist,
                                                     bool skip_first_bin = false) {
  if (is_histogram_empty(hist))
    return "empty";

  int nbins = hist.rows, nread_bins = (skip_first_bin ? nbins - 1 : nbins);
  double max_bin_value, max_bin_index;
  if (!hist_max(hist, max_bin_value, max_bin_index, skip_first_bin))
    return "histogram too small";

  // check distribution of colors
  double uniform_bin_value =
      (cv::norm(hist, cv::NORM_L1) - (skip_first_bin ? hist.at<float>(0) : 0))
      / nread_bins;
  if (max_bin_index < 0 || max_bin_value < 3 * uniform_bin_value)
    return "multicolor";
  // one dominant color
  float h = 180.f * max_bin_index / nbins;
  //  printf("h:%f\n", h);
  //  cv::Vec3b h2rgb = hue2rgb<cv::Vec3b>(h);
  //  std::cout  << "h2rgb:" << h2rgb << std::endl;
  return hue_to_string(h);
} // end hue_hist_dominant_color_to_string()

} // end namespace vision_utils

#endif // HUE_HIST_DOMINANT_COLOR_TO_STRING_H
