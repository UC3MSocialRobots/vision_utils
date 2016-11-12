/*!
  \file        vector_of_histograms_to_image.h
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

#ifndef VECTOR_OF_HISTOGRAMS_TO_IMAGE_H
#define VECTOR_OF_HISTOGRAMS_TO_IMAGE_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>
#include "vision_utils/colormaps.h"
#include "vision_utils/normalize_hist.h"
#include "vision_utils/histogram_to_image.h"

namespace vision_utils {

void vector_of_histograms_to_image(const std::vector<Histogram> & hists,
                                   cv::Mat3b & histImg,
                                   const int width1, const int height1,
                                   RatioColormap ratio_colormap
                                   = ratio2grey,
                                   std::vector<bool>* refresh_mask = NULL){
  unsigned int n_hists = hists.size();
  // check refresh_mask
  bool use_refresh_mask = (refresh_mask != NULL);
  if (use_refresh_mask && refresh_mask->size() != n_hists) {
    printf("vector_of_histograms_to_image(): error, refresh_mask->size()=%li != n_hists=%i\n",
           refresh_mask->size(), n_hists);
    return;
  }
  //printf("vector_of_histograms_to_image(n_hists:%i)\n", n_hists);

  // check bounds
  int min_width = width1, min_height = height1 * n_hists;
  if (histImg.cols < min_width || histImg.rows < min_height)
    histImg.create(min_height, min_width); // rows, cols

  // draw each hist
  for (unsigned int hist_idx = 0; hist_idx < n_hists; ++hist_idx) {
    // if this histogram is not in refresh_mask, do nothing
    if (use_refresh_mask && !(*refresh_mask)[hist_idx])
      continue;
    // get ROI and draw histogram in it
    cv::Mat3b histImg_roi = histImg(cv::Rect(0, height1 * hist_idx, width1, height1));
    histogram_to_image(hists[hist_idx], histImg_roi,
                                        width1, height1, ratio_colormap);
    // draw a separator line
    cv::line(histImg_roi, cv::Point(0, histImg_roi.rows - 1),
             cv::Point(histImg_roi.cols, histImg_roi.rows - 1), CV_RGB(0, 0, 0));
  } // end loop hist_idx
} // end vector_of_histograms_to_image();

} // end namespace vision_utils

#endif // VECTOR_OF_HISTOGRAMS_TO_IMAGE_H
