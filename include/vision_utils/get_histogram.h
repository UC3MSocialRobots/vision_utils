/*!
  \file        get_histogram.h
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

#ifndef GET_HISTOGRAM_H
#define GET_HISTOGRAM_H
// std includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/normalize_hist.h"

namespace vision_utils {

//! Computes the 1D histogram of the first channel of the image
Histogram get_histogram(const cv::Mat &image,
                        const int & nbins, const double max_value,
                        const cv::Mat & mask = cv::Mat(),
                        bool want_normalize_hist = true) {
  Histogram hist;
  int histSize[] = {nbins};
  float hranges[] = { 0.f, 1.f * max_value };
  const float* ranges[] = { hranges };
  int channels[] = {0};
  // Compute histogram
  cv::calcHist(&image,
               1, // histogram from 1 image only
               channels, // the channel used
               mask, // no mask is used
               hist, // the resulting histogram
               1, // it is a 1D histogram
               histSize, // number of bins
               ranges // pixel value range
               );
  if (want_normalize_hist)
    normalize_hist(hist);
  return hist;
} // end get_histogram()

} // end namespace vision_utils

#endif // GET_HISTOGRAM_H
