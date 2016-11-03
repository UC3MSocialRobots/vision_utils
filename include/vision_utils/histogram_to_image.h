/*!
  \file        histogram_to_image.h
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

#ifndef HISTOGRAM_TO_IMAGE_H
#define HISTOGRAM_TO_IMAGE_H
// std includes
#include <opencv2/core/core.hpp>
#include <vision_utils/colormaps.h>
#include <vision_utils/hue_hist_dominant_color_to_string.h>
#include <vision_utils/draw_text_centered.h>

namespace vision_utils {

// Computes the 1D histogram and returns an image of it.
void histogram_to_image(const Histogram &hist,
                        cv::Mat3b & histImg,
                        const int width, const int height,
                        RatioColormap ratio_colormap
                        = ratio2grey){
  // Image on which to display histogram
  histImg.create(height, width);
  histImg.setTo(cv::Scalar::all(255));
  if (hist.empty())
    return;
  // Get min and max bin values
  double maxVal=0;
  double minVal=0;
  cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
  // special case: minVal = maxVal
  if (minVal == maxVal)
    maxVal = minVal + 1;
  int hpt = 0.1 * height; // set highest point at 90% of height
  // find the linear mapping   "y = alpha * x + beta" such as
  // 0 -> height
  // maxVal -> hpt
  double alpha = (hpt - height) / maxVal, beta = hpt - alpha * maxVal;
  int nbins = hist.rows;
  double bin_width = 1.f * width / nbins;
  // draw average
  int average_px = alpha / nbins + beta;
  cv::line(histImg, cv::Point(0, average_px), cv::Point(histImg.cols, average_px),
           cv::Scalar::all(150), 1);
  // write dominant color
  if (ratio_colormap == ratio2hue)
    draw_text_centered
        (histImg, hue_hist_dominant_color_to_string(hist, true),
         cv::Point(width / 2, hpt / 2), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));
  //    cv::putText(histImg, hue_hist_dominant_color_to_string(hist, true),
  //                cv::Point(10, hpt - 3), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));
  // Draw a vertical line for each bin
  for( int bin_idx = 0; bin_idx < nbins; bin_idx++ ) {
    float binVal = hist.at<float>(bin_idx);
    //printf("bin_idx:%i, binVal:%g\n", bin_idx, binVal);
    cv::Point p1(bin_idx * bin_width, beta); // a zero value
    cv::Point p2((bin_idx + 1) * bin_width, alpha * binVal + beta);
    //printf("p1:(%i, %i), p2:(%i, %i)\n", p1.x, p1.y, p2.x, p2.y);
    cv::rectangle(histImg, p1, p2, ratio_colormap(1.f * bin_idx / nbins), -1);
    cv::rectangle(histImg, p1, p2, cv::Scalar::all(0), 1);
  }
} // end histogram_to_image();

} // end namespace vision_utils

#endif // HISTOGRAM_TO_IMAGE_H
