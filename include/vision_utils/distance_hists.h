/*!
  \file        distance_hists.h
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

#ifndef DISTANCE_HISTS_H
#define DISTANCE_HISTS_H
// std includes
#include <algorithm> // for std::min(), std::max()...
#include <opencv2/core/core.hpp>
#include <vision_utils/normalize_hist.h>

namespace vision_utils {

/*! distance between two histograms
 * \param h1
 *   First compared histogram.
 * \param h2
 *   Second compared histogram of the same size as H1.
 * \param remaps_to_0_1
 *  true for remapping value to 0 .. 1.
 *  In that case, h1 and h2 must be normalized
 * \param method
 *  cf cv::compareHist():
 *  CV_COMP_CHISQR: sums the normalized square difference between the bins
 *  CV_COMP_INTERSECT: simply compares, for each bin, the two values
 *  in each histogram, and keeps the minimum one.
 *  The similarity measure is then simply the sum of these minimum values.
 *  Consequently, two images having histograms with no colors in common would
 *  get an intersection value of 0, while two identical histograms would get a
 *  value equal to the total number of pixels.
 *  CV_COMP_CORREL: is based on the normalized cross-correlation operator used
 *  in signal processing to measure the similarity between two signals
 *  CV_COMP_BHATTACHARYYA: used in statistics to estimate the similarity between
 *  two probabilistic distributions
 * \return
 *   the value normalization
 *   if remaps_to_0_1, return 0 for identical histograms
 *                     and 1 for completely different histograms
 */
double distance_hists(const Histogram & h1,
                      const Histogram & h2,
                      const int method = CV_COMP_BHATTACHARYYA,
                      bool remaps_to_0_1 = true) {
  double raw_value = cv::compareHist(h1, h2, method);
  if (!remaps_to_0_1)
    return raw_value;
  switch (method) {
    case CV_COMP_INTERSECT: // 0 = very diff -> 1 = identical
      return 1. - raw_value;
    case CV_COMP_CHISQR: // 2 = very diff -> 0 = identical
      return raw_value / 2.;
    case CV_COMP_CORREL: //  -1 = opposite -> 0 = very diff -> 1 = identical
      return std::min(1., 1. - raw_value);
      //return (-raw_value + 1) / 2.f;
    case CV_COMP_BHATTACHARYYA: // 1 = very diff -> 0 = identical
    default:
      return raw_value;
  } // end switch (method)
} // end distance_hists()

} // end namespace vision_utils

#endif // DISTANCE_HISTS_H
