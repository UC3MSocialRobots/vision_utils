/*!
  \file        distance_hist_vectors.h
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

#ifndef DISTANCE_HIST_VECTORS_H
#define DISTANCE_HIST_VECTORS_H
// std includes
#include <stdio.h> // for printf(), etc
#include <vector>
#include "vision_utils/is_histogram_empty.h"
#include "vision_utils/distance_hists.h"

namespace vision_utils {

/*!
 * Compute the average distance of two vectors of histograms.
 * \brief distance_hists
 * \param hists1, hists2
 *  two vectors of histograms, of same size
 * \param method, remaps_to_0_1
 *   cf distance_hists()
 * \return
 *   the average distance between corresonding pairs (hists1, hists2)
 */
inline double distance_hist_vectors(const std::vector<Histogram> & hists1,
                                    const std::vector<Histogram> & hists2,
                                    const int method = CV_COMP_BHATTACHARYYA,
                                    bool remaps_to_0_1 = true) {
  unsigned int nhists1 = hists1.size();
  unsigned int nhists2 = hists2.size();
  if (nhists1 != nhists2) {
    printf("vectors of Histogram with a different number of histograms!"
           "%i != %i\n", nhists1, nhists2);
    return -1;
  }
  unsigned int nhists_non_empty = 0;
  double sum = 0;
  for (unsigned int hist_idx = 0; hist_idx < nhists1; ++hist_idx) {
    if (is_histogram_empty(hists1[hist_idx]) || is_histogram_empty(hists2[hist_idx]))
      continue;
    ++nhists_non_empty;
    sum += distance_hists
           (hists1[hist_idx], hists2[hist_idx], method, remaps_to_0_1);
  } // end loop hist_idx

  return sum / nhists_non_empty;
} // end distance_hist_vectors()

} // end namespace vision_utils

#endif // DISTANCE_HIST_VECTORS_H
