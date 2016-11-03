/*!
  \file        merge_histograms.h
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

#ifndef MERGE_HISTOGRAMS_H
#define MERGE_HISTOGRAMS_H
// std includes
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

/*!
 * Merge two histograms as an arithmetic average
 * \param h1, h2
 *    input histograms
 * \param out
 *    output histogram
 * \param want_normalize_hist
 *    true for normalizing the hist
 */
bool merge_histograms(const Histogram & h1,
                      const Histogram & h2,
                      Histogram & out,
                      double weight1 = 1,
                      double weight2 = 1,
                      bool want_normalize_hist = true) {
  if(h1.rows != h2.rows) { // check same number of bins
    printf("merge_histograms(): different nb of bins: %i, %i! Returning\n",
           h1.rows, h2.rows);
    return false;
  }
  // make average
  if (weight1 == 1 && weight2 == 1) // avoid consuming and useless multiplication
    out = h1 + h2;
  else
    out = weight1 * h1 + weight2 * h2;
  if (want_normalize_hist)
    normalize_hist(out);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Merge two histograms as an arithmetic average
 * \param h1, h2
 *    input histograms
 * \param out
 *    output histogram
 * \param want_normalize_hist
 *    true for normalizing the hist
 */
bool merge_histograms(const std::vector<Histogram> & hists,
                      const std::vector<double> & weights,
                      Histogram & out,
                      bool want_normalize_hist = true) {
  unsigned int nhists = hists.size();
  if (weights.size() != nhists) { // check size consistency
    printf("merge_histograms(): %li weights, %i histograms! Returning\n",
           weights.size(), nhists);
    return false;
  }
  if (hists.size() == 0) {
    out = Histogram();
    return true;
  }

  out = weights.front() * hists.front();
  for (unsigned int hist_idx = 1; hist_idx < nhists; ++hist_idx) {
    if(out.rows != hists[hist_idx].rows) { // check same number of bins
      printf("merge_histograms(): different nb of bins: hist#%i:%i, out:%i! Returning\n",
             hist_idx, hists[hist_idx].rows, out.rows);
      return false;
    }
    if (weights[hist_idx] == 0) // skip weights equal to zero
      continue;
    out += weights[hist_idx] * hists[hist_idx];
  }
  if (want_normalize_hist)
    normalize_hist(out);
  return true;
}

} // end namespace vision_utils

#endif // MERGE_HISTOGRAMS_H
