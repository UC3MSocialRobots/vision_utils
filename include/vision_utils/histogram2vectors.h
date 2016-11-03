/*!
  \file        histogram2vectors.h
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

#ifndef HISTOGRAM2VECTORS_H
#define HISTOGRAM2VECTORS_H
// std includes
#include <vector>

namespace vision_utils {

void histogram2vectors(const Histogram & h, const double max_value,
                       std::vector<double> & bins, std::vector<double> & freqs) {
  unsigned int nbins = h.rows;
  bins.resize(nbins);
  freqs.resize(nbins);
  double hstep = 1. * max_value / nbins;
  for (unsigned int bin_idx = 0; bin_idx < nbins; ++bin_idx) {
    bins[bin_idx] = hstep * (.5 + bin_idx);
    freqs[bin_idx] = h.at<float>(bin_idx);
    //printf("bins:%g, freqs:%g\n", bins[bin_idx], freqs[bin_idx]);
  } // end loop bin_idx
} // histogram2vectors();

} // end namespace vision_utils

#endif // HISTOGRAM2VECTORS_H
