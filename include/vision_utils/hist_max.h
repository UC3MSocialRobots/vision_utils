/*!
  \file        hist_max.h
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

#ifndef HIST_MAX_H
#define HIST_MAX_H
// std includes
#include <stdio.h> // for printf(), etc

namespace vision_utils {

bool hist_max(const Histogram & hist,
              double & max_bin_value, double & max_bin_index,
              bool skip_first_bin = false) {
  int nbins = hist.rows;
  if (nbins < (skip_first_bin ? 2 : 1)) {
    printf("max_bin_value(): histogram too small\n");
    return false;
  }
  max_bin_value = 0;
  max_bin_index = -1;
  for (int bin_idx = (skip_first_bin ? 1 : 0); bin_idx < nbins; ++bin_idx) {
    float bin_value = hist.at<float>(bin_idx);
    if (max_bin_value < bin_value) {
      max_bin_value = bin_value;
      max_bin_index = bin_idx;
    }
  } // end loop bin_idx
  return true;
}

} // end namespace vision_utils

#endif // HIST_MAX_H
