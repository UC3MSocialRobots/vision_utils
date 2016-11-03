/*!
  \file        mean_std_dev_modulo.h
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

#ifndef MEAN_STD_DEV_MODULO_H
#define MEAN_STD_DEV_MODULO_H
// std includes
#include <vector>
#include <vision_utils/histogram2vectors.h>
#include <vision_utils/mean_std_dev_grouped_data.h>

namespace vision_utils {

void mean_std_dev_modulo(const Histogram & h, const double max_value,
                         double & mean, double & std_dev) {
  // first generate a vector of hue values, another of freqs
  static std::vector<double> hues, freqs, hues_shifted;
  histogram2vectors(h, max_value, hues, freqs);
  // now call mean_std_dev();
  mean_std_dev_grouped_data_modulo(hues, freqs, 0., 180., hues_shifted, mean, std_dev);
} // mean_std_dev();

} // end namespace vision_utils

#endif // MEAN_STD_DEV_MODULO_H
