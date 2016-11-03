/*!
  \file        mean_std_dev.h
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

#ifndef MEAN_STD_DEV_H
#define MEAN_STD_DEV_H
// std includes
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

template<class T>
inline void mean_std_dev
(const T* data, unsigned int data_size, double & mean, double & std_dev)
{
  if (data_size == 0) {
    printf("mean_std_dev(): empty data\n");
    mean = 0;
    std_dev = 0;
    return;
  }
  // mean
  mean = 1.f * std::accumulate(data, data + data_size, (T) 0) / data_size;

  // std dev
  double sum_squared = 0;
  for (unsigned int i = 0; i < data_size; ++i)
    sum_squared += 1.f * (data[i] - mean) * (data[i] - mean);
  std_dev =sqrt(sum_squared / (data_size - 1));
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline void mean_std_dev
(const std::vector<T> & data, double & mean, double & std_dev)
{
  mean_std_dev(data.data(), data.size(), mean, std_dev);
}

} // end namespace vision_utils

#endif // MEAN_STD_DEV_H
