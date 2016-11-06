/*!
  \file        mean.h
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

#ifndef MEAN_H
#define MEAN_H
// std includes
#include <numeric> // for accumulate
#include <stdio.h> // for printf(), etc

namespace vision_utils {

template<class T>
inline double mean
(const T* data, unsigned int data_size)
{
  if (data_size == 0) {
    printf("mean(): empty data\n");
    return 0;
  }
  return 1.f * std::accumulate(data, data + data_size, (T) 0) / data_size;
} // end mean()

} // end namespace vision_utils

#endif // MEAN_H
