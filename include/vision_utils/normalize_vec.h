/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/11/12
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

  odo Description of the file
 */

#ifndef NORMALIZE_VEC_H
#define NORMALIZE_VEC_H
#include <vector>
#include <algorithm>
#include <math.h>

namespace vision_utils {

// from opencv2/core/base.hpp
enum NormTypes { NORM_INF       = 1,
                 NORM_L1        = 2,
                 NORM_L2        = 4,
               };

template<class T>
double normL1_vec(const std::vector<T> & V) {
  unsigned int n = V.size();
  double sum = 0;
  for (unsigned int i = 0; i < n; ++i)
    sum += fabs(V[i]);
  return sum;
}

////////////////////////////////////////////////////////////////////////////////


template<class T>
double normL2_vec(const std::vector<T> & V) {
  unsigned int n = V.size();
  double sum = 0;
  for (unsigned int i = 0; i < n; ++i)
    sum += V[i] * V[i];
  return sqrt(sum);
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
double norminf_vec(const std::vector<T> & V) {
  if (V.empty())
    return 0;
  unsigned int n = V.size();
  double max = V[0];
  for (unsigned int i = 1; i < n; ++i)
    if (V[i] > max)
      max = V[i];
  return max;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
double norm_vec(const std::vector<T> & V, NormTypes normtype) {
  switch (normtype) {
    case NORM_INF:
      return norminf_vec(V);
    case NORM_L1:
      return normL1_vec(V);
    case NORM_L2:
    default:
      return normL2_vec(V);
  }
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
void normalize_vec(std::vector<T> & V, NormTypes normtype = NORM_L2) {
  double norm = norm_vec(V, normtype), norminv = 1 / norm;
  if (fabs(norm) < 1E-10) // 0 norm
    return;
  unsigned int n = V.size();
  for (unsigned int i = 1; i < n; ++i)
    V[i] *= norminv;
}

} // end namespace vision_utils
#endif // NORMALIZE_VEC_H

