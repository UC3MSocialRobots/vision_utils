/*!
  \file        mean_std_dev_grouped_data.h
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

#ifndef MEAN_STD_DEV_GROUPED_DATA_H
#define MEAN_STD_DEV_GROUPED_DATA_H
// std includes
#include <stdio.h> // for printf(), etc
#include <vector>
#include <numeric> // for accumulate

namespace vision_utils {

// from http://www.angelfire.com/blues/michaelyang/ive/dms/chapter_05/5_6_StaDev.html
template<class T>
inline void mean_std_dev_grouped_data
(const T* mid_pts, const T* occurs, unsigned int data_size,
 double & mean, double & std_dev)
{
  if (data_size == 0) {
    printf("mean_std_dev_grouped_data(): empty mid_pts\n");
    mean = 0;
    std_dev = 0;
    return;
  }
  // mean
  double sum_occur = std::accumulate(occurs, occurs + data_size, (T) 0),
      sum_occur_inv = 1. / sum_occur;
  //printf("sum_occur:%g\n", sum_occur);
  if (fabs(sum_occur) < 1E-10) {
    printf("mean_std_dev_grouped_data(): sum_occur=0\n");
    mean = 0;
    std_dev = 0;
    return;
  }
  double sum_crossed = 0;
  for (unsigned int i = 0; i < data_size; ++i)
    sum_crossed += 1.f * mid_pts[i] * occurs[i];
  mean = sum_crossed * sum_occur_inv;
  // std_dev
  double sum_squared = 0;
  for (unsigned int i = 0; i < data_size; ++i)
    sum_squared += 1.f * mid_pts[i] * mid_pts[i] * occurs[i];
  if (sum_occur > data_size) // use sample variance (s^2)
    std_dev = sqrt((sum_squared - sum_crossed * sum_crossed * sum_occur_inv) / (sum_occur - 1));
  else // use population variance (s^2)
    std_dev = sqrt((sum_squared - sum_crossed * sum_crossed * sum_occur_inv) * sum_occur_inv);
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline void mean_std_dev_grouped_data
(const std::vector<T> & mid_pts, const std::vector<T>& occurs,
 double & mean, double & std_dev)
{
  if (mid_pts.size() != occurs.size()) {
    printf("mean_std_dev_grouped_data(): mid_pts.size()=%li != occurs.size()=%li\n",
           mid_pts.size(), occurs.size());
    return;
  }
  mean_std_dev_grouped_data(mid_pts.data(), occurs.data(), mid_pts.size(), mean, std_dev);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief mean_std_dev_grouped_data_modulo
 * \param mid_pts
 * \param occurs
 * \param min_pt, max_pt
 *    the pts of mid_pts are such as min_pt = max_pt
 *    (makes sense for instance for the hue component, where 0 = 360)
 * \param data_size
 * \param mean
 * \param std_dev
 */
template<class T>
inline void mean_std_dev_grouped_data_modulo
(const std::vector<T> & mid_pts, const std::vector<T> & occurs,
 T min_pt, T max_pt, std::vector<T> & mid_pts_shifted,
 double & mean, double & std_dev)
{
  if (mid_pts.size() != occurs.size()) {
    printf("mean_std_dev_grouped_data(): mid_pts.size()=%li != occurs.size()=%li\n",
           mid_pts.size(), occurs.size());
    return;
  }
  unsigned int data_size = mid_pts.size();
  if (data_size == 0) {
    printf("mean_std_dev_grouped_data(): empty mid_pts\n");
    mean = 0;
    std_dev = 0;
    return;
  }
  // first find peak
  T peak_pt = 0, max_occur = 0;
  for (unsigned int i = 0; i < data_size; ++i) {
    if (occurs[i] > max_occur) {
      max_occur = occurs[i];
      peak_pt = mid_pts[i];
    }
  } // end loop i
  // determine the needed shift: center the peak, i.e. peak -> (min_pt + max_pt)/2
  double pt_shift = .5 * ((double) min_pt + (double) max_pt) - peak_pt;
  //printf("pt_shift:%g\n", pt_shift);

  // then shift the pt values
  mid_pts_shifted.resize(data_size);
  for (unsigned int i = 0; i < data_size; ++i) {
    mid_pts_shifted[i] = modulo_real((T) (mid_pts[i] + pt_shift), min_pt, max_pt);
    //printf("data:%g:%g\n", (double) mid_pts_shifted[i], (double) occurs[i]);
  } // end loop i

  mean_std_dev_grouped_data(mid_pts_shifted, occurs, mean, std_dev);

  // now shift back mean
  mean = modulo_real((T) (mean - pt_shift), min_pt, max_pt);
}

} // end namespace vision_utils

#endif // MEAN_STD_DEV_GROUPED_DATA_H
