/*!
  \file        two_vectors_closer_than_threshold.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/5
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

#ifndef TWO_VECTORS_CLOSER_THAN_THRESHOLD_H
#define TWO_VECTORS_CLOSER_THAN_THRESHOLD_H
#include <vector>

namespace vision_utils {

/*!
 \param A
    a vector of 2D points
 \param B
    a vector of 2D points
 \param min_dist
    a threshold distance
 \return bool
    true if there is a a pair of points closer than min_dist
*/
template<class _Pt2>
inline bool two_vectors_closer_than_threshold(const std::vector<_Pt2> & A,
                                              const std::vector<_Pt2> & B,
                                              const float min_dist) {
  float min_dist_sq = min_dist * min_dist;
  for (unsigned int A_idx = 0; A_idx < A.size(); ++A_idx) {
    for (unsigned int B_idx = 0; B_idx < B.size(); ++B_idx) {
      if (distance_points_squared(A[A_idx], B[B_idx])
          < min_dist_sq)
        return true;
    } // end loop B_idx
  } // end loop A_idx
  return false;
} // end two_vectors_closer_than_threshold()

////////////////////////////////////////////////////////////////////////////////

} // end namespace vision_utils

#endif // TWO_VECTORS_CLOSER_THAN_THRESHOLD_H
