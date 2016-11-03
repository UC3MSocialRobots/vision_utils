/*!
  \file        distance_patterns.h
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

#ifndef DISTANCE_PATTERNS_H
#define DISTANCE_PATTERNS_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 Compare two patterns of points.
 Aligns them beforehand (rotation and translation, but no scaling).
 \param A
 \param B
 \param max_distance
 \return double
    +double::max() if the computed distance is > max_distance
*/
template<class _Pt2>
inline float distance_patterns(const std::vector<_Pt2> & A,
                               const std::vector<_Pt2> & B,
                               const float max_distance
                               = std::numeric_limits<float>::max()
                                 ) {
  std::vector<_Pt2> B_rot;
  align_polygons(A, B, B_rot);
  //printf("B_rot:%s\n", accessible_to_string(B_rot).c_str());

  // compare A and B_rot
  float sum_dist = D22_with_min<_Pt2, std::vector<_Pt2> >
      (A, B_rot, max_distance);
  // float sum_dist = 0;
  //for (unsigned int ptA_idx = 0; ptA_idx < A.size(); ++ptA_idx)
  //sum_dist += distance_point_polygon(A[ptA_idx], B_rot, false);
  return sum_dist;
} // end distance_patterns()

} // end namespace vision_utils

#endif // DISTANCE_PATTERNS_H
