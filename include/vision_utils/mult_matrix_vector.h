/*!
  \file        mult_matrix_vector.h
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

#ifndef MULT_MATRIX_VECTOR_H
#define MULT_MATRIX_VECTOR_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 * computes M times V with OpenCV matrix mult routines.
 * However, this routine only takes 2 matrices as arguments,
 * so here we do a nice cast.
 * @param M a 3X3 matrix
 * @param V a 1X3 std::vector
 * @return M * V a 1X3 std::vector
 */
template<class Vector3, class Mat1f>
static inline Vector3 mult_matrix_vector(const Mat1f & M, const Vector3 & V) {
  // convert it Vec -> Mat
  //const float V_array[3][1] = { V[0], V[1], V[2] };
  //Mat1f V_matrix(V);
  Mat1f V_matrix(3, 1);
  V_matrix(0, 0) = V.x;
  V_matrix(1, 0) = V.y;
  V_matrix(2, 0) = V.z;
  //printf("size:%i x %i", V_matrix.cols, V_matrix.rows);
  //V_matrix.t();
  // make the transfo
  Mat1f M_times_V_matrix = M * V_matrix;
  // convert the result Mat -> Vec
  //    for (int comp_idx = 0; comp_idx < 3; ++comp_idx)
  //        M_times_V[comp_idx] = M_times_V_matrix(comp_idx, 0);
  Vector3 M_times_V(
        M_times_V_matrix(0, 0),
        M_times_V_matrix(1, 0),
        M_times_V_matrix(2, 0)
        );
  return M_times_V;
}

} // end namespace vision_utils

#endif // MULT_MATRIX_VECTOR_H
