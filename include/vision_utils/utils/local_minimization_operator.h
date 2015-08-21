/*!
  \file        local_minimization_operator.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/9/20

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

This implements the Local Minimization Operator,  as defined in
M. Lindstrom and J. Eklundh,
"Detecting and tracking moving objects from a mobile platform using
a laser range scanner"
Intelligent Robots and Systems,  …, 2001.
 */

#ifndef LOCAL_MINIMIZATION_OPERATOR_H
#define LOCAL_MINIMIZATION_OPERATOR_H

#include <algorithm>

/*! Given a vector q with n elements,
a bounded minimization operator can be applied,
that sets each element equal
to the minimum of all elements in a vicinity bounded by delta.
The only reason for the different ranges in k is that
the minimization cannot be done outside the defined
range of qi.

 \param q
 \param q_out
    Must be of size q_size, otherwise will generate segfaults!
 \param q_size
 \param window_size

*/
template<class _Container>
inline void apply_local_minimization(const _Container & q,
                                     _Container & q_out,
                                     const int & q_size,
                                     const int & window_size) {
  int neigh_min, neigh_max;
  for (int q_idx = 0; q_idx < q_size; ++q_idx) {
    neigh_min = std::max(q_idx - window_size, 0);
    neigh_max = std::min(q_idx + window_size, q_size - 1);
    q_out[q_idx] = q[neigh_min];
    for (int neigh_idx = neigh_min + 1; neigh_idx <= neigh_max; ++neigh_idx) {
      if (q_out[q_idx] > q[neigh_idx])
        q_out[q_idx] = q[neigh_idx];
    } // end loop neigh_idx
  } // end loop q_idx
}

#endif // LOCAL_MINIMIZATION_OPERATOR_H
