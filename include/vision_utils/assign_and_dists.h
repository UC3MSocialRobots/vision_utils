/*!
  \file        assign_and_dists.h
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

#ifndef ASSIGN_AND_DISTS_H
#define ASSIGN_AND_DISTS_H
// std includes
#include <vector>
#include "vision_utils/cmatrix.h"
#include <vision_utils/distance_points_squared.h>
#include "vision_utils/hausdorff_distances.h"
#include "vision_utils/linear_assign.h"
#include "vision_utils/match.h"

namespace vision_utils {

/*!
  A solver for the Linear Assignment Problem
  ( en.wikipedia.org/wiki/Assignment_problem )
 It computes distances between each pair of objects to match,
 then calls linear_assign()

 \param v1
    a vector of objects to match to objects in v2
 \param v2
    a vector of objects to match to objects in v1
 \param costs_buffer
  A matrix buffer where we will store costs of all pairs between v1 and v2
 \param best_assign
  (out) the resulting best AssignmentList
 \param best_cost
  (out) the cost of best_assign
 \param cost_func
  the distance function between elements of v1 and of v2.
  It should be fast.
 \param assign_func
   the assignement function for costs matrices.
   It is advisable to use linear_assign() and not brute_force_assign(),
   the former being MUCH faster and as accurate.
 \return bool
   true if success.
*/
template<class T1, class T2>
bool assign_and_dists(const std::vector<T1> & v1,
                      const std::vector<T2> & v2,
                      CMatrix<Cost> & costs_buffer,
                      MatchList & best_assign,
                      Cost & best_cost,
                      Cost (*cost_func)(const T1 &, const T2 &)
                      = &(distance_points_squared),
                      bool (*assign_func)
                      (CMatrix<Cost> &, MatchList &, Cost &)
                      = &linear_assign
                        ) {
  //printf("assign_and_dists()\n");
  costs_buffer.resize(v1.size(), v2.size());
  for (unsigned int v1_idx = 0; v1_idx < v1.size(); ++v1_idx)
    for (unsigned int v2_idx = 0; v2_idx < v2.size(); ++v2_idx)
      costs_buffer[v1_idx][v2_idx] = cost_func(v1[v1_idx], v2[v2_idx]);
  //debugStream("costs_buffer:" << costs_buffer.to_string());

  return assign_func(costs_buffer, best_assign, best_cost);
} // end assign_and_dists()

////////////////////////////////////////////////////////////////////////////////


/*!
// * Call bruteforce_distance() to match each of the points of \arg queries
// * with the \arg features set.
// * \return  the aggregated L1 distance between \arg features and the points of
// * \arg queries
// */
template<class Pt2>
class LapDistance2D {
public:
  inline float distance(const std::vector<Pt2> & A,
                        const std::vector<Pt2> & B) {
    debugStream("A:" << iterable_to_string(A)
                << "\nB:" << iterable_to_string(B) << std::endl);
    assign_and_dists<Pt2, Pt2>
        (A, B, _costs_buffer, _assign, _dist_AB,
         //&distance_points_squared
         &dist_Linf_double<Pt2>
         );
    return _dist_AB;
  }

  CMatrix<Cost> _costs_buffer;
  MatchList _assign;
  Cost _dist_AB;
};

} // end namespace vision_utils

#endif // ASSIGN_AND_DISTS_H
