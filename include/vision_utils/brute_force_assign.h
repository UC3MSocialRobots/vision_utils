/*!
  \file        brute_force_assign.h
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

#ifndef BRUTE_FORCE_ASSIGN_H
#define BRUTE_FORCE_ASSIGN_H
// std includes
#include <algorithm> // for std::min(), std::max()...
#include <stdio.h> // for printf(), etc

namespace vision_utils {

//#define debug

/*!
  A solver for the Linear Assignment Problem
  ( en.wikipedia.org/wiki/Assignment_problem )

  It uses brute force to find the best assignment
  (it tries all possible assignments).

 \param costs
    the cost matrix for each pair
 \param best_assign
    (out) the resulting best AssignmentList
 \param best_cost
    (out) the cost of best_assign
 \return bool
    true if the assignment was succesful
*/
bool brute_force_assign(CMatrix<Cost> & costs,
                        MatchList & best_assign,
                        Cost & best_cost) {
  //printf("brute_force_assign()\n");
  unsigned v1s = costs.get_dim1(), v2s = costs.get_dim2();
  if (v1s == 0 || v2s == 0) {
    best_assign.resize(std::max(v1s, v2s));
    for (unsigned int i = 0; i < std::max(v1s, v2s); ++i) {
      best_assign[i].first = (v1s == 0 ? UNASSIGNED : i);
      best_assign[i].second = (v2s == 0 ? UNASSIGNED : i);
    }
    return true;
  }

  if (v1s > v2s) {
    // transpose the costs matrix and solve the transposed problem
    CMatrix<Cost> costs_transposed;
    costs.transpose_to(costs_transposed);
    if (!brute_force_assign(costs_transposed, best_assign, best_cost))
      return false;
    reverse_assignment_list(best_assign);
    return true;
  } // end if (v1s > v2s)

  OrderedCombination
      curr_comb(v1s, v2s), best_comb = curr_comb;
  best_cost = std::numeric_limits<Cost>::max();
  combination_init(curr_comb, v1s, v2s);
  while(true) {
    // test combination
    Cost curr_cost = 0;
    for (unsigned int v1_idx = 0; v1_idx < v1s; ++v1_idx)
      curr_cost += costs[v1_idx][curr_comb[v1_idx]];
    if (best_cost > curr_cost) {
      best_cost = curr_cost;
      best_comb = curr_comb;
      //printf("new best_comb:'%s' (%g)", accessible_to_string(best_comb).c_str(), best_cost);
    }

    // increment combination
    bool next_comb_found = curr_comb.incr();
    if (next_comb_found == false)
      break;
  }

#if 0 // checks
  printf("costs:\n%s", costs.to_string().c_str());
#endif

  // convert combination to AssignmentList
  best_assign.clear();
  best_assign.reserve(v1s);
  for (unsigned int i = 0; i < v1s; ++i) {
    Match assign;
    assign.first = i;
    assign.second = best_comb[i];
    assign.cost = costs[assign.first][assign.second];
    best_assign.push_back(assign);
  }

  return true;
} // end brute_force_assign()

} // end namespace vision_utils

#endif // BRUTE_FORCE_ASSIGN_H
