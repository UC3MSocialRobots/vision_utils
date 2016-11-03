/*!
  \file        linear_assign.h
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

#ifndef LINEAR_ASSIGN_H
#define LINEAR_ASSIGN_H
// std includes
#include <algorithm> // for std::min(), std::max()...
#include <stdio.h> // for printf(), etc
#include <vector>
#include "vision_utils/reverse_assignment_list.h"
#include "vision_utils/lap.h"


namespace vision_utils {

/*!
  A solver for the Linear Assignment Problem
  ( en.wikipedia.org/wiki/Assignment_problem )
  It is a wrapper for the R. Jonker and A. Volgenant algorithm,
  presented in:
   "A Shortest Augmenting Path Algorithm for Dense and Sparse Linear
    Assignment Problems," Computing 38, 325-340, 1987

 \param costs
    the cost matrix for each pair
 \param best_assign
    (out) the resulting best AssignmentList
 \param best_cost
    (out) the cost of best_assign
 \return bool
    true if the linear assignment was succesful
*/
bool linear_assign(CMatrix<Cost> & costs,
                   MatchList & best_assign,
                   Cost & best_cost) {
  //printf("linear_assign()\n");
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
    // generate a square matrix with costs of 0 for new elements
    // inspired by http://en.wikipedia.org/wiki/Assignment_problem#Example
    CMatrix<Cost> costs_squared(v1s, v1s);
    costs_squared.set_to_zero();
    for (unsigned int i1 = 0; i1 < v1s; ++i1)
      for (unsigned int i2 = 0; i2 < v2s; ++i2)
        costs_squared[i1][i2] = costs[i1][i2];
    // try linear assignement with squared costs matrix
    if (!linear_assign(costs_squared, best_assign, best_cost))
      return false;
    // clean assignements to newly created elements
    int v2s_int = v2s; // conversion unsigned int -> int
    for (unsigned int assign_idx = 0; assign_idx < best_assign.size(); ++assign_idx) {
      //  printf("assign_idx:%i (%i, %i)\n",
      //         assign_idx, best_assign[assign_idx].first, best_assign[assign_idx].second);
      if (best_assign[assign_idx].second >= v2s_int)
        best_assign[assign_idx].second = UNASSIGNED;
    } // end loop assign_idx
    return true;
  } // end if v1s > v2s

  if (v1s < v2s) {
    // transpose the costs matrix and solve the transposed problem
    CMatrix<Cost> costs_transposed;
    costs.transpose_to(costs_transposed);
    if (!linear_assign(costs_transposed, best_assign, best_cost))
      return false;
    reverse_assignment_list(best_assign);
    return true;
  } // end if (v1s < v2s)

  // call C lap
  int rowsol[v1s], colsol[v1s];
  double u[v1s], v[v1s];
  best_cost = lap(v1s, costs.ptr_to_data(), rowsol, colsol, u, v);

#if 0 // checks
  printf("costs:\n%s\n", costs.to_string().c_str());
  checklap(v1s, costs.ptr_to_data(), rowsol, colsol, u, v);
  double sum = 0;
  for (int i = 0; i < v1s; ++i) {
    printf("i:%i -> rowsol[i]:%i\n", i, rowsol[i]);
    sum += costs[i][rowsol[i]];
  }
  printf("sum:%g\n", sum);
#endif

  // build AssignmentList
  best_assign.clear();
  best_assign.reserve(v1s);
  for (unsigned int i = 0; i < v1s; ++i) {
    Match assign;
    assign.first = i;
    assign.second = rowsol[i];
    assign.cost = costs[assign.first][assign.second];
    best_assign.push_back(assign);
  }
  return true;
} // end linear_assign()

////////////////////////////////////////////////////////////////////////////////

//! Wrapper for linear_assign() using a vector
bool linear_assign_from_cost_vec(const std::vector<Cost> & costs,
                                 unsigned int dim1, unsigned int dim2,
                                 MatchList & best_assign,
                                 Cost & best_cost) {
  //printf("linear_assign_from_cost_vec()\n");
  CMatrix<double> costs_mat;
  if (!costs_mat.from_vector(costs, dim1, dim2))
    return false;
  if (!linear_assign(costs_mat, best_assign, best_cost))
    return false;
  return true;
} // end linear_assign_from_cost_vec()

} // end namespace vision_utils

#endif // LINEAR_ASSIGN_H
