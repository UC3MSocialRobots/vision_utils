/*!
  \file        assignment_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/16

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

\todo Description of the file

 */

#ifndef ASSIGNEMENT_UTILS_H
#define ASSIGNEMENT_UTILS_H

#include "vision_utils/utils/combinatorics_utils.h"
#include "vision_utils/utils/cmatrix.h"
#include "vision_utils/utils/geometry_utils.h"

#include "vision_utils/utils/lap.h"

namespace assignment_utils {

typedef double Cost;

struct Match {
  int first;
  int second;
  Cost cost;
};
inline bool operator == (const Match& A, const Match & B) {
  return (A.first == B.first) && (A.second == B.second) && (A.cost == B.cost);
}
inline bool operator < (const Match& A, const Match& B) {
  return A.first < B.first;
}

typedef std::vector<Match> MatchList;
static const int UNASSIGNED = -1;

////////////////////////////////////////////////////////////////////////////////

/*!
  Inverts a list of assignements, by inverting each pair.
 \param list
 \example (1, 2), (2, 4)  will become (2, 1), (4, 2)
*/
void reverse_assignment_list(MatchList & list) {
  for (unsigned int pair_idx = 0; pair_idx < list.size(); ++pair_idx)
    std::swap(list[pair_idx].first, list[pair_idx].second);
} // end reverse_assignment_list()

void sort_assignment_list(MatchList & list) {
  std::sort(list.begin(), list.end());
} // end reverse_assignment_list()

Cost assignment_list_cost(const MatchList & list) {
  Cost total_cost = 0;
  unsigned int npairs = list.size();
  for (unsigned int pair_idx = 0; pair_idx < npairs; ++pair_idx)
    total_cost += list[pair_idx].cost;
  return total_cost;
}

////////////////////////////////////////////////////////////////////////////////

//! convert an AssignmentList to a string, for displaying and debugging
inline std::string assignment_list_to_string(const MatchList & list) {
  std::ostringstream out;
  unsigned int npairs = list.size();
  out << "AssignmentList:" << npairs
      << " pairs, total cost:" << assignment_list_cost(list) << ": ";
  for (unsigned int pair_idx = 0; pair_idx < npairs; ++pair_idx)
    out << list[pair_idx].first << "->" << list[pair_idx].second
        << " (" << std::setprecision(3) << list[pair_idx].cost << "); ";
  return out.str();
} // end assignment_list_to_string()

////////////////////////////////////////////////////////////////////////////////

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
  // printf("linear_assign()\n");
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
  // printf("linear_assign_from_cost_vec()\n");
  CMatrix<double> costs_mat;
  if (!costs_mat.from_vector(costs, dim1, dim2))
    return false;
  if (!assignment_utils::linear_assign(costs_mat, best_assign, best_cost))
    return false;
  return true;
} // end linear_assign_from_cost_vec()

////////////////////////////////////////////////////////////////////////////////

//#define debug
#include "vision_utils/utils/debug2.h"

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
  // printf("brute_force_assign()\n");
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

  combinatorics_utils::OrderedCombination
      curr_comb(v1s, v2s), best_comb = curr_comb;
  best_cost = std::numeric_limits<Cost>::max();
  combinatorics_utils::combination_init(curr_comb, v1s, v2s);
  while(true) {
    // test combination
    Cost curr_cost = 0;
    for (unsigned int v1_idx = 0; v1_idx < v1s; ++v1_idx)
      curr_cost += costs[v1_idx][curr_comb[v1_idx]];
    if (best_cost > curr_cost) {
      best_cost = curr_cost;
      best_comb = curr_comb;
      debugPrintf("new best_comb:'%s' (%g)",
                  string_utils::accessible_to_string(best_comb).c_str(), best_cost);
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

////////////////////////////////////////////////////////////////////////////////

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
                      = &(geometry_utils::distance_points_squared),
                      bool (*assign_func)
                      (CMatrix<Cost> &, MatchList &, Cost &)
                      = &linear_assign
                        ) {
  // printf("assign_and_dists()\n");
  costs_buffer.resize(v1.size(), v2.size());
  for (unsigned int v1_idx = 0; v1_idx < v1.size(); ++v1_idx)
    for (unsigned int v2_idx = 0; v2_idx < v2.size(); ++v2_idx)
      costs_buffer[v1_idx][v2_idx] = cost_func(v1[v1_idx], v2[v2_idx]);
  debugStream("costs_buffer:" << costs_buffer.to_string());

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
    debugStream("A:" << string_utils::iterable_to_string(A)
                << "\nB:" << string_utils::iterable_to_string(B) << std::endl);
    assignment_utils::assign_and_dists<Pt2, Pt2>
        (A, B, _costs_buffer, _assign, _dist_AB,
         //&geometry_utils::distance_points_squared
         &hausdorff_distances::dist_Linf_double<Pt2>
         );
    return _dist_AB;
  }

  CMatrix<assignment_utils::Cost> _costs_buffer;
  assignment_utils::MatchList _assign;
  assignment_utils::Cost _dist_AB;
};

////////////////////////////////////////////////////////////////////////////////

//! remove the unassigned matches in a MatchList
inline void clean_assign(MatchList & assign) {
  for (unsigned int i = 0; i < assign.size(); ++i) {
    if (assign[i].first != assignment_utils::UNASSIGNED
        && assign[i].second != assignment_utils::UNASSIGNED)
      continue;
    assign.erase(assign.begin() + i);
    --i;
  } // end for i
} // end clean_assign();

////////////////////////////////////////////////////////////////////////////////

} // end namespace assignment_utils

#endif // ASSIGNEMENT_UTILS_H
