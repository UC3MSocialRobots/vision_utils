#ifndef HAUSDORFF_DISTANCES_H
#define HAUSDORFF_DISTANCES_H

/***************************************************************************//**
* \class hausdorff_distances
*
* \brief some functions for measuring distances between sets of points
*
* cf "A Modified Hausdorff Distance for Object Matching"
* by Marie-Pierre Dubuisson and Ani1 K. Jain
*
* \author Arnaud Ramey ( arnaud.ramey@m4x.org )
*
* \date April 2009
*******************************************************************************/

///// STL imports
#include <vector>           // for vectors
#include <limits>
#include <stdlib.h>         // for abs(int)
#include <math.h>

namespace vision_utils {

/*!
 * \brief   returns |ab|, L1 norm
 */
template<class _Pt2>
inline double dist_L1_double(const _Pt2 & a, const _Pt2 & b) {
  return fabs(a.x - b.x) + fabs(a.y - b.y);
}

template<class _Pt2>
inline double dist_L1_int(const _Pt2 & a, const _Pt2 & b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

/*!
 * \brief   returns |ab|, L2 norm
 */
template<class _Pt2>
inline double dist_L2(const _Pt2 & a, const _Pt2 & b) {
  return hypot(a.x - b.x, a.y - b.y);
}

/*!
 * \brief   returns |ab|, Linf norm
 */
template<class _Pt2>
inline double dist_Linf_double(const _Pt2 & a, const _Pt2 & b) {
  return std::max(fabs(a.x - b.x), fabs(a.y - b.y));
}

template<class _Pt2>
inline double dist_Linf_int(const _Pt2 & a, const _Pt2 & b) {
  return std::max(abs(a.x - b.x), abs(a.y - b.y));
}

/*!
 * \brief   returns min_{b in B} dist_func(a,b)
 * By default, use L2 distance
 */
template<class _Pt2, class Pt2Iterable>
inline double dist_pt_set(const _Pt2 & a, const Pt2Iterable & B,
                          double (*dist_func_ptr)(const _Pt2 &, const _Pt2 &)
                          = &dist_L2) {
  double min= std::numeric_limits<double>::max();
  unsigned int nB = B.size();
  for (unsigned int B_idx = 0; B_idx < nB; ++B_idx) {
    double curr = (*dist_func_ptr)(a, B[B_idx]);
    if (min > curr)
      min = curr;
  } // end loop B_idx
  return min;
}

/*!
 * \brief   the hausdorf d22 distance, modified to accept a max value
 *
 * \param   A a vector of _Pt2
 * \param   B a vector of _Pt2
 * \param   min the min distance for which we will return INFINITY
 * \return  max(d6 (A, B), d6 (B, A) )
            = max( 1 / |A| * sum_{a in A} d(a, B) ,
                   1 / |B| * sum_{b in B} d(b, A))
                                                    if < min,
            INFINITY                                else

            It is between 0      (perfect match)
            and d6 (a0, b0)      (completely different)
                                 where a0 and b0 are the most remote points
                                 of A and B.
 */
template<class _Pt2, class Pt2Iterable>
inline double
D22_with_min(const Pt2Iterable & A, const Pt2Iterable & B,
             const double min,
             double (*dist_func_ptr)(const _Pt2 &, const _Pt2 &)
             = &dist_L1_int) {

  unsigned int nA = A.size(), nB = B.size();
  double min_A = min * nA, sum_A = 0;
  for (unsigned int idx_A = 0; idx_A < nA; ++idx_A) {
    sum_A += dist_pt_set( A[idx_A], B, dist_func_ptr);
    if (sum_A > min_A)
      return std::numeric_limits<typename _Pt2::value_type>::max();
  } // end loop A
  sum_A = sum_A / nA;

  double min_B = min * nB, sum_B = 0;
  for (unsigned int idx_B = 0; idx_B < nB; ++idx_B) {
    sum_B += dist_pt_set( B[idx_B], A, dist_func_ptr);
    if (sum_B > min_B)
      return std::numeric_limits<typename _Pt2::value_type>::max();
  } // end loop B
  sum_B = sum_B / nB;

  return std::max(sum_A, sum_B);
}

} // end namespace vision_utils

#endif

