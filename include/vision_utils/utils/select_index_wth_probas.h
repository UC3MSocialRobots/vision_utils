#ifndef SELECT_INDEX_WTH_PROBAS_H
#define SELECT_INDEX_WTH_PROBAS_H

#include <vector>
#include <functional>
#include <numeric>
#include <stdlib.h>

namespace select_index_with_probas {

//! an index
typedef unsigned int Index;

////////////////////////////////////////////////////////////////////////////////

//! return a random number between 0 and 1
inline double rand_0_1() {
  //return ( (Proba) rand() / (Proba) RAND_MAX);
  return drand48();
}

////////////////////////////////////////////////////////////////////////////////

/*! a version where the sum of the probas between the iterators
    is already known */
template <typename InputIterator>
inline Index select_given_sum(InputIterator first, InputIterator last, double sum) {
  if (first == last) // empty vector
    return 0;

  double selector = rand_0_1() * sum, curr_sum = 0;
  //ROS_WARN("selector:%g, sum:%g", selector, sum);
  Index ans_index = 0;
  while ( true ) {
    curr_sum = curr_sum + *first++;
    if (selector <= curr_sum)
      break;
    if (first == last) // we reached the last item
      break;
    ++ans_index;
  } // end while ( first!=last )
  return ans_index;
} // end select_given_sum()

////////////////////////////////////////////////////////////////////////////////

/*!
 Select an index within a vector whose values are the probabilities
 \param first
 \param last
 \return Index
    the selected index
*/
template <typename InputIterator>
inline Index select(InputIterator first, InputIterator last) {
  return select_given_sum<InputIterator>
      (first, last, std::accumulate(first, last, (double) 0));
} // end select_index_wth_probas()


} // end namespace select_index_with_probas

#endif // SELECT_INDEX_WTH_PROBAS_H
