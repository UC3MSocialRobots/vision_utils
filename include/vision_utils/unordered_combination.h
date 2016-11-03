/*!
  \file        unordered_combination.h
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

#ifndef UNORDERED_COMBINATION_H
#define UNORDERED_COMBINATION_H
// std includes
#include <sstream> // for ostringstream
#include <stdexcept> // for invalid_argument
#include <string>

namespace vision_utils {

/*!
 * \param comb
 *    a combination
 * \return
 *    a string representation of the combination
 */
static inline std::string combination_iterable_to_string(const UnorderedCombinationSet & comb) {
  if (comb.size() == 0)
    return "{}";
  std::ostringstream ans_stream;
  ans_stream << "[";
  for (UnorderedCombinationSet::const_iterator i = comb.begin();
       i != comb.end(); ++ i)
    ans_stream << *i << ";";
  ans_stream << "]";
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Generate a random combination (but a legit one, so that no element is repeated).
 * \param comb
 *    The combination to be filled.
 * \param k
 *    The number of elements we will put in comb
 * \param n
 *    The total number of elements, must be >= k
 * \example k=8, n=10, a few samples:
 * [0;1;2;3;5;6;7;8] [0;1;2;3;4;7;8;9] [1;2;3;4;6;7;8;9] [0;1;2;5;6;7;8;9]
 * [0;1;3;4;5;6;7;9] [1;2;3;4;5;6;7;8] [0;1;2;4;6;7;8;9] [1;2;3;4;5;6;8;9]
 * [0;2;3;4;5;7;8;9] [0;1;2;4;5;7;8;9]
 */
static inline void combination_set_random(UnorderedCombinationSet & comb,
                                          unsigned int k, unsigned int n) {
  if (k > n) {
    char error_msg[1000];
    sprintf(error_msg, "Impossible to generate a combination with k=%i > n=%i",
            k, n);
    throw std::invalid_argument(error_msg);
  }

  comb.clear();
#if 0
  //std::set<int> comb;

  for (unsigned int new_value_idx = 0; new_value_idx < k; ++new_value_idx) {
    // find a value that was not used before
    int new_value;
    while (true) {
      new_value = rand() % n;
      if (comb.find(new_value) == comb.end())
        break;
    }

    // add this new value
    comb.insert(new_value);
    //comb.push_back(new_value);
  } // end loop new_value_idx
#else
  while (comb.size() < k)
    comb.insert(rand() % n);
#endif
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Generate a random combination (but a legit one, so that no element is repeated).
 * \param comb
 *    The combination to be filled.
 * \param k
 *    The number of elements we will put in comb
 * \param n
 *    The total number of elements, must be >= k
 * \example k=8, n=10, a few samples:
 * [0;1;2;3;5;6;7;8] [0;1;2;3;4;7;8;9] [1;2;3;4;6;7;8;9] [0;1;2;5;6;7;8;9]
 * [0;1;3;4;5;6;7;9] [1;2;3;4;5;6;7;8] [0;1;2;4;6;7;8;9] [1;2;3;4;5;6;8;9]
 * [0;2;3;4;5;7;8;9] [0;1;2;4;5;7;8;9]
 */
static inline void combination_vec_random(UnorderedCombinationVec & comb,
                                          unsigned int k, unsigned int n,
                                          UnorderedCombinationSet comb_set_buffer) {
  combination_set_random(comb_set_buffer, k, n);
  comb.clear();
  comb.reserve(k);
  std::copy(comb_set_buffer.begin(), comb_set_buffer.end(), std::back_inserter(comb));
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Find the next unordered combination.
 * Repeated elements are not allowed.
 * And [0;1] is equal to [1;0].
 * \param comb
 *    The combination to be filled.
 * \param k
 *    The number of elements we will put in comb
 * \param n
 *    The total number of elements, must be >= k
 * \return
 *    true if success
 * \example Successive iterations if k:3, n:5
 * [0;1;2] [0;1;3] [0;1;4] [0;2;3] [0;2;4] [0;3;4] [1;2;3] [1;2;4] [1;3;4]
 */
static inline bool combination_incr(UnorderedCombinationVec & comb, unsigned int k, unsigned int n) {
  //printf("combination_incr()");
  int i = k - 1;
  ++comb[i];
  while ((i >= 0) && (comb[i] >= n - k + 1 + i)) {
    --i;
    ++comb[i];
  }

  if (comb[0] >= n - k) /* Combination (n-k, n-k+1, ..., n) reached */
    return 0; /* No more combinations can be generated */

  /* comb now looks like (..., x, n, n, n, ..., n).
        Turn it into (..., x, x + 1, x + 2, ...) */
  for (i = i + 1; i < (int) k; ++i)
    comb[i] = comb[i - 1] + 1;

  return 1;
}

} // end namespace vision_utils

#endif // UNORDERED_COMBINATION_H
