/*!
  \file        combinatorics_utils.h
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

Some useful functions for playing with combinatorics.
Cultural minute:
http://en.wikipedia.org/wiki/Combinatorics

 */

#ifndef COMBINATORICS_UTILS_H
#define COMBINATORICS_UTILS_H

#include <string>
#include <vector>
#include <sstream>
#include <set>
#include <algorithm> // for min()
#include <stdexcept> // for invalid_argument()
#include <math.h> // for log

#include "vision_utils/string_casts_stl.h"

namespace vision_utils {
//cut:permutation
typedef std::vector<unsigned int> Permutation;
typedef std::vector<unsigned int> UnorderedCombinationVec;
typedef std::set<int> UnorderedCombinationSet;

/*!
 Populates ans with [0 1 2 ... (end_index-1)]
 \param ans the vector to populate
 \param end_index the number of elements at the end in the vector
*/
static inline void create_listing(Permutation & ans, const int end_index) {
  ans.clear();
  ans.reserve(end_index);
  for (int var = 0; var < end_index; ++var)
    ans.push_back(var);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 Find the next permutation if they are ordered with the lexicographical ordering.
 Implementation of
 http://en.wikipedia.org/wiki/Permutation#Systematic_generation_of_even_permutations
 \example for [0;1;2;3], successive iterations will give:
[0;1;2;3] [0;1;3;2] [0;2;1;3] [0;2;3;1] [0;3;1;2] [0;3; 2;1] [1;0;2;3]
[1;0;3;2] [1;2;0;3] [1;2;3;0] [1;3;0;2] [1;3;2;0] [2;0;1;3] [2;0;3;1]
[2;1;0;3] [2;1;3;0] [2;3; 0;1] [2;3;1;0] [3;0;1;2] [3;0;2;1] [3;1;0;2]
[3;1;2;0] [3;2;0;1] [3;2;1;0]
 \param permut the permutation to increase
 \return bool true if it was a success
*/
static inline bool permutation_find_next_lexicographical_ordering
(Permutation & permut) {
  // 1. Find the largest index k such that a[k] < a[k + 1].
  // If no such index exists, the permutation is the last permutation.
  int k = permut.size() - 2;
  Permutation::const_iterator current_pos = permut.end() - 2;
  Permutation::const_iterator current_pos_next = permut.end() - 1;
  while (true) {
    if (*current_pos < *current_pos_next)
      break;
    // go backwards
    --current_pos;
    --current_pos_next;
    --k;
    if (k < 0) // it was the last permutation
      return false;
  }
  //printf("k:%i", k);

  // 2. Find the largest index l such that a[k] < a[l].
  // Since k + 1 is such an index, l is well defined and satisfies k < l.
  int l = permut.size() - 1;
  current_pos_next = permut.end() - 1;
  while (true) {
    if (*current_pos < *current_pos_next)
      break;
    // go backwards
    --current_pos_next;
    --l;
  }
  //printf("l:%i", l);

  // 3. Swap a[k] with a[l].
  std::swap(permut[k], permut[l]);

  // 4. Reverse the sequence from a[k + 1] up to and including the final element a[n].
  std::reverse(permut.begin() + (k + 1), permut.end());

  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Init a combination with the smallest possible.
 * \param comb
 *    The combination to be filled.
 * \param k
 *    The number of elements we will put in comb
 * \example k=5, returns [0;1;2;3;4]
 */
static inline void combination_init(Permutation & comb, unsigned int k, unsigned int) {
  create_listing(comb, k);
}

////////////////////////////////////////////////////////////////////////////////
//cut:unordered_combination
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

////////////////////////////////////////////////////////////////////////////////
//cut:ordered_combination
/*! a small class for ordered combinations.
 * Stores the occurences of each
 */
class OrderedCombination : public std::vector<unsigned int> {
public:
  /*! Constructor. Also inits the inherent combination. */
  OrderedCombination(unsigned int k, unsigned int n) : _k(k), _n(n), occurrences(n, 0) {
    // the first combination is (0, 1, 2, ... k)
    combination_init(*this, k, n);
    // update occurences
    for (unsigned int i = 0; i < k; ++i)
      occurrences[i] = 1;
  }

  /*! Increments the combination.
   * \return true if succesful incrementation
   * \example succesive iterations if k:3,n:5:
   * [0;1;2] [0;1;3] [0;1;4] [0;2;1] [0;2;3] [0;2;4] [0;3;1] [0;3;2] [0;3;4]
   * [0;4;1] [0;4;2] [0;4;3] [1;0;2] [1;0;3] [1;0;4] [1;2;0] [1;2;3] [1;2;4]
   * [1;3;0] [1;3;2] [1;3;4] [1;4;0] [1;4;2] [1;4;3] [2;0;1] [2;0;3] [2;0;4]
   * [2;1;0] [2;1;3] [2;1;4] [2;3;0] [2;3;1] [2;3;4] [2;4;0] [2;4;1] [2;4;3]
   * [3;0;1] [3;0;2] [3;0;4] [3;1;0] [3;1;2] [3;1;4] [3;2;0] [3;2;1] [3;2;4]
   * [3;4;0] [3;4;1] [3;4;2] [4;0;1] [4;0;2] [4;0;3] [4;1;0] [4;1;2] [4;1;3]
   * [4;2;0] [4;2;1] [4;2;3] [4;3;0] [4;3;1] [4;3;2]
   */
  inline bool incr() {
    /* increment the vector */
    int index_to_incr = _k - 1;
    while(index_to_incr >= 0) {
      unsigned int* combi_ptr = &((*this)[index_to_incr]);
      --occurrences[ *combi_ptr ]; // substract one to the occurence of the old value
      ++(*combi_ptr); // the new value is incremented (modulo n)
      if (*combi_ptr >= _n) {
        *combi_ptr = 0;
        ++occurrences[ *combi_ptr ]; // update new occurence
        --index_to_incr; // mark the previous position to be checked
      }
      else { // incrementation is over and succesful
        ++occurrences[ *combi_ptr ]; // update new occurence
        break;
      }
    }

    // we have reached the final combination
    if (index_to_incr < 0)
      return false;

    //  printf("occurrences:'%s'\n",
    //              accessible_to_string(occurrences).c_str());

    // check for combinations with a repeatition
    for (unsigned int i = 0; i < _n; ++i) {
      if (occurrences[i] > 1)
        return incr();
    } // end loop i

    return true;
  }

  unsigned int _k, _n;
  std::vector<int> occurrences;
}; // end class OrderedCombination

////////////////////////////////////////////////////////////////////////////////

/*!
 * Apply a combination on a container to produce a "shuffled" version of it.
 * \param in
 *    The container to be shuffled.
 * \param out
 *    The shuffled container.
 * \param comb
 *    The combination.
 */
template<class _Container, class _Iterable>
static inline void apply_combination_on_container(const _Container & in,
                                                  _Container & out,
                                                  const _Iterable & comb) {
  // add the new eleemnts
  out.clear();
  for(typename _Iterable::const_iterator it = comb.begin();
      it != comb.end() ; ++it)
    out.push_back( in.at( *it ) );
}

////////////////////////////////////////////////////////////////////////////////
//cut:select_random_sample
/*!
 * Keep a given number of random elements of a container.
 * Elements are not repeated (use of a \a UnorderedCombinationSet).
 * \param in
 *  The "big" container.
 * \param out
 *  The created sample, of size \a size.
 * \param size
 *  The wanted size for \a out. Must be <= size of \a in.
 */
template<class _Container>
static inline void select_random_sample(const _Container & in,
                                        _Container & out,
                                        int size) {
  if (size >(int)  in.size())
    throw std::invalid_argument("Impossible to select a sample of %i elements "
                                "> sample size = %i",
                                size, in.size());
  // generate a random combination
  UnorderedCombinationSet comb;
  combination_set_random(comb, size, in.size());
  //printf("comb.size():%u", comb.size());
  // add the new eleemnts
  apply_combination_on_container<_Container, UnorderedCombinationSet>
      (in, out, comb);
}
//cut:rand_gaussian
/*!
 *\brief   returns a random number with a gaussian law
 * (standard normal distribution).
 * Its probability density function is
 * phi(x) = 1 / sqrt(2 * PI) * exp(-x^2 / 2)
 */
inline double rand_gaussian() {
  //  double x1 = 1.f * rand() / RAND_MAX;
  //  double x2 = 1.f * rand() / RAND_MAX;
  //  return sqrt(-2 * log(x1)) * cos(2.f * M_PI * x2);

  // Box–Muller method
  return sqrt(-2 * log(drand48())) * cos(2.f * M_PI * drand48());

  // Polar form of Box–Muller method
  //  double u = -1 +  2 * drand48(), v = -1 + 2 * drand48();
  //  double s = u * u + v * v;
  //  return u * sqrt((-2 * log(s)) / s);

  // from http://c-faq.com/lib/gaussian.html
  //  static double U, V;
  //  static int phase = 0;
  //  double Z;
  //  if(phase == 0) {
  //    U = (rand() + 1.) / (RAND_MAX + 2.);
  //    V = rand() / (RAND_MAX + 1.);
  //    Z = sqrt(-2 * log(U)) * sin(2 * M_PI * V);
  //  } else
  //    Z = sqrt(-2 * log(U)) * cos(2 * M_PI * V);
  //  phase = 1 - phase;
  //  return Z;
} // end rand_gaussian()

//cut:add_gaussian_noise
template<class Pt3>
inline void add_gaussian_noise(Pt3 & p,
                               const double & pos_error_std_dev) {
  p.x +=  rand_gaussian() * pos_error_std_dev;
  p.y +=  rand_gaussian() * pos_error_std_dev;
  p.z +=  rand_gaussian() * pos_error_std_dev;
}

}; // end vision_utils

#endif // COMBINATORICS_UTILS_H
