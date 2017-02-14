/*!
  \file        permutation.h
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

#ifndef PERMUTATION_H
#define PERMUTATION_H
// std includes
#include <vector>
#include <set>

namespace vision_utils {

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

} // end namespace vision_utils

#endif // PERMUTATION_H
