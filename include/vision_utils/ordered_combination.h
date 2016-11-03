/*!
  \file        ordered_combination.h
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

#ifndef ORDERED_COMBINATION_H
#define ORDERED_COMBINATION_H
// std includes
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

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

} // end namespace vision_utils

#endif // ORDERED_COMBINATION_H
