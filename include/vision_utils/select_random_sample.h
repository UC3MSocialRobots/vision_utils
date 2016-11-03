/*!
  \file        select_random_sample.h
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

#ifndef SELECT_RANDOM_SAMPLE_H
#define SELECT_RANDOM_SAMPLE_H
// std includes
#include <stdexcept> // for invalid_argument

namespace vision_utils {

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

} // end namespace vision_utils

#endif // SELECT_RANDOM_SAMPLE_H
