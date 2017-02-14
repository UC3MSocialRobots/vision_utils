/*!
  \file        find_closest_points_brute_force.h
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

#ifndef FIND_CLOSEST_POINTS_BRUTE_FORCE_H
#define FIND_CLOSEST_POINTS_BRUTE_FORCE_H
// std includes
#include <stdexcept> // for invalid_argument
#include <vector>

namespace vision_utils {

/*!
 \fn find_closest_points_brute_force
 \param set1
 \param set2
 \param best_permutation a copy of the best permutation
 \return double the squared distance with the best matching
*/
template<class Point2>
static inline double find_closest_points_brute_force(
    const std::vector<Point2> & set1,
    const std::vector<Point2> & set2,
    Permutation & best_permutation) {

  // size check
  if (set1.size() != set2.size()) {
    throw std::invalid_argument("the two sets have a different size");// (%li != %li",
                                //(int) set1.size(), (int) set2.size());
  }

  // prepair the stuff
  double best_distance_squared = -1;
  Permutation current_permutation;
  create_listing(current_permutation, set1.size());
  typename std::vector<Point2>::const_iterator set1_it;
  Permutation::const_iterator perm_it;
  double current_distance_squared;

  bool next_was_found = true;
  while (next_was_found) {
    //printf("current_permutation:'%s'",
    //accessible_to_string(current_permutation).c_str());
    // do the stuff
    set1_it = set1.begin();
    perm_it = current_permutation.begin();
    current_distance_squared = 0;
    for (uint idx = 0; idx < set1.size(); ++idx) {
      const Point2* set2_pt = &set2[ *perm_it ];
      current_distance_squared += pow(set1_it->x - set2_pt->x, 2)
                                  + pow(set1_it->y - set2_pt->y, 2);
      // stop if the best distance is already reached
      if (current_distance_squared > best_distance_squared) {
        //printf("Breaking...");
        break;
      }
      // advance iterators
      ++set1_it;
      ++perm_it;
    } // end loop set1_it

    if (best_distance_squared == -1
        || current_distance_squared < best_distance_squared) {
      // keep the data
      best_distance_squared = current_distance_squared;
      best_permutation = current_permutation;

    } // end new best_distance_squared

    next_was_found =
                     permutation_find_next_lexicographical_ordering(current_permutation);
  } // end while next_was_found

  return best_distance_squared;
}

} // end namespace vision_utils

#endif // FIND_CLOSEST_POINTS_BRUTE_FORCE_H
