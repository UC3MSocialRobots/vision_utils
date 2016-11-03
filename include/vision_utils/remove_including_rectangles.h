/*!
  \file        remove_including_rectangles.h
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

#ifndef REMOVE_INCLUDING_RECTANGLES_H
#define REMOVE_INCLUDING_RECTANGLES_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 * Remove the rectangles that include other rectangles in their area,
 * or the contrary.
 * \param recs_in
 *  The rectangles to be filtered.
 * \param recs_out
 *  [OUT] the filtered rectangles. None of them contains another one.
 * \param remove_including
 *  If true, remove the rectangles that include other rectangles in their area
 *            (keep only the smallest ones).
 *  If false, remove the rectangles that are included into another
 *            (keep only the biggest ones).
 */
template<class Rect>
inline void remove_including_rectangles(const std::vector<Rect> & recs_in,
                                        std::vector<Rect> & recs_out,
                                        bool remove_including = true) {
  //printf("remove_including_rectangles(%i recs)\n", recs_in.size());
  recs_out.clear();
  recs_out.reserve(recs_in.size());
  const Rect* r1 = &(recs_in[0]);
  for( unsigned int r1_idx = 0; r1_idx < recs_in.size(); r1_idx++ ) {
    bool r1_ok = true;
    const Rect* r2 = &(recs_in[0]);
    for( unsigned int r2_idx = 0; r2_idx < recs_in.size(); r2_idx++ ) {
      if (r1_idx == r2_idx) { // jump to next rectangle
        ++r2;
        continue;
      }
      //printf("r1_idx:%i, r2_idx:%i, r1:%s, r2:%s, inter:%s, equal:%i\n",
      //       r1_idx, r2_idx, print_rect(*r1).c_str(), print_rect(*r2).c_str(),
      //       print_rect(rectangle_intersection(*r1, *r2)).c_str(),
      //       rectangle_intersection(*r1, *r2) == (remove_including ? *r1 : *r2));

      // rectangles equal -> keep the one with the lowest index
      if(*r1 == *r2) {
        if (r1_idx > r2_idx) {
          //printf("Case 1\n");
          r1_ok = false;
          break;
        }
        else { // jump to next rectangle
          ++r2;
          continue;
        }
      } // end if(*r1 == *r2)

      // if r2 included in r1, break
      if(rectangle_intersection(*r1, *r2) == (remove_including ? *r1 : *r2)) {
        //printf("Case 2\n");
        r1_ok = false;
        break;
      }
      ++r2; // increment pointer
    } // end for r2_idx
    if( r1_ok ) // no rectangle is included in r1
      recs_out.push_back(*r1);
    ++r1; // increment pointer
  } // end for r1_idx
} // end remove_including_rectangles()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Remove the rectangles included in other rectangles in their area.
 * or the contrary.
 * \param recs_in
 *  The rectangles to be filtered.
 * \param recs_out
 *  [OUT] the filtered rectangles. None of them contains another one.
 */
template<class Rect>
inline void remove_included_rectangles(const std::vector<Rect> & recs_in,
                                       std::vector<Rect> & recs_out) {
  remove_including_rectangles(recs_in, recs_out, false);
}

} // end namespace vision_utils

#endif // REMOVE_INCLUDING_RECTANGLES_H
