/*!
  \file        oriented_average_angle_in_vector.h
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

#ifndef ORIENTED_AVERAGE_ANGLE_IN_VECTOR_H
#define ORIENTED_AVERAGE_ANGLE_IN_VECTOR_H
// std includes
#include <algorithm> // for std::min(), std::max()...
#include <vector>

namespace vision_utils {

/*!
 Computes the average angle of a point with its neighbours
 \param pts
 \param pt_pos you should not ask for the left angle of the first pt or
        right angle of the last one
 \param to_left
 \param max_window_size the nb of neighbours to take into account.
        For the points at the very beginning or end of the vector,
        less points will be used.
 \return the angle between -PI and PI
*/
template<class Point2>
static inline double oriented_average_angle_in_vector(const std::vector<Point2> & pts,
                                                      const unsigned int pt_pos,
                                                      const unsigned int max_window_size,
                                                      bool to_left = true) {
  // take care of the extreme points
  if ((!to_left && pt_pos >= pts.size()-1) ||
      (to_left && pt_pos <= 0)) {
    //printf("size:%i, pt_pos:%i and going to %s: extreme point! Average angle with its neighbours impossible.", pts.size(), pt_pos, (to_left ? "left" : "right"));
    return 0;
  }

  // determine the size of the window (existing neighbours)
  int window_size = std::min(max_window_size,
                             (to_left ?
                                pt_pos :
                                (unsigned int) (pts.size() - 1 - pt_pos)));
  //printf("window_size:%i", window_size);

  // collect all the angles in the surroundings
  typename std::vector<Point2>::const_iterator
      curr_it = pts.begin() + pt_pos,
      neigh_it = curr_it + (to_left ? -1 : 1);
  std::vector<double> angles;
  for (int neigh_count = 0; neigh_count < window_size; ++neigh_count) {
    angles.push_back(oriented_angle_of_a_vector( *neigh_it - *curr_it ));
    // move iterator
    neigh_it += (to_left ? -1 : 1);
  } // end loop neigh_count

  // average them
  return oriented_average_angle(angles);
}

} // end namespace vision_utils

#endif // ORIENTED_AVERAGE_ANGLE_IN_VECTOR_H
