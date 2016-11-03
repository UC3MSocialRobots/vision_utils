/*!
  \file        oriented_average_angle.h
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

#ifndef ORIENTED_AVERAGE_ANGLE_H
#define ORIENTED_AVERAGE_ANGLE_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 * \brief   returns the average angle of a bunch of points,
            taking care of the painful case around [+-pi]
 * \param   angles some angles in the interval [-pi,+pi] radians
 * \return  the average angle , in the interval [-pi,+pi] radians.
 */
static inline double oriented_average_angle(const std::vector<double> & angles) {
  double avg = 0, best_angle = 0, curr_angle, lowest_dist;

  // iterate on all the angles
  for(std::vector<double>::const_iterator angle_it = angles.begin();
      angle_it != angles.end() ; ++ angle_it) {
    lowest_dist = INFINITY;

    // examinate angle - 2 PI, angle, angle + 2 PI
    for (int mod = -1; mod <= 1; ++mod) {
      curr_angle = *angle_it + mod * 2 * M_PI;
      if (fabs(avg - curr_angle) < lowest_dist)
        best_angle = curr_angle;
    } // end loop mod

    // add the best
    avg += best_angle;
  } // end loop angle_it

  // average it
  avg = avg / angles.size();

  // get the avg back in [-PI, PI]
  if (avg > M_PI)
    return avg - 2 * M_PI;
  else if (avg < - M_PI)
    return avg + 2 * M_PI;
  else
    return avg;
}

} // end namespace vision_utils

#endif // ORIENTED_AVERAGE_ANGLE_H
