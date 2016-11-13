/*!
  \file        add_gaussian_noise.h
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

#ifndef ADD_GAUSSIAN_NOISE_H
#define ADD_GAUSSIAN_NOISE_H
#include <vision_utils/rand_gaussian.h>

namespace vision_utils {

template<class Pt3>
inline void add_gaussian_noise(Pt3 & p,
                               const double & pos_error_std_dev) {
  p.x +=  rand_gaussian() * pos_error_std_dev;
  p.y +=  rand_gaussian() * pos_error_std_dev;
  p.z +=  rand_gaussian() * pos_error_std_dev;
}

} // end namespace vision_utils

#endif // ADD_GAUSSIAN_NOISE_H
