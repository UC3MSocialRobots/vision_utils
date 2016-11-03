/*!
  \file        rand_gaussian.h
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

#ifndef RAND_GAUSSIAN_H
#define RAND_GAUSSIAN_H

#include <stdlib.h> // for drand48
#include <math.h> // for cos

namespace vision_utils {

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

} // end namespace vision_utils

#endif // RAND_GAUSSIAN_H
