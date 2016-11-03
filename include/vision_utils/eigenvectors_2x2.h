/*!
  \file        eigenvectors_2x2.h
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

#ifndef EIGENVECTORS_2X2_H
#define EIGENVECTORS_2X2_H

namespace vision_utils {

/*!  Computes eigenvectors for a 2x2 matrix,
 * ( a b )
 * ( c d)
 *\see http://www.math.harvard.edu/archive/21b_fall_04/exhibits/2dmatrices/index.html
   http://en.wikipedia.org/wiki/Eigenvalue_algorithm#2.C3.972_matrices */
template<class Vec2>
bool eigenvectors_2x2(const double & a, const double & b,
                      const double & c, const double & d,
                      double & eigen1, double & eigen2,
                      Vec2 & V1, Vec2 & V2) {
  double T = a + d,
      D = a * d - b *c,
      t = .25 * T * T - D;
  if (t < 0)
    return false;
  eigen1 = .5 * T + sqrt(t);
  eigen2 = -eigen1 + T;
  V1 = Vec2(a - eigen2, c);
  if (fabs(V1.x) < 1E-2 && fabs(V1.y) < 1E-2) // avoid null vectors
    V1 = Vec2(b, d - eigen2);
  V2 = Vec2(a - eigen1, c);
  if (fabs(V2.x) < 1E-2 && fabs(V2.y) < 1E-2) // avoid null vectors
    V2 = Vec2(b, d - eigen1);
  return true;
}

} // end namespace vision_utils

#endif // EIGENVECTORS_2X2_H
