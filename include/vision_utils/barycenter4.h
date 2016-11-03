/*!
  \file        barycenter4.h
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

#ifndef BARYCENTER4_H
#define BARYCENTER4_H

namespace vision_utils {

/*!
 * \brief   returns the point as follows
     *
 *       <--tx-->
 *      A ___________________ B
 *  ^   |                     |
 *  |   |                     |
 *  ty  |                     |
 *  |   |        P            |
 *  v   |        +            |
 *      |                     |
 *      C ___________________ D
 */
template<class Point2i, class Point2f>
static inline void barycenter4(const float& tx, const float& ty,
                               const Point2i & A, const Point2i & B,
                               const Point2i & C, const Point2i & D,
                               Point2f & rep) {
  float P1x = A.x + ty * (D.x - A.x);
  float P2x = B.x + ty * (C.x - B.x);
  rep.x = P1x + tx * (P2x - P1x);

  float P1y = A.y + ty * (D.y - A.y);
  float P2y = B.y + ty * (C.y - B.y);
  rep.y = P1y + tx * (P2y - P1y);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   equation of the line between two points.
 * Cf http://fr.wikipedia.org/wiki/%C3%89quation_de_droite
 */
//template<class P> static inline Point3 line_equation(P A, P B);

} // end namespace vision_utils

#endif // BARYCENTER4_H
