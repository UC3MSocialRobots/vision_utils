/*!
  \file        barycenter.h
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

#ifndef BARYCENTER_H
#define BARYCENTER_H
// std includes
#include <vector>

namespace vision_utils {

/*!
 * \brief   computes the barycenter of a std::vector of points
     *
 * \param   src the std::vector
 * \return  the barycenter
 */
template<class Point2>
static inline Point2 barycenter(const std::vector<Point2> & src) {
  if (src.empty())
    return Point2();
  double x = 0, y = 0;
  unsigned int npts = src.size();
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    x += src[pt_idx].x;
    y += src[pt_idx].y;
  } // end loop pt_idx
  return Point2(x / npts, y / npts);
}

////////////////////////////////////////////////////////////////////////////////

///*! a templated version of the barycenter function */
//template<class Point2>
//static inline void barycenter(const float& t,
//                              const Point2 & begin, const Point2 & end,
//                              Point2 & rep) {
//    rep.x = t * begin.x + (1.0 - t) * end.x;
//    rep.y = t * begin.y + (1.0 - t) * end.y;
//}

} // end namespace vision_utils

#endif // BARYCENTER_H
