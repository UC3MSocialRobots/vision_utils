/*!
  \file        boundingBox_vec.h
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

#ifndef BOUNDINGBOX_VEC_H
#define BOUNDINGBOX_VEC_H

namespace vision_utils {

/*!
 * \brief   get the bounding box of the points in a vector
 * \param   pts a vector of 2D points
 */
template<class Pt2Iterable, class _Bbox>
static _Bbox boundingBox_vec(const Pt2Iterable & pts) {
  //printf("boundingBox_vec()");
  if (pts.size() == 0)
    return _Bbox(-1,-1,-1, -1);
  typedef typename Pt2Iterable::value_type _Point2;
  typename _Point2::value_type
      xMin = pts.front().x, yMin = pts.front().y,
      xMax = xMin, yMax = yMin;

  for (unsigned int pt_idx = 0; pt_idx < pts.size(); ++pt_idx) {
    const _Point2* pt = &pts[pt_idx];
    if (pt->x < xMin)
      xMin = pt->x;
    else if (pt->x > xMax)
      xMax = pt->x;

    if (pt->y < yMin)
      yMin = pt->y;
    else if (pt->y > yMax)
      yMax = pt->y;
  } // end loop pt_idx

  return _Bbox(xMin, yMin, 1 + xMax - xMin, 1 + yMax - yMin);
}

} // end namespace vision_utils

#endif // BOUNDINGBOX_VEC_H
