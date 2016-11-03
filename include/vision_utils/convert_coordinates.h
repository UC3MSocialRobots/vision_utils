/*!
  \file        convert_coordinates.h
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

#ifndef CONVERT_COORDINATES_H
#define CONVERT_COORDINATES_H

namespace vision_utils {

/*!
  Converts a point from a coordinate system to another
  \param bbox_src the bounding box of the source system
  \param bbox_dst the bounding box of the dest system
  \param query the point in the source system
  \return the point with coordinates in the dest system
  */
template<class _Bbox_src, class _Bbox_dst,
         class _Point2_src, class _Point2_dst>
static inline _Point2_dst
convert_coordinates(const _Bbox_src & bbox_src,
                    const _Bbox_dst & bbox_dst,
                    const _Point2_src & query
                    ) {
  //    //printf("bbox:%f+%f -> %i+%i, diff:%f",
  //                 bbox_src.x, bbox_src.width,
  //                 bbox_dst.x, bbox_dst.width,
  //                 query.x - bbox_src.x);
  _Point2_dst ans(
        bbox_dst.x +
        (query.x - bbox_src.x) * bbox_dst.width / bbox_src.width,
        bbox_dst.y +
        (query.y - bbox_src.y) * bbox_dst.height / bbox_src.height
        );
  return ans;
}

} // end namespace vision_utils

#endif // CONVERT_COORDINATES_H
