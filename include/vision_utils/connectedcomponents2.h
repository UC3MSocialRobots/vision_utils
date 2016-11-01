/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/10/31
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

  odo Description of the file
 */

#ifndef CONNECTEDCOMPONENTS2_H
#define CONNECTEDCOMPONENTS2_H

#include <vision_utils/disjoint_sets2.h>

namespace vision_utils {

/*!
 *\brief   returns all the list of points making all the connected components
 *of the image
     *
 *\param   img the monochrome image
 *\param   components_pts the vector of vector of points which will
 *contain the results
 *\param   boundingBoxes the bounding boxes of the points
 */
inline void connectedComponents2(cv::Mat1b & img,
                                 std::vector< std::vector<cv::Point> > & components_pts,
                                 std::vector<cv::Rect> & boundingBoxes) {
  DisjointSets2 set(img);
  return set.get_connected_components(img.cols, components_pts, boundingBoxes);
}

} // end namespace vision_utils

#endif // CONNECTEDCOMPONENTS2_H

