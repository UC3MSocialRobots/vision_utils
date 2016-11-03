/*!
  \file        extractLayer.h
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

#ifndef EXTRACTLAYER_H
#define EXTRACTLAYER_H
// std includes
#include <opencv2/core/core.hpp>
#include <vector>

namespace vision_utils {

/*!
 *\brief   extract one of the layer from an image
     *
 *\param   src the source
 *\param   dest the destination
 *\param   layer_idx the number of the layer (between 0 and 2)
 */
inline void extractLayer(const cv::Mat3b & src, cv::Mat1b & dest,
                         const int layer_idx) {
  // TODO check there is a copy done
#if 1
  std::vector<cv::Mat> planes;
  cv::split(src, planes);
  dest = planes.at(layer_idx);
#else
  IplImage src_ipl = src, dest_ipl = dest;
  if (layer == 0)
    cvSplit(&src_ipl, &dest_ipl, 0, 0, 0); // hue
  if (layer == 1)
    cvSplit(&src_ipl, 0, &dest_ipl, 0, 0); // saturation
  if (layer == 2)
    cvSplit(&src_ipl, 0, 0, &dest_ipl, 0); // value
#endif
}

} // end namespace vision_utils

namespace vision_utils {

} // end namespace vision_utils

#endif // EXTRACTLAYER_H
