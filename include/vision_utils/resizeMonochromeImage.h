/*!
  \file        resizeMonochromeImage.h
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

#ifndef RESIZEMONOCHROMEIMAGE_H
#define RESIZEMONOCHROMEIMAGE_H
// std includes
#include <opencv2/imgproc/imgproc.hpp>

namespace vision_utils {

/*!
 *\brief   resize a monochroome image in another one -
     *
 *\param   src monochrome image
 *\param   dest monochrome image
 */
inline void resizeMonochromeImage(const cv::Mat1b & src, cv::Mat1b & dest) {
  //printf("resizeMonochromeImage('%s', '%s'')", infosImage(src).c_str(), infosImage(dest).c_str());
  cv::resize(src, dest, dest.size(), 0, 0, cv::INTER_NEAREST);
}

} // end namespace vision_utils

#endif // RESIZEMONOCHROMEIMAGE_H
