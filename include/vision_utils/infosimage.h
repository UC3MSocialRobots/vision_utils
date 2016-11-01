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

#ifndef INFOSIMAGE_H
#define INFOSIMAGE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace vision_utils {

inline std::string infosImage(const cv::Mat & i) {
  std::ostringstream concl;
  concl << "size:(" << i.cols << "x" << i.rows << "), ";
  concl << i.channels() << " channels";
  concl << ", depth:" << i.depth();
  concl << ", type:" << i.type();
  concl << ", isContinuous:" << i.isContinuous();
  return concl.str();
}

} // end namespace vision_utils

#endif // INFOSIMAGE_H

