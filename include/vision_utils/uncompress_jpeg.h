/*!
  \file        uncompress_jpeg.h
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

#ifndef UNCOMPRESS_JPEG_H
#define UNCOMPRESS_JPEG_H
// std includes
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

namespace vision_utils {

inline void uncompress_jpeg(cv::Mat3b & output, const std::string & ans) {
  std::vector<uchar> ans_vector (ans.begin(), ans.end());
  output = cv::imdecode(cv::Mat (ans_vector), -1);
  //printf("output:%s", infosImage(output).c_str());
}

} // end namespace vision_utils

#endif // UNCOMPRESS_JPEG_H
