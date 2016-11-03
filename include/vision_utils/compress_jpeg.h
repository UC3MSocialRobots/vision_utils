/*!
  \file        compress_jpeg.h
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

#ifndef COMPRESS_JPEG_H
#define COMPRESS_JPEG_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>

namespace vision_utils {

inline void compress_jpeg(const cv::Mat3b & input, const int factor,
                          std::string & ans) {
  std::vector<int> params;
  params.push_back(CV_IMWRITE_JPEG_QUALITY);
  params.push_back(factor);
  std::vector<uchar> buf;
  bool encode_OK = cv::imencode(".jpg", input, buf, params);
  if (!encode_OK)
    printf("Encoding failed !");
  ans = std::string(buf.begin(), buf.end());
}

} // end namespace vision_utils

#endif // COMPRESS_JPEG_H
