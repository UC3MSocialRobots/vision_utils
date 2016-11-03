/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/11/2
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

#ifndef FILE_FORMAT_H
#define FILE_FORMAT_H
#include <opencv2/highgui/highgui.hpp>

namespace vision_utils {

enum FileFormat {
  FILE_PNG = 0,
  FILE_BMP = 1,
  FILE_PPM_BINARY = 2,
  FILE_JPG = 3 // lossy, shouldnt be used
};

typedef std::vector<int> ParamsVec;

inline ParamsVec format2params(FileFormat format) {
  ParamsVec ans;
  switch (format) {
  case FILE_PNG:
    ans.push_back(CV_IMWRITE_PNG_COMPRESSION);
    ans.push_back(9);
  case FILE_PPM_BINARY:
    ans.push_back(CV_IMWRITE_PXM_BINARY);
    ans.push_back(1);
  case FILE_JPG:
    ans.push_back(CV_IMWRITE_JPEG_QUALITY);
    ans.push_back(85);
  case FILE_BMP:
  default:
  {}
  }
  return ans;
}

inline std::string format2extension(FileFormat format) {
  switch (format) {
  case FILE_BMP:
    return ".bmp";
  case FILE_PPM_BINARY:
    return ".ppm";
  default:
  case FILE_PNG:
    return ".png";
  }
  return ""; // never reached
}

} // end namespace vision_utils

#endif // FILE_FORMAT_H

