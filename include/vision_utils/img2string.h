/*!
  \file        img2string.h
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

#ifndef IMG2STRING_H
#define IMG2STRING_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

static std::string img2string(const cv::Mat1b & img) {
  std::ostringstream ans;
  ans << "(" << img.cols << "x" << img.rows << "):" << std::endl;
  int ncols = img.rows, nrows = img.rows;
  for (int row = 0; row < nrows; ++row) {
    const uchar* rowdata = img.ptr<uchar>(row);
    for (int col = 0; col < ncols; ++col)
      ans << (rowdata[col] ? 'X' : '-');
    ans << std::endl;
  }
  return ans.str();
}

} // end namespace vision_utils

#endif // IMG2STRING_H
