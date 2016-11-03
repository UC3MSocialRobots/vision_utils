/*!
  \file        to_string.h
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

#ifndef TO_STRING_H
#define TO_STRING_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdexcept> // for invalid_argument
#include <string>

namespace vision_utils {

//#define FILENAME "image_utils_temp.yml"

/*!
 *cast a matrix to a string
 *\param mat
 *\param s its string representation
 */
inline void to_string(const cv::Mat & mat, std::string & s) {
  //printf("to_string()");
  if (mat.isContinuous() == false)
    throw std::invalid_argument("Cannot convert an image that is not continuous !");
  size_t data_length = mat.cols * mat.rows * mat.elemSize();
  std::ostringstream s_stream;
  s_stream << mat.cols << ' ' << mat.rows << ' ' << mat.type() << ' '
           << data_length << ' ';
  s_stream << std::string((char*) mat.data, data_length);
  s = s_stream.str();
}

} // end namespace vision_utils

#endif // TO_STRING_H
