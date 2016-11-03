/*!
  \file        from_string.h
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

#ifndef FROM_STRING_H
#define FROM_STRING_H
// std includes
#include <stdexcept> // for invalid_argument
#include <string>

namespace vision_utils {

/*!
 *populate a matrix with a string
 *\param mat
 *\param s
 */
inline void from_string(cv::Mat & mat, const std::string & s) {
  //printf("from_string()");
  /*
     * with short stream 1
     */
  std::istringstream s_stream(s);
  int cols, rows, type;
  size_t data_length;
  s_stream >> cols;
  s_stream >> rows;
  s_stream >> type;
  s_stream >> data_length;
  //printf("Creating an image of size %i rows x %i cols, type:%i.", rows, cols, type);
  if (rows > 3000 || cols > 3000 || rows <= 0 || cols <= 0) {
    char error_msg[1000];
    sprintf(error_msg, "Soemthing is wrong with those dimensions %ix%i.", rows, cols);
    throw std::invalid_argument(error_msg);
  }
  mat.create(rows, cols, type);
  if (mat.isContinuous() == false)
    throw std::invalid_argument("Cannot convert to an image that is not continuous !");

  // get everything left in the stream
  int data_stream_begin_position = 1 + (int) s_stream.tellg();
  std::string::const_iterator s_ptr = s.begin();
  // go to the wanted position
  for (int var = 0; var < data_stream_begin_position; ++var)
    ++s_ptr;
  // read the data
  //printf("Ready to read %i bytes.", data_length);
  memcpy(mat.data, &s.at(data_stream_begin_position), data_length);
}

} // end namespace vision_utils

#endif // FROM_STRING_H
