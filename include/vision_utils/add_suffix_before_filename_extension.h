/*!
  \file        add_suffix_before_filename_extension.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/12
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

#ifndef ADD_SUFFIX_BEFORE_FILENAME_EXTENSION_H
#define ADD_SUFFIX_BEFORE_FILENAME_EXTENSION_H
#include <sstream>
#include <string>

namespace vision_utils {

/*!
  Add a suffix to a filename
 \param path
    The absolute or relative path to filename
 \param suffix
    The string to be added to path, before the file extension
 \return std::string
 \example ("/foo/bar.dat", "_out") returns "/foo/bar_out.dat"
          ("/foo/bar", "_out") returns "/foo/bar_out"
*/
inline std::string add_suffix_before_filename_extension
(const std::string & path, const std::string & suffix = "out") {
  std::string::size_type dot_pos = path.find_last_of('.');
  std::ostringstream out;
  if (dot_pos == std::string::npos) {
    out << path << suffix;
  }
  else {
    std::string path_before_dot = path.substr(0, dot_pos);
    std::string path_after_dot = path.substr(dot_pos + 1);
    out << path_before_dot << suffix << "." << path_after_dot;
  }
  return out.str();
} // end add_suffix_before_filename_extension()

} // end namespace vision_utils

#endif // ADD_SUFFIX_BEFORE_FILENAME_EXTENSION_H
