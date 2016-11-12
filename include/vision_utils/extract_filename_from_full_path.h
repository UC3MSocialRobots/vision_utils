/*!
  \file        extract_filename_from_full_path.h
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

#ifndef EXTRACT_FILENAME_FROM_FULL_PATH_H
#define EXTRACT_FILENAME_FROM_FULL_PATH_H
#include <string>

namespace vision_utils {

/*!
 \param path
 \return std::string
 \example path="/tmp/foo/bar.dat", returns "bar.dat"
 \example path="/bar.dat", returns "bar.dat"
 \example path="bar.dat", returns "bar.dat"
*/
inline std::string extract_filename_from_full_path(const std::string & path) {
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos == std::string::npos)
    return path;
  return path.substr(slash_pos + 1);
} // end extract_filename_from_full_path()

} // end namespace vision_utils

#endif // EXTRACT_FILENAME_FROM_FULL_PATH_H
