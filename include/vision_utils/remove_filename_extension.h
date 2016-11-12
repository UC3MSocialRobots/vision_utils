/*!
  \file        remove_filename_extension.h
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

#ifndef REMOVE_FILENAME_EXTENSION_H
#define REMOVE_FILENAME_EXTENSION_H
#include <string>

namespace vision_utils {

/*!
 * Remove the extension from a filename
 * \param path
 *  the full path
 * \example
 *  "/foo/bar" -> "/foo/bar"
 *  "/foo/bar.dat" -> "/foo/bar"
 *  "/foo.zim/bar.dat" -> "/foo.zim/bar"
 *  "/foo.zim/bar" -> "/foo.zim/bar"
 */
inline std::string remove_filename_extension
(const std::string & path) {
  std::string::size_type dot_pos = path.find_last_of('.');
  if (dot_pos == std::string::npos)
    return path;
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos != std::string::npos && slash_pos > dot_pos) // dot before slash
    return path;
  return path.substr(0, dot_pos);
}

} // end namespace vision_utils

#endif // REMOVE_FILENAME_EXTENSION_H
