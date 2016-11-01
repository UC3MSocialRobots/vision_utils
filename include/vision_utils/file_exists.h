/*!
  \file        file_exists.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/10/30
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

\todo Description of the file
 */

#ifndef FILE_EXISTS_H
#define FILE_EXISTS_H

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

namespace vision_utils {

/*!
 \param filename
    a relative or absolute filename
 \return true if the file with given filename exists
*/
inline bool file_exists(const std::string & filename) {
  return boost::filesystem::exists(filename)
      && boost::filesystem::is_regular_file(filename);
}

} // end namespace vision_utils

#endif // FILE_EXISTS_H
