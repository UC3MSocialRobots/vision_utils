/*!
  \file        change_filename_extension.h
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

#ifndef CHANGE_FILENAME_EXTENSION_H
#define CHANGE_FILENAME_EXTENSION_H
#include <string>

namespace vision_utils {

inline std::string change_filename_extension
(const std::string & path,
 const std::string & new_extension) {
  return remove_filename_extension(path) + new_extension;
}

} // end namespace vision_utils

#endif // CHANGE_FILENAME_EXTENSION_H
