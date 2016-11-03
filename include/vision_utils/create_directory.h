/*!
  \file        create_directory.h
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

#ifndef CREATE_DIRECTORY_H
#define CREATE_DIRECTORY_H
// std includes
#include <string>

namespace vision_utils {

/*!
 \param directoryname
    a relative or absolute directoryname
 \see http://www.boost.org/doc/libs/1_44_0/libs/filesystem/v3/doc/reference.html
 \return true if a new directory was created, otherwise false
*/
inline bool create_directory(const std::string & directoryname) {
  return boost::filesystem::create_directory(directoryname);
}

} // end namespace vision_utils

#endif // CREATE_DIRECTORY_H
