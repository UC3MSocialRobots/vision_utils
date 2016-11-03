/*!
  \file        directory_exists.h
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

#ifndef DIRECTORY_EXISTS_H
#define DIRECTORY_EXISTS_H
// std includes
#include <string>
// boost
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

namespace vision_utils {

/*!
 \param directoryname
    a relative or absolute directoryname
 \see http://www.boost.org/doc/libs/1_44_0/libs/filesystem/v3/doc/reference.html
 \return true if the directory with given directoryname exists
*/
inline bool directory_exists(const std::string & directoryname) {
  return boost::filesystem::exists(directoryname)
      && boost::filesystem::is_directory(directoryname);
}

} // end namespace vision_utils

#endif // DIRECTORY_EXISTS_H
