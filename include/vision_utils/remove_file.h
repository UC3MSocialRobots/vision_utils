/*!
  \file        remove_file.h
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

#ifndef REMOVE_FILE_H
#define REMOVE_FILE_H
// std includes
#include <string>

namespace vision_utils {

/*!
 \param filename
    a relative or absolute filename
 \return  false if filename did not exist in the first place, otherwise true.
*/
inline bool remove_file(const std::string & filename) {
  return boost::filesystem::remove(filename);
}

} // end namespace vision_utils

#endif // REMOVE_FILE_H
