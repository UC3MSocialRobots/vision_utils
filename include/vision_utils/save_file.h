/*!
  \file        save_file.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/10/29
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

#ifndef SAVE_FILE_H
#define SAVE_FILE_H

#include <fstream>

namespace vision_utils {

/*! Save a string to a given file
 * \param filepath the filename where to save the content,
    for instance "/tmp/foo.txt"
 * \param content the string to save
 */
inline bool save_file(const std::string & filepath, const std::string & content) {
  // //printf("save_file('%s')", filepath.c_str());
  std::ofstream myfile(filepath.c_str());
  if (!myfile.is_open()) { // check if success
    printf("Unable to open file '%s' for writing.\n", filepath.c_str());
    return false;
  }
  myfile << content;
  return true;
}

} // end namespace vision_utils

#endif // SAVE_FILE_H
