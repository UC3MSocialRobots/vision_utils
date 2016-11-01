/*!
  \file        retrieve_file.h
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

#ifndef RETRIEVE_FILE_H
#define RETRIEVE_FILE_H

#include <fstream>

namespace vision_utils {

/*! get the content of a file on the hard drive
 * \param filepath for instance /foo/bar/test.txt
 * \param ans where we save the answer
 * \return true if success, false otherwise
 */
inline bool retrieve_file(const std::string & filepath, std::string & ans) {
  // open the file
  std::ifstream myfile(filepath.c_str(), std::ios::in);
  /*
   * check if success
   */
  if (!myfile || myfile.is_open() == false) {
    // error while reading the file
    printf("Unable to open file '%s'\n", filepath.c_str());
    return false;
  }
  /*
   * concatenate to buffer
   */
  std::string line;
  std::ostringstream buffer;
  bool first_line = true;
  while (myfile.good()) {
    // add a cariage return if it is not the first line
    if (first_line)
      first_line = false;
    else
      buffer << std::endl;

    getline(myfile, line);
    buffer << line;
  } // end myfine.good()
  myfile.close();
  ans = buffer.str();
  return true;
} // end retrieve_file()

} // end namespace vision_utils

#endif // RETRIEVE_FILE_H
