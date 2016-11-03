/*!
  \file        append_end_file.h
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

#ifndef APPEND_END_FILE_H
#define APPEND_END_FILE_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

/*! Save a string to add of file
 * \param filepath the filename where to save the content,
    for instance "/tmp/foo.txt"
 * \param content the string to save
 */
inline void append_end_file(const std::string & filepath, const std::string & ans ) {
  // //printf("save_file('%s')", filepath.c_str());
  std::ofstream myfile;
  myfile.open(filepath.c_str(),std::ofstream::out | std::ofstream::app);
  // check if success
  if (myfile.is_open()){
    myfile << ans;
    myfile.close();
  }else{// error while reading the file
    printf("Unable to open file '%s' for writing.\n", filepath.c_str());
  }
}

} // end namespace vision_utils

#endif // APPEND_END_FILE_H
