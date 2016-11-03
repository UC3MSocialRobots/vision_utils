/*!
  \file        retrieve_file_split.h
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

#ifndef RETRIEVE_FILE_SPLIT_H
#define RETRIEVE_FILE_SPLIT_H
// std includes
#include <fstream> // for std::ifstream()
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
#include <vision_utils/replace_home.h>

namespace vision_utils {

/*! get the content of a file on the hard drive
 * \param filepath for instance /foo/bar/test.txt
 * \param ans where we save the answer - one line in the vector for each file line
 */
inline bool retrieve_file_split(const std::string & filepath,
                                std::vector<std::string> & ans,
                                bool remove_empty_lines = false,
                                bool remove_empty_last_line = true) {
  //printf("retrieve_file_split('%s')", filepath.c_str());
  // open the file
  std::ifstream myfile(replace_home(filepath).c_str(), std::ios::in);
  if (!myfile.is_open()) {// error while reading the file
    printf("Unable to open file '%s'\n", filepath.c_str());
    return false;
  }
  std::string line;
  ans.clear();
  while (myfile.good()) {
    getline(myfile, line);
    if (remove_empty_lines && line.empty())
      continue;
    ans.push_back(line);
  } // end myfine.good()
  if (remove_empty_last_line && ans.back().empty())
    ans.pop_back();
  myfile.close();
  return true;
}

} // end namespace vision_utils

#endif // RETRIEVE_FILE_SPLIT_H
