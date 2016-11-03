/*!
  \file        file_io.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/31

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

The basic functions for reading and writing strings to files
on the hard drive.
 */

#ifndef FILE_IO_H
#define FILE_IO_H

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include "vision_utils/retrieve_file.h"
#include "vision_utils/save_file.h"

namespace vision_utils {

//cut:replace_home
inline std::string replace_home(const std::string & filepath) {
  std::string filepath_replaced = filepath;
  std::string::size_type home_pos = filepath_replaced.find('~');
  if (home_pos != std::string::npos) {
    std::string home = getenv("HOME");
    //find_and_replace(filepath_replaced, "~", home);
    filepath_replaced.replace(home_pos, 1, home);
  }
  return filepath_replaced;
}

////////////////////////////////////////////////////////////////////////////////
//cut:retrieve_file_split
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

////////////////////////////////////////////////////////////////////////////////
//cut:append_end_file
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

////////////////////////////////////////////////////////////////////////////////
//cut:concatenate_vector
inline std::string concatenate_vector(const std::vector<std::string> & vec) {
  //printf("concatenate_vector()");
  std::ostringstream out;
  for (std::vector<std::string>::const_iterator line = vec.begin();
       line != vec.end() ; ++line) {
    // add a newline except for first line
    if (line != vec.begin())
      out << std::endl;
    out << *line;
  }
  return out.str();
}

////////////////////////////////////////////////////////////////////////////////
//cut:save_file_split
inline void save_file_split(const std::string & filepath,
                            const std::vector<std::string> & content) {
  //printf("save_file_vec('%s')", filepath.c_str());
  save_file(filepath, concatenate_vector(content));
}
} // end namespace vision_utils
//cut
#endif // FILE_IO_H
