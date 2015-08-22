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
#include <vision_utils/utils/debug_utils.h>
#include <stdint.h>
#include <stdlib.h>

namespace string_utils {

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

/*! get the content of a file on the hard drive
 * \param filepath for instance /foo/bar/test.txt
 * \param ans where we save the answer - one line in the vector for each file line
 */
inline bool retrieve_file_split(const std::string & filepath,
                                std::vector<std::string> & ans,
                                bool remove_empty_lines = false,
                                bool remove_empty_last_line = true) {
  maggieDebug3("retrieve_file_split('%s')", filepath.c_str());
  // open the file
  std::ifstream myfile(replace_home(filepath).c_str(), std::ios::in);
  if (!myfile.is_open()) {// error while reading the file
    maggiePrint("Unable to open file '%s'", filepath.c_str());
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

/*! get the content of a file on the hard drive
 * \param filepath for instance /foo/bar/test.txt
 * \param ans where we save the answer
 * \return true if success, false otherwise
 */
inline bool retrieve_file(const std::string & filepath, std::string & ans) {
  maggieDebug3("retrieve_file('%s')", filepath.c_str());
  // open the file
  std::ifstream myfile(replace_home(filepath).c_str(), std::ios::in);
  /*
   * check if success
   */
  if (!myfile || myfile.is_open() == false) {
    // error while reading the file
    maggiePrint("Unable to open file '%s'", filepath.c_str());
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

////////////////////////////////////////////////////////////////////////////////

/*! Save a string to a given file
 * \param filepath the filename where to save the content,
    for instance "/tmp/foo.txt"
 * \param content the string to save
 */
inline bool save_file(const std::string & filepath, const std::string & content) {
  // maggieDebug2("save_file('%s')", filepath.c_str());
  std::ofstream myfile(filepath.c_str());
  if (!myfile.is_open()) { // check if success
    maggiePrint("Unable to open file '%s' for writing.", filepath.c_str());
    return false;
  }
  myfile << content;
  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*! Save a string to add of file
 * \param filepath the filename where to save the content,
    for instance "/tmp/foo.txt"
 * \param content the string to save
 */
inline void append_end_file(const std::string & filepath, const std::string & ans ) {
  // maggieDebug2("save_file('%s')", filepath.c_str());
  std::ofstream myfile;
  myfile.open(filepath.c_str(),std::ofstream::out | std::ofstream::app);
  // check if success
  if (myfile.is_open()){
    myfile << ans;
    myfile.close();
  }else{// error while reading the file
    maggiePrint("Unable to open file '%s' for writing.", filepath.c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////

inline std::string concatenate_vector(const std::vector<std::string> & vec) {
  maggieDebug2("concatenate_vector()");
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

inline void save_file_split(const std::string & filepath,
                            const std::vector<std::string> & content) {
  maggieDebug2("save_file_vec('%s')", filepath.c_str());
  save_file(filepath, concatenate_vector(content));
}
} // end namespace string_utils

#endif // FILE_IO_H
