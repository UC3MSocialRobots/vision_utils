/*!
  \file        convert_n_colors.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/11/15

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
#ifndef CONVERT_N_COLORS_H
#define CONVERT_N_COLORS_H

#include <stdlib.h>
// AD
#include "vision_utils/utils/system_utils.h"

namespace image_utils {

inline bool reduce_monochrome(const std::string & file_in,
                              const std::string & file_out) {
  // convert to a temporary file
  std::string tmp_file = "/tmp/monochrome.png";
  std::ostringstream order;
  order << "convert " << file_in << " -monochrome " << tmp_file;
  int retval = system(order.str().c_str()); // http://www.imagemagick.org/script/exception.php
  // check file size
  order.str("");
  order << "ls -l " << tmp_file << " | awk '{ print $5 }'";
  std::string size_str = system_utils::exec_system_get_output(order.str().c_str());
  // printf("size:'%s'\n", size_str.c_str());
  if (size_str[0] == '0') {
    printf("reduce_monochrome(): Could not convert '%s' to monochrome!\n", file_in.c_str());
    return false;
  }
  // printf("reduce_monochrome(): Conversion of '%s' to monochrome OK.\n", file_in.c_str());
  // move back file
  order.str("");
  order << "mv " << tmp_file << " " << file_out;
  retval = system(order.str().c_str());
  if (retval < 0) {
    printf("reduce_monochrome(): Could not write to '%s'!\n", file_out.c_str());
    return false;
  }
  return true;

}

/*! 
  Reduce the number of colors (and its size) in an image.
  From http://stackoverflow.com/questions/14031965/convert-32-bit-png-to-8-bit-png-with-imagemagick-by-preserving-semi-transparent
*/
inline bool convert_n_colors(const std::string & file_in,
                             unsigned int ncolors,
                              const std::string & file_out) {
  // convert to a temporary file
  std::string tmp_file = "/tmp/ncolors.png";
  std::ostringstream order; // convert original.png -colors n PNG8:output.png
  order << "convert " << file_in << " -colors " << ncolors
      << " PNG8:" << tmp_file;
  int retval = system(order.str().c_str()); // http://www.imagemagick.org/script/exception.php
  // check file size
  order.str("");
  order << "ls -l " << tmp_file << " | awk '{ print $5 }'";
  std::string size_str = system_utils::exec_system_get_output(order.str().c_str());
  // printf("size:'%s'\n", size_str.c_str());
  if (size_str[0] == '0') { 
    printf("convert_n_colors(): Could not convert '%s' to %i colors!\n", file_in.c_str(), ncolors);
    return false;
  }
  // printf("convert_n_colors(): Conversion of '%s' to %i colors OK.\n", file_in.c_str(), ncolors);
  // move back file
  order.str("");
  order << "mv " << tmp_file << " " << file_out;
  retval = system(order.str().c_str());
  if (retval < 0) {
    printf("convert_n_colors(): Could not write to '%s'!\n", file_out.c_str());
    return false;
  }
  return true; 
} // end convert_n_colors()

} // end namespace image_utils {
#endif // CONVERT_N_COLORS_H
