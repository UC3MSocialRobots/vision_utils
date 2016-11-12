/*!
  \file        imwrite_debug.h
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

#ifndef IMWRITE_DEBUG_H
#define IMWRITE_DEBUG_H
// std includes
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
#include "vision_utils/convert_n_colors.h"

namespace vision_utils {

enum NbColors {
  COLOR_24BITS = 0,
  COLORS256 = 1,
  MONOCHROME = 2
};

inline bool imwrite_debug(const std::string& filename, cv::InputArray img,
                          NbColors ncolors = COLOR_24BITS,
                          const std::vector<int>& params=std::vector<int>()) {
  if (!cv::imwrite(filename, img, params)) {
    printf("/!\\ Could not write file '%s'\n", filename.c_str());
    return false;
  }
  // color reduction
  if (ncolors == COLORS256 && !convert_n_colors(filename, 256, filename)) {
    printf("/!\\ Could not reduce file '%s' to 256 colors \n", filename.c_str());
    return false;
  }
  else if (ncolors == MONOCHROME && !reduce_monochrome(filename, filename)) {
    printf("/!\\ Could not reduce file '%s' to monochrome \n", filename.c_str());
    return false;
  }
  printf("Succesfully written file '%s'\n", filename.c_str());
  return true;
} // end imwrite_debug()

} // end namespace vision_utils

#endif // IMWRITE_DEBUG_H
