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
#include <opencv2/core/core.hpp>
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

////////////////////////////////////////////////////////////////////////////////

/*!
 * It’s possible to serialize this through the OpenCV I/O XML/YAML interface
 *(just as in case of the OpenCV data structures) by adding a read and
 * a write function inside and outside of your class. Cf:
 * http://docs.opencv.org/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html
 * \param fs
 *  the stream
 * \param x
 *  the value to serialize
 */
template<class _T>
void write(cv::FileStorage & fs, const std::string &, const _T& x) {
  x.write(fs);
} // end write()

////////////////////////////////////////////////////////////////////////////////

/*!
 * It’s possible to serialize this through the OpenCV I/O XML/YAML interface
 *(just as in case of the OpenCV data structures) by adding a read and
 * a write function inside and outside of your class. Cf:
 * http://docs.opencv.org/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html
 * \param node
 * \param x
 * \param default_value
 *
 */
template<class _T>
void read(const cv::FileNode & node, _T & x, const _T& default_value = _T()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
} //end read()

} // end namespace vision_utils

#endif // IMWRITE_DEBUG_H
